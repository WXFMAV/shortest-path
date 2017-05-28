/*
 * PlannerTracking.cpp
 *
 *  Created on: 2017年4月19日
 *      Author: Hualin He
 */

#include "PlannerTracking.h"
#include <glog/logging.h>

using namespace Eigen;
using namespace iarc_arena_simulator;

#define state_n  9
#define list_capacity  PARAM::list_capacity

PlannerTracking::PlannerTracking() {
    // TODO Auto-generated constructor stub
    _status = ERROR;
    _state_x.setZero();
    _control_u.setZero();
    _controller_model = none;
    _controller = nullptr;
    _pos_now = 0;
    _pos_end = -1;
    _wp_list.clear();
    //_path_list.clear();
    //_time_list.clear();
    _fp_tracking = NULL;
    _seq_saved = -1;
}

PlannerTracking::~PlannerTracking() {
    // TODO Auto-generated destructor stub
    _status = ERROR;
    if (_controller != nullptr){
        delete _controller;
    }
    if (_fp_tracking != NULL){
        fclose(_fp_tracking);
        _fp_tracking = NULL;
    }
}

int PlannerTracking::init()
{
    _controller_model = lqr;
    _controller = new G_Controller_PID;
    if (_controller->init() < 0){
        LOG(ERROR) << "build controller failed!";
        return -1;
    }
    else{
        LOG(INFO) << "build controller success";
    }

    _fp_tracking = fopen(PARAM::file_name_tracking.c_str(),"w");
    if ( _fp_tracking ==NULL){
        LOG(ERROR) << "can not open file: "<<PARAM::file_name_tracking.c_str();
        return -1;
    }
    _status = OK;
    _state_x.setZero(state_n);
    //_path_list.resize(list_capacity);
    //_time_list.resize(list_capacity);
    _wp_list.resize(list_capacity);
    _seq_saved = -1;
    _pos_now = 0;
    _pos_end = 0;
    return 0;
}
int PlannerTracking::update_quad(Quadrotor *theQuad)
{
    //get system feedback.
    double x[state_n];
    theQuad->get_pos(x[0], x[1], x[2]);
    theQuad->get_vel(x[3], x[4], x[5]);
    theQuad->get_acc(x[6], x[7], x[8]);
   // theQuad->get_quat(x[9], x[10], x[11], x[12]);
    for (int k=0; k<state_n; k++){
        _state_x[k] = x[k];
    }
    LOG(INFO) << "update_quad ok!";
    return 0;
}
int PlannerTracking::update_waypoints_list(const iarc_arena_simulator::IARCWaypointsList::ConstPtr& waypoints_list)
{
    int m = ((iarc_arena_simulator::IARCWaypointsList)(*waypoints_list)).list.size();
#define wplist ((iarc_arena_simulator::IARCWaypointsList)(*waypoints_list)).list

    uint32_t time_now = _wp_list[_pos_now % list_capacity].tms;
    uint32_t add_k = 0;

    for (int k = 0; k<m; k++){
        if (wplist[k].tms > time_now){
            add_k++;
            _wp_list[(_pos_now + add_k) % list_capacity] = wplist[k];

            VectorXd wp(state_n);
            wp[0] = wplist[k].x; wp[1] = wplist[k].y; wp[2] = wplist[k].z;
            wp[3] = wplist[k].vx; wp[4] = wplist[k].vy; wp[5] = wplist[k].vz;
            wp[6] = wplist[k].ax; wp[7] = wplist[k].ay; wp[8] = wplist[k].az;

            LOG (INFO) <<"wp seq=" <<wplist[k].seq<< "wp time="<< wplist[k].tms << " wp="<< wp.transpose();
        }
    }
    if (add_k != 0)
    {
        //a new path list is generated.
        _pos_end = _pos_now + add_k;
    }
    if ( _pos_end - _pos_now > list_capacity){
        LOG(ERROR) << "list capacity too small , or waypoint list to large to be stored!";
        _pos_now = _pos_end - list_capacity;
    }
    LOG(INFO) << "waypoint list received! size=" <<m <<" _pos_now = "<< _pos_now <<" _pos_end = "<<_pos_end;
#undef wplist
    return 0;
}
Eigen::VectorXd PlannerTracking::get_vector_from_waypoint(const iarc_arena_simulator::IARCWaypoint &wp)
{
    VectorXd v;
    v.setZero(state_n);
    v[0] = wp.x; v[1] = wp.y; v[2] = wp.z;
    v[3] = wp.vx; v[4] = wp.vy; v[5] = wp.vz;
    v[6] = wp.ax; v[7] = wp.ay; v[8] = wp.az;
    return v;
}
iarc_arena_simulator::IARCWaypoint  PlannerTracking::get_waypoint_now()
{
    uint32_t time_now = arena_time_now();

    if (_pos_now <=  _pos_end && time_now > _wp_list[_pos_now % list_capacity].tms){   //move to next waypoint
            _pos_now++;
    }
    if(_pos_now > _pos_end){ //generate current state as target waypoint.
        LOG(WARNING) << "waypoint list is empty!";
        //const std::string &frame_id, const ros::Time &stamp, uint32_t seq,
        _wp_list[_pos_now % list_capacity ] = make_waypoint(PARAM::str_arena_frame, ros::Time::now(), 0,
                time_now, _state_x, turn_none, robot_arena);
        _pos_end = _pos_now;
    }

    IARCWaypoint wp_now = _wp_list[_pos_now % list_capacity];
    LOG(INFO) <<"path list accept! pos_now= " <<_pos_now <<" pos_end = "<< _pos_end  <<"seq =" << wp_now.seq<<" now time ="<<time_now;

    return wp_now;
}
int PlannerTracking::gen_control_cmd(IARC_COMMAND &cmd)
{
    uint32_t time_now = arena_time_now();
    IARCWaypoint wp = get_waypoint_now();
    VectorXd wp_now = get_vector_from_waypoint(wp);
    VectorXd err_x = _state_x - wp_now ;
    VectorXd ref_x = VectorXd::Zero(state_n);
//    double tf = (double)( _time_list[_pos_now % list_capacity] - time_now)/1000.0;
    LOG(INFO) << "set_ref";
    _controller->set_reference(ref_x);  //zero for we use LQR regulator.
    _controller->set_feedback(err_x);  //err_x  is the state x in controller.
    LOG(INFO) << "get_control";
    _control_u = _controller->get_control();
   // ( (G_Controller_LQR*)_controller)->set_tf(tf);
  //  _control_u =( (G_Controller_LQR*)_controller)->get_control();

    cmd.mav_vx = _control_u[0];
    cmd.mav_vy = _control_u[1];
    cmd.mav_vz = _control_u[2];
    cmd.mav_vyawdegree = 0.0;

    if (wp.seq != _seq_saved)
    {
    	cmd.robot_turn_id = wp.robot_id;
    	cmd.robot_turn_kind = wp.robot_cmd;
    }
    else
    {
    	cmd.robot_turn_id = robot_arena;
    	cmd.robot_turn_kind = turn_none;
    }
    _seq_saved = wp.seq;
    LOG(INFO) <<"gen_control_cmd ok";

    if ( _fp_tracking != NULL){
        fprintf(_fp_tracking, "%d %d %d %d (%.2lf, %.2lf, %.2lf) (%.2lf, %.2lf, %.2lf) (%.2lf, %.2lf, %.2lf)\
  (%.2lf, %.2lf, %.2lf) (%.2lf, %.2lf, %.2lf) (%.2lf, %.2lf, %.2lf)\
  (%.2lf, %.2lf, %.2lf %.2lf) id-kind(%d, %d)\n",
                time_now, wp.seq,  _pos_now,  _wp_list[_pos_now % list_capacity].tms,
                _state_x[0], _state_x[1], _state_x[2],
                _state_x[3], _state_x[4], _state_x[5],
                _state_x[6], _state_x[7], _state_x[8],
                wp_now[0], wp_now[1], wp_now[2],
                wp_now[3], wp_now[4], wp_now[5],
                wp_now[6], wp_now[7], wp_now[8],
                _control_u[0], _control_u[1], _control_u[2], cmd.mav_vyawdegree,
				cmd.robot_turn_id, cmd.robot_turn_kind);
    }
    return 0;
}
