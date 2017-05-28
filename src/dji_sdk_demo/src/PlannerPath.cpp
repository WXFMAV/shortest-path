/*
 * PlannerPath.cpp
 *
 *  Created on: 2017年4月19日
 *      Author: Hualin He
 */

#include "PlannerPath.h"
#include "common.h"
#include"G_Shortest_Path.h"
#include "G_Bazier.h"
#include <math.h>
#include <glog/logging.h>

using namespace Eigen;
using namespace std;
using namespace iarc_arena_simulator;

#define waypoint_dT_ms  PARAM::waypoint_dT_ms
#define taskslist_capacity PARAM::taskslist_capacity
#define task_period_ms  PARAM::task_period_ms

PlannerPath::PlannerPath() {
    // TODO Auto-generated constructor stub
    _status = ERROR;
    _planner = test;
    _wp_seq = 0;
    memset(&_quad_status, 0, sizeof(_quad_status));
    _tgt_status.clear();
    _obs_status.clear();

    _fp_path = NULL;
    _fp_path_env = NULL;
    _pos_now = 1;
    _pos_end = 0;
    _task_seq_saved = -1;
    _tasks_list.clear();
}

PlannerPath::~PlannerPath() {
    // TODO Auto-generated destructor stub
    if (_fp_path != NULL){
        fclose(_fp_path);
        _fp_path = NULL;
    }
    if(_fp_path_env != NULL){
    	fclose(_fp_path_env);
    	_fp_path_env = NULL;
    }
}

int PlannerPath::init()
{
    _status = OK;
    _planner = PureTracking;
    //_planner = BiBFS;
    _wp_seq = 0;
    memset(&_quad_status, 0, sizeof(_quad_status));
    _tgt_status.clear();
    _obs_status.clear();
    _pos_now = 1;
    _pos_end = 0;
    _task_seq_saved = -1;
    _tasks_list.resize(taskslist_capacity);

    _fp_path = fopen(PARAM::file_name_path.c_str(),"w");
    if( _fp_path ==NULL){
        LOG(ERROR) << "open file error: "<< PARAM::file_name_path.c_str();
        return -1;
    }

    _fp_path_env = fopen(PARAM::file_name_path_env.c_str(),"w");
    if(_fp_path_env == NULL){
    	LOG(ERROR)  << "open file error: "<<PARAM::file_name_path_env.c_str();
    	return -1;
    }
    return 0;
}
bool PlannerPath::ready_for_plan()
{
    if (_quad_status.z < 0.1) return false;

    return true;
}
int PlannerPath::plan(iarc_arena_simulator::IARCWaypointsList &wplist)
{
    bool ready = false;
    wplist.list.clear();
    ready = ready_for_plan();

    if( !ready){
        LOG(INFO) << "system not ready for path planning!";
        return -1;
    }

    switch(_planner)
    {
    case test:{
        //planner_test(wplist);
    }break;
    case A_star:{
        LOG(ERROR) << "can not found the code!";
    }break;
    case BiBFS:{
        plan_with_obs(wplist);
        //LOG(ERROR) << "can not found the code!";
    }break;
    case PureTracking:{
         plan_PureTracking(wplist);
    }break;
    }

    LOG(INFO) << "plan finished wplist size = "<<wplist.list.size() << "time now= "<<arena_time_now();
    if (_fp_path != NULL){
        uint32_t time_now = arena_time_now();
        fprintf(_fp_path , "%d %d ", time_now, wplist.list.size());
        for(int k = 0; k< wplist.list.size(); k++){
            fprintf(_fp_path, "( %d %d %d (%.2lf %.2lf %.2lf) (%.2lf %.2lf %.2lf) (%.2lf %.2lf %.2lf)) ",
                    k, wplist.list[k].seq, wplist.list[k].tms,
                    wplist.list[k].x, wplist.list[k].y, wplist.list[k].z,
                    wplist.list[k].vx, wplist.list[k].vy, wplist.list[k].vz,
                    wplist.list[k].ax, wplist.list[k].ay, wplist.list[k].az);
        }
        fprintf(_fp_path,"\n");
    }
    return 0;
}
int PlannerPath::planner_test(iarc_arena_simulator::IARCWaypointsList &wplist)
{
    //Generate an example path for the targets to follow.
    //from current state now
    vector<double> x, y, z, timespace;
    uint32_t time_now = arena_time_now()+200; // plan the path after 200ms
    uint32_t time_end = time_now + 200;  //just plan for 200ms
    double t0 = ((double) time_now) / 1000.0;
    double tf = ((double) time_end) / 1000.0;
    double dT = ((double) waypoint_dT_ms) / 1000.0;
    double x0 = _quad_status.x;
    double y0 = _quad_status.y;
    double z0 = _quad_status.z;
    timespace.clear();
    x.clear();
    y.clear();
    z.clear();

    for (int k = 0; ; k++){
        double t = t0 + ((double)k) * dT;
        if ( t > tf ){
            break;
        }
        timespace.push_back(t);

        x.push_back(sin(t * 2 * M_PI/12.0) * 2.0  );
        y.push_back(cos(t * 2 * M_PI/12.0) * 2.0 );
        z.push_back(sin(t * 2 * M_PI/4.0)+2.0) ;
        /*
        x.push_back(sin(t * 2 * M_PI/12.0) * 2.0 - sin(t0 * 2 * M_PI/12.0) * 2.0 + x0);
        y.push_back(cos(t * 2 * M_PI/12.0) * 2.0 - cos(t0 * 2 * M_PI/12.0) * 2.0 + y0);
        z.push_back(sin(t * 2 * M_PI/12.0)+2.0 - sin(t0 * 2 * M_PI/12.0)+ z0);
        */
    }

    int sz = timespace.size();
    vector<double> vx = get_diff(x, timespace);
    vector<double> vy = get_diff(y, timespace);
    vector<double> vz = get_diff(z, timespace);
    vector<double> ax = get_diff(vx, timespace);
    vector<double> ay = get_diff(vy, timespace);
    vector<double> az = get_diff(vz, timespace);

    wplist.list.clear();
    wplist.header.frame_id = "/iarc_arena";
    wplist.header.stamp = ros::Time::now();
    for (int k = 0; k<sz; k++){
        iarc_arena_simulator::IARCWaypoint wp;
        wp.header = wplist.header;
        wp.seq = ++_wp_seq;
        wp.tms = (uint32_t)(timespace[k] *1000.0);
        wp.x = x[k];
        wp.y = y[k];
        wp.z = z[k];
        wp.vx = vx[k];
        wp.vy = vy[k];
        wp.vz = vz[k];
        wp.ax = ax[k];
        wp.ay = ay[k];
        wp.az = az[k];
        wplist.list.push_back(wp);
    }
    return 0;
}
vector<double> PlannerPath::get_diff(std::vector<double> a, std::vector<double> time)
{
    vector<double> da;
    da.clear();
    int sz = a.size();
    if (sz<2) return da;

    for (int k = 0; k < sz-1; k++)
    {
        da.push_back( (a[k + 1] - a[k]) / (time[k + 1] - time[k]));
    }
    da.push_back(da[sz-2]);
    return da;
}

int PlannerPath::update_taskslist(const iarc_arena_simulator::IARCTasksList::ConstPtr &taskslist)
{

    int m = ((iarc_arena_simulator::IARCTasksList)(*taskslist)).list.size();
 #define taskslist ((iarc_arena_simulator::IARCTasksList)(*taskslist)).list

     uint32_t time_now = arena_time_now();
     uint32_t add_k = 0;

     for (int k = 0; k<m; k++){
         if (taskslist[k].time_end > time_now){
             _tasks_list[(_pos_now+add_k)% taskslist_capacity] = taskslist[k];

             if ( _tasks_list[(_pos_now+add_k)% taskslist_capacity].time_start < time_now){
            	 _tasks_list[(_pos_now+add_k)% taskslist_capacity].time_start = time_now;
             }

             _tasks_list[(_pos_now+add_k)% taskslist_capacity].time_start =
                     (_tasks_list[(_pos_now+add_k)% taskslist_capacity].time_start >time_now?
                             _tasks_list[(_pos_now+add_k)% taskslist_capacity].time_start :time_now);
             LOG (INFO) <<"pos = "<<_pos_now + add_k<< " task seq=" <<taskslist[k].task_seq<< "start time="
                     <<_tasks_list[(_pos_now+add_k)% taskslist_capacity].time_start  << " end time="
                     <<_tasks_list[(_pos_now+add_k)% taskslist_capacity].time_end <<" task type="
                     <<(uint32_t)_tasks_list[(_pos_now+add_k)% taskslist_capacity].task_type <<" task value="
                     <<_tasks_list[(_pos_now+add_k)% taskslist_capacity].task_value<<"robot id ="
                     <<_tasks_list[(_pos_now+add_k)% taskslist_capacity].robot_id<< "robot cmd="
                     <<_tasks_list[(_pos_now+add_k)% taskslist_capacity].robot_cmd<<"pos =("
                     <<_tasks_list[(_pos_now+add_k)% taskslist_capacity].final_pose.position.x<< ","
                     <<_tasks_list[(_pos_now+add_k)% taskslist_capacity].final_pose.position.y<<","
                     <<_tasks_list[(_pos_now+add_k)% taskslist_capacity].final_pose.position.z<<")";
             add_k++;
         }
     }
     if (add_k != 0)
     {
         //a new path list is generated.
         _pos_end = _pos_now + add_k-1;
     }
     if ( _pos_end - _pos_now > taskslist_capacity){
         LOG(ERROR) << "list capacity too small , or tasks list too large to be stored!";
         _pos_now = _pos_end - taskslist_capacity;
     }
     LOG(INFO) << "tasks list received! size=" <<m <<" _pos_now = "<< _pos_now <<" _pos_end = "<<_pos_end;
 #undef taskslist

    return 0;
}

iarc_arena_simulator::IARCTask PlannerPath::get_task_now()
{
    uint32_t time_now = arena_time_now();
       if (_pos_now <=  _pos_end && time_now > _tasks_list[_pos_now % taskslist_capacity].time_end){   //move to next waypoint
               _pos_now++;
       }
       if(_pos_now > _pos_end){ //generate current state as target waypoint.
           LOG(WARNING) << "tasks list is empty!";
           // make_task_type_reach(std::string frame_id, ros::Time stamp, double aimx,
           //double aimy, double aimz, uint32_t start_time_ms, uint32_t aim_time_ms);
           std::string frame_id = PARAM::str_arena_frame;
           ros::Time stamp = ros::Time::now();
           _tasks_list[_pos_now % taskslist_capacity] = make_task_type_reach(frame_id, stamp,
                   _quad_status.x, _quad_status.y, _quad_status.z, time_now, time_now + task_period_ms);
           _pos_end = _pos_now;
           _tasks_list[_pos_now % taskslist_capacity].task_seq = -1;
       }

       iarc_arena_simulator::IARCTask task_now = _tasks_list[_pos_now % taskslist_capacity];

       LOG(INFO) <<"task accept! pos_now= " <<_pos_now <<" pos_end = "<< _pos_end
               <<"task seq="<<task_now.task_seq<< "start time="<<task_now.time_start
               << " end time="<<task_now.time_end
               <<" task type=" <<::str_task_type[(uint32_t)task_now.task_type]
               <<" task value="<<task_now.task_value
               <<"robot id ="<<task_now.robot_id
               <<"pos =("<<task_now.final_pose.position.x
               << "," <<task_now.final_pose.position.y
               <<","  <<task_now.final_pose.position.z<<")";
       return task_now;
}

int PlannerPath::smooth_waypoints(std::vector<iarc_arena_simulator::IARCWaypoint> &list)
{
    int sz = list.size();
   if(sz<2) return -1;

    for ( int k = 0; k < sz-1; k++){
        double detT = ((double)(list[k+1].tms - list[k].tms))/1000.0;
        if( fabs(detT) < 1e-6 ){
            LOG(ERROR) << "detT = 0.0 ";
        }
        list[k].vx = (list[k + 1].x - list[k].x) / detT;
        list[k].vy = (list[k + 1].y - list[k].y) / detT;
        list[k].vz = (list[k + 1].z - list[k].z) / detT;
    }
    list[sz - 1].vx = list[sz - 2].vx;
    list[sz - 1].vy = list[sz - 2].vy;
    list[sz - 1].vz = list[sz - 2].vz;

    for ( int k = 0; k < sz-1; k++){
            double detT = ((double)(list[k+1].tms - list[k].tms))/1000.0;
            list[k].ax = (list[k + 1].vx - list[k].vx) / detT;
            list[k].ay = (list[k + 1].vy - list[k].vy) / detT;
            list[k].az = (list[k + 1].vz - list[k].vz) / detT;
        }
    list[sz - 1].ax = list[sz - 2].ax;
    list[sz - 1].ay = list[sz - 2].ay;
    list[sz - 1].az = list[sz - 2].az;

    return 0;
}

int PlannerPath::add_one_waypoint( std::vector<iarc_arena_simulator::IARCWaypoint> &list, uint32_t time_ms,
            double x, double y, double z, double vx, double vy, double vz ,double ax, double ay, double az,
            robot_id id, robotcmd_kind cmd)
{
    IARCWaypoint  wp;
    wp.seq = _wp_seq++;
    wp.header.frame_id = PARAM::str_arena_frame;
    wp.header.stamp = ros::Time::now();
    wp.tms = time_ms;
    wp.x = x; wp.y = y; wp.z = z;
    wp.vx = vx; wp.vy = vy; wp.vz = vz;
    wp.ax = ax; wp.ay = ay; wp.az = az;
    wp.robot_id = id; wp.robot_cmd = cmd;
    list.push_back(wp);
    return 0;
}

int PlannerPath::plan_PureTracking(iarc_arena_simulator::IARCWaypointsList &wplist)
{
    bool updated = false;
    IARCTask task= get_task_now();
    IARCWaypoint wp;
    wp.tms = arena_time_now();
    wp.x = _quad_status.x;
    wp.y = _quad_status.y;
    wp.z = _quad_status.z;
    wp.robot_id = robot_id::robot_arena;
    wp.robot_cmd = robotcmd_kind::turn_none;

    if(_fp_path_env != NULL){
    	fprintf(_fp_path_env, "\nPlanPath\n");
    	fprintf(_fp_path_env, "time= %d taskseq= %d tasktype= %s robotid= %d cmdtype= %s turntime=%d\n",
    			arena_time_now(), task.task_seq, ::str_task_type[task.task_type], task.robot_id, ::str_kind_turn[task.robot_cmd], get_time_sendcmd(task));
    }

    switch(task.task_type)
    {
    case type_hover:
    {
          if(task.task_seq != _task_seq_saved)
          {
                wplist.list.clear();
                add_one_waypoint(wplist.list, task.time_end,
                        (double)_quad_status.x, (double)_quad_status.y, (double)_quad_status.z,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        robot_arena, turn_none);
                _task_seq_saved = task.task_seq;
                updated = true;
          }
    }      break;
    case type_reach: case type_cruise:
    {
          if(task.task_seq != _task_seq_saved)
          {
        	  double aimx, aimy, aimz;
                wplist.list.clear();
                uint32_t dT = 200;
                uint32_t new_dT;
                uint32_t det = task.time_end - task.time_start;
                int kpath =  det / dT + 1;
                new_dT = det / kpath;

                aimx = task.final_pose.position.x;//_tgt_status[task.robot_id].x;
                aimy = task.final_pose.position.y;//_tgt_status[task.robot_id].x;
                aimz = task.final_pose.position.z;
                add_one_waypoint(wplist.list, task.time_end - (kpath - 1) * (new_dT),
                        (double)(_quad_status.x + (aimx - _quad_status.x) / ((double)kpath)),
                        (double)(_quad_status.y + (aimy - _quad_status.y) / ((double)kpath)),
                        (double)(_quad_status.z + (aimz - _quad_status.z) / ((double)kpath)),
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        robot_arena, turn_none);

                for(int k = 1; k < kpath; k++)
                {
                     add_one_waypoint(wplist.list, wplist.list[0].tms + k * new_dT,
                             (double)(wplist.list[0].x + (double(k)) * (aimx - _quad_status.x) / ((double)kpath)),
                             (double)(wplist.list[0].y + (double(k)) * (aimy - _quad_status.y) / ((double)kpath)),
                             (double)(wplist.list[0].z + (double(k)) * (aimz - _quad_status.z) / ((double)kpath)),
                             0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0,
                             robot_arena, turn_none);
                }
                _task_seq_saved = task.task_seq;
                updated = true;
          }
    }     break;

    case type_follow:
    {
    	IARCRobot r = get_robot_by_id(task.robot_id);
    	if ( r.id != robot_none  && r.id != robot_arena && r.belief_rate > 0.0)
          {
        	  double aimx, aimy, aimz;
                wplist.list.clear();
                uint32_t dT = 200;
                uint32_t new_dT;
                uint32_t det = task.time_end - task.time_start;
                int kpath =  det / dT + 1;
                new_dT = det / kpath;
                aimx =r.x;//_tgt_status[task.robot_id].x;
                aimy = r.y;//_tgt_status[task.robot_id].x;
                aimz = task.final_pose.position.z;
                add_one_waypoint(wplist.list, task.time_end - (kpath - 1) * (new_dT),
                        (double)(_quad_status.x + (aimx - _quad_status.x) / ((double)kpath)),
                        (double)(_quad_status.y + (aimy - _quad_status.y) / ((double)kpath)),
                        (double(_quad_status.z + (aimz - _quad_status.z) / ((double)kpath))),
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        (robot_id)task.robot_id, turn_none);

                for(int k = 1; k < kpath; k++)
                {
                     add_one_waypoint(wplist.list, wplist.list[0].tms + k * (new_dT),
                             (double)(wplist.list[0].x + (double(k)) * (aimx - _quad_status.x) / ((double)kpath)),
                             (double)(wplist.list[0].y + (double(k)) * (aimy - _quad_status.y) / ((double)kpath)),
                             (double)(wplist.list[0].z + (double(k)) * (aimz - _quad_status.z) / ((double)kpath)),
                             0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0,
                             (robot_id)task.robot_id, turn_none);
                }
                _task_seq_saved = task.task_seq;
                updated = true;
          }
          else
          {
             LOG(ERROR) <<"error! follow robot ID error!";
          }
    } break;
    case type_interact:
    {
    	IARCRobot r = get_robot_by_id(task.robot_id);
    	if ( r.id != robot_none  && r.id != robot_arena && r.belief_rate > 0.0)
        {
        	double aimx, aimy, aimz;
            wplist.list.clear();
            uint32_t timenow = arena_time_now();
            uint32_t timestart = timenow; //( timenow > task.time_start ? timenow : task.time_start);
            uint32_t time_action = get_time_sendcmd(task);

            if( time_action < timestart){
            	LOG(INFO) << "time exceed!";
            	time_action = timestart;
            }

            uint32_t dT = 200;
            uint32_t new_dT;
            uint32_t det = time_action - timestart;
            int kpath =  det / dT + 1;
            new_dT = det / kpath;

      		IARCRobot r2;
      		static IARCRobot r2_saved;

      		if( (timenow) % PARAM::period_turn_ms > PARAM::period_turn_ms - PARAM::time_turn_180_ms - 1000
      				|| (timenow ) % PARAM::period_turn_ms < 1000
					|| (timenow) > time_action){
      			r2 = r2_saved;
      		}
      		else{
      			r2 = ::make_prediction_robot(time_action, r, 0);
      		}

      		r2_saved = r2;

              aimx = r2.x;//_tgt_status[task.robot_id].x;
              aimy = r2.y;//_tgt_status[task.robot_id].x;
              aimz = task.final_pose.position.z;
              add_one_waypoint(wplist.list, time_action - (kpath - 1) * (new_dT),
                      (double)(_quad_status.x + (aimx - _quad_status.x) / ((double)kpath)),
                      (double)(_quad_status.y + (aimy - _quad_status.y) / ((double)kpath)),
                      (double)(_quad_status.z + (aimz - _quad_status.z) / ((double)kpath)),
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      (robot_id)task.robot_id, turn_none);

              for(int k = 1; k < kpath; k++)
              {
                   add_one_waypoint(wplist.list, wplist.list[0].tms + k * (new_dT),
                           (double)(wplist.list[0].x + (double(k)) * (aimx- _quad_status.x) / ((double)kpath)),
                           (double)(wplist.list[0].y + (double(k)) * (aimy - _quad_status.y) / ((double)kpath)),
                           (double)(wplist.list[0].z + (double(k)) * (aimz - _quad_status.z) / ((double)kpath)),
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0,
                           (robot_id)task.robot_id, turn_none);
              }
              //calculate task.time_end - time_turn at wiche pos and then send a turn cmd at that time..
              uint32_t time_turn = time_action;
              uint32_t kind_turn = task.robot_cmd;

              int turn_pos =kpath -1;
              if(turn_pos < 0 ) turn_pos = 0;

              wplist.list[turn_pos].robot_id = task.robot_id;
              wplist.list[turn_pos].robot_cmd = kind_turn;

              add_one_waypoint(wplist.list, task.time_end,
                      wplist.list[kpath-1].x,
					  wplist.list[kpath-1].y,
					  wplist.list[kpath-1].z,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      (robot_id)task.robot_id, turn_none);

              kpath++;

              _task_seq_saved = task.task_seq;
              updated = true;
          }
          else
          {
                LOG(ERROR) <<"error! interact robot ID error! id=" << task.robot_id;
          }
    }break;
    default:
          LOG(ERROR) << "unknow task type! can not be handled! task_type=" << task.task_type;
    }

    smooth_waypoints(wplist.list);

    if(_fp_path_env != NULL){
    	fprintf(_fp_path_env, "nPaths= %d\n", wplist.list.size());
    	for(int k = 0; k< wplist.list.size(); k++){
    		if( k == 0 || k == wplist.list.size()-1 || wplist.list[k].robot_cmd != turn_none){
    			fprintf(_fp_path_env, "k= %d seq= %d time= %d robotid= %d cmd= %s\n",k, wplist.list[k].seq, wplist.list[k].tms,
    					wplist.list[k].robot_id, ::str_kind_turn[wplist.list[k].robot_cmd]);
    		}
    	}
    }

    return 0;
}

int PlannerPath::plan_with_obs(iarc_arena_simulator::IARCWaypointsList &wplist)
{
    double orgx,orgy,orgt;//起始点
    double aimx,aimy,aimt;//目标点
    double obs[TOTAL_OBS][5];//障碍物的位置，速度，以及该障碍物是否可见。//这是障碍物的数量是有限的。
    double danger_radius;//危险距离
    double step_t,step_x,step_y;//空间的划分
    double range_xmin,range_ymin,range_xmax,range_ymax;//定义空间的限制大小
    int options;//选项
    int total_obs;

    wplist.list.clear();

    uint32_t time_now = arena_time_now();
    IARCTask task_now= get_task_now();
    if(task_now.time_end <=  time_now) return 0;

    orgx = _quad_status.x;
    orgy = _quad_status.y;
    orgt = ((double)time_now)/1000.0;
    aimx = task_now.final_pose.position.x;
    aimy = task_now.final_pose.position.y;
    aimt = ((double)task_now.time_end)/1000.0;
    danger_radius = 1.0;
    step_t = 1.0 / 1.0;
    step_x = 1.0 / 1.0;
    step_y = 1.0 / 1.0;
    /*
    range_xmin = min(orgx, aimx) - 2.0;
    range_ymin = min(orgy, aimy) -2.0;
    range_xmax = max(orgx, aimx) + 2.0;
    range_ymax = max(orgy, aimy) + 2.0;
    */
    range_xmin = -10.0;
    range_xmax = 10.0;
    range_ymin = -10.0;
    range_ymax = 10.0;
    options = 1.0;
    //--------------------------prhs[6]------------

    total_obs=TOTAL_OBS;//_obs_status.size();
    for(int i=0; i<_obs_status.size(); i++){
        obs[i][0] = _obs_status[i].x;
        obs[i][1] = _obs_status[i].y;
        obs[i][2] = _obs_status[i].velocity * cos(_obs_status[i].theta);
        obs[i][3] = _obs_status[i].velocity * sin(_obs_status[i].theta);
        obs[i][4] = 1.0;    //visiable
    }
    for(int i=_obs_status.size(); i<TOTAL_OBS; i++){
        for(int j = 0; j<5; j++){
                   obs[i][j] = 0.0;
               }
    }

    int nstep = 0;
    double **path = NULL;
    uint32_t t1 = arena_time_now();

    G_Shortest_Path theArena;

    theArena.Init(orgx,orgy,orgt,aimx,aimy,aimt,obs,danger_radius,step_t,step_x,step_y,range_xmin,range_ymin,range_xmax,range_ymax,options,total_obs);

    theArena.BFS(nstep);

//#error "function  BFS2() caused error!"
//    return 0 ;

    uint32_t t2 = arena_time_now();
    LOG(INFO) <<"time start="<<t1<<  " time end="<<t2<< " nstep =" <<nstep << "time cost=  "<<t2-t1;

    vector<double> x, y, z, timespace;
    x.clear();
    y.clear();
    z.clear();
    timespace.clear();
    LOG(INFO) << "nstep = " <<nstep;
    if(nstep>0)
    {
        path=new double*[nstep];
        for(int i=0; i<nstep; i++)
            path[i]=new double[3];

        theArena.GetPath_BFS(path);

        for (int k = 0; k<nstep; k++){
            timespace.push_back(path[k][2]);

            x.push_back(path[k][0]);
            y.push_back(path[k][1]);
            z.push_back(2.0) ;
        }
        for(int i=0; i<nstep; i++)
            delete []path[i];
        delete []path;
    }
    else
    {
        double time_now  = (double)(arena_time_now()/1000);
        x.push_back(_quad_status.x); y.push_back(_quad_status.y);
        z.push_back(2.0); timespace.push_back(time_now);
        x.push_back(_quad_status.x); y.push_back(_quad_status.y);
        z.push_back(2.0); timespace.push_back(time_now+0.5);
        x.push_back(_quad_status.x); y.push_back(_quad_status.y);
        z.push_back(2.0); timespace.push_back(time_now+1.0);
        nstep = 3;
    }

    wplist.list.clear();
    wplist.header.frame_id = PARAM::str_arena_frame;
    wplist.header.stamp = ros::Time::now();
    for (int k = 0; k<nstep; k++){
        iarc_arena_simulator::IARCWaypoint wp;
        wp.header = wplist.header;
        wp.seq = ++_wp_seq;
        wp.tms = (uint32_t)(timespace[k] *1000.0);
        wp.x = x[k];
        wp.y = y[k];
        wp.z = z[k];
        wplist.list.push_back(wp);
    }

    smooth_waypoints(wplist.list);

    LOG(INFO) << "plan ok";
    return 0;
}

int PlannerPath::graph_planing_path(std::vector<IARC_POSITION> &path,
			const std::vector<IARCRobot> &obs, double src_x, double src_y, double dst_x, double dst_y)
{
	int nNodes = _map_size_x * _map_size_y * PARAM::edge_num_direction + 2;
	struct G_Graph::Graph* graph = G_Graph::createGraph(nNodes);
	int src_xi, src_yi, dst_xi, dst_yi, src, dst;
	arena2map(src_x, src_y, src_xi, src_yi);
	arena2map(dst_x, dst_y, dst_xi, dst_yi);
	src = nNodes - 2;
	dst = nNodes - 1;

	path.clear();
	//build graph
	FILE * fp_edgemap = fopen(PARAM::file_name_edgemap.c_str(), "w");

	for(int i = 0; i< _map_size_x; i++){
		for(int j = 0; j< _map_size_y; j++){
			for(int k1 = 0; k1< PARAM::edge_num_direction; k1++){

				int ni = i + PARAM::edge_offx[k1];
				int nj = j + PARAM::edge_offy[k1];

				if( !in_grid_map(ni, nj) || !edge_in_safe_area(i, j, ni, nj) ) continue;

				for(int k2 = 0; k2< PARAM::edge_num_direction; k2++){
					int ni2 = ni + PARAM::edge_offx[k2];
					int nj2 = nj + PARAM::edge_offy[k2];
					if ( !in_grid_map(ni2, nj2) || !edge_in_safe_area(ni, nj, ni2, nj2)) continue;

					if(fabs(PARAM::edge_angel[k1][k2]) < PARAM::edge_angle_limited &&
							fabs(PARAM::edge_angel[k2][k1]) < PARAM::edge_angle_limited){
						G_Graph::weight_type w1, w2;
						w1 = PARAM::edge_weight[k2] + fabs(PARAM::edge_angel[k1][k2])/(PARAM::edge_angle_de) * PARAM::edge_coef_angle;
						w2 = PARAM::edge_weight[k1] + fabs(PARAM::edge_angel[k2][k1])/(PARAM::edge_angle_de) * PARAM::edge_coef_angle;

						G_Graph::addEdge(graph, get_nodes_edge_id(k1, i, j), get_nodes_edge_id(k2, ni, nj), w1);
						G_Graph::addEdge(graph, get_nodes_edge_id(k2, ni, nj), get_nodes_edge_id(k1, i, j), w2);

						//LOG(INFO) <<"conect: "<<i<<" "<<j<<" "<<ni<<" "<<nj<<" "<<ni2<<" "<<nj2;
						fprintf(fp_edgemap, "%d %d %d %d %d %d %d %d %.3lf %.3lf\n", i, j, ni, nj, ni2, nj2,
								get_nodes_edge_id(k1,i,j), get_nodes_edge_id(k2,ni,nj),  w1, w2);

					}
				}
				G_Graph::addEdge(graph, get_nodes_edge_id(k1,i, j), dst,
						PARAM::edge_nosolution_jump + (i - dst_xi) * (i - dst_xi) + (j - dst_yi) * (j - dst_yi));
			}
		}
	}

	if (sqrt(_quad_status.vx * _quad_status.vx + _quad_status.vy* _quad_status.vz) > PARAM::edge_velocity_static){
		double theta = atan2(_quad_status.vy, _quad_status.vx);
		if(theta < 0.0 ) theta  = theta + M_PI * 2.0;
		int ke = theta / (M_PI * 2.0 /(double)(PARAM::edge_num_direction));
		for(int k2 = 0; k2< PARAM::edge_num_direction; k2++){
			int ni = src_xi + PARAM::edge_offx[k2];
			int nj = src_yi + PARAM::edge_offy[k2];
			if( !in_grid_map(ni, nj) || !edge_in_safe_area(src_xi, src_yi, ni, nj) ) continue;
			if(fabs(PARAM::edge_angel[ke][k2]) < PARAM::edge_angle_limited){
						G_Graph::weight_type w1 = PARAM::edge_weight[k2] + fabs(PARAM::edge_angel[ke][k2])/PARAM::edge_angle_de * PARAM::edge_coef_angle;
						G_Graph::addEdge(graph, src, get_nodes_edge_id(k2, src_xi, src_yi), w1);
						LOG(INFO)<<"src con:"<<src<<" "<<src_xi<<" "<<src_yi<<" "<<k2;
					}
			}
	}
	else{
		for(int k = 0; k< PARAM::edge_num_direction; k++){
			int ni = src_xi + PARAM::edge_offx[k];
			int nj = src_yi + PARAM::edge_offy[k];
			if( !in_grid_map(ni, nj) || !edge_in_safe_area(src_xi, src_yi, ni, nj) ) continue;

			G_Graph::weight_type w1 = PARAM::edge_weight[k];
			G_Graph::addEdge(graph, src, get_nodes_edge_id(k, src_xi, src_yi), w1);
			LOG(INFO)<<"src con:"<<src<<" "<<src_xi<<" "<<src_yi<<" "<<k;
		}
	}

	for(int k = 0; k< PARAM::edge_num_direction; k++){
		int from_xi, from_yi;
		from_xi = dst_xi - PARAM::edge_offx[k];
		from_yi = dst_yi - PARAM::edge_offy[k];
		if(!in_grid_map(from_xi, from_yi) || !edge_in_safe_area(from_xi, from_yi, dst_xi, dst_yi)) continue;
		G_Graph::weight_type w1 = G_Graph::weight_zero + 0.001;
		G_Graph::addEdge(graph, get_nodes_edge_id(k, from_xi, from_yi), dst, w1);
		LOG(INFO)<<"dst con:"<<dst<<" "<<from_xi<<" "<<from_yi<<" "<<k;
	}

	fclose(fp_edgemap);
	//planning
	std::vector<int> nodes_path;
	bool found = false;
	found = G_Graph::dijkstra(graph, src, dst, PARAM::edge_nosolution_jump,nodes_path);

	//create path
	if(nodes_path.size() <= 0 ){
		LOG(ERROR) <<"no path found!";
	}
	else{
		IARC_POSITION pt;
		path.clear();

		pt.x = src_x;//lastx;
		pt.y = src_y;//lasty;

		path.push_back(pt);

		cout<<"path list"<<endl;
		int xi, yi;

		for(int k = nodes_path.size() -2; k >= 1; k--){
			xi = 0; yi = 0;
			pt.x = 0.0; pt.y = 0.0;
			get_edge_first_grid_by_id(nodes_path[k], xi, yi);
			map2arena(xi, yi, pt.x, pt.y);
			path.push_back(pt);
			cout<<nodes_path[k]<<" "<<pt.x<<" "<<pt.y<<" "<<get_k_of_edge_id(nodes_path[k])<<endl;
		}
		cout<<endl;

		get_edge_first_grid_by_id(nodes_path[0], xi, yi);

		if(found){
			pt.x = dst_x;
			pt.y = dst_y;
		}
		else{
			map2arena(xi, yi, pt.x, pt.y);
		}
		path.push_back(pt);
		path.push_back(pt);
	}

	freeGraph(graph);

	return 0;
}
int PlannerPath::planner_test_auto_task()
{

    return 0;
}

int PlannerPath::test_main()
{
	IARCRobot obstacle;
	obstacle.visible = true;
	obstacle.belief_rate = 1.0;
	obstacle.id = (robot_id)0;
	obstacle.time_ms = arena_time_now();
	obstacle.x = 0.0;
	obstacle.y = 0.0;
	_obs_status.clear();
	_obs_status.push_back(obstacle);
/*
	obstacle.x = +5.0;
	obstacle.y = 0.0;
	_obs_status.push_back(obstacle);

	obstacle.x = 0.0;
	obstacle.y = -2.0;
	_obs_status.push_back(obstacle);

	obstacle.x = 2.0;
	obstacle.y = 2.0;
	_obs_status.push_back(obstacle);
*/
	_quad_status.x = -6.0;
	_quad_status.y = -6.0;
	_quad_status.z = 2.0;
	_quad_status.vx = 0.0;
	_quad_status.vy = 0.0;
	_quad_status.vz = 0.0;

	std::vector<IARC_POSITION> path;
	path.clear();

	double src_x, src_y;
	double dst_x, dst_y;

	src_x = _quad_status.x;
	src_y = _quad_status.y;
	dst_x = 6.0;
	dst_y = 6.0;

	cout<<"start graph planning!"<<endl;

	uint32_t t1, t2;
	t1 = arena_time_now();
	cout<<"time t1= "<<t1<<endl;
	graph_planing_path(path, _obs_status, src_x, src_y, dst_x, dst_y);
	t2 = arena_time_now();
	cout<<"time t2= "<<t2<<" t2-t1= "<< t2 - t1<<endl;

	/*
	FILE *fp_path = fopen(PARAM::file_name_edgepath.c_str(),"w");
	for(int i = 0; i < path.size(); i++){
		cout<<path[i].x <<" "<< path[i].y<<endl;
		fprintf(fp_path, "%.2lf %.2lf\n", path[i].x, path[i].y);
	}
	fclose(fp_path);
*/
	std::vector<IARC_POSITION> curve;
	std::vector<iarc_arena_simulator::IARCWaypoint> list;
	list.clear();

	//smooth the path by bazier.
	G_Bazier::createCurve3ok(path, 0, path.size() - 1, curve, 0.02);

	FILE *fp_path = fopen(PARAM::file_name_edgepath.c_str(),"w");
	for(int i = 0; i < curve.size(); i++){
		cout<<curve[i].x <<" "<< curve[i].y<<endl;
		fprintf(fp_path, "%.2lf %.2lf\n", curve[i].x, curve[i].y);
	}
	fclose(fp_path);

	return 0;
}
