/*
 * Quadrotor.cpp
 *
 *  Created on: 2017年4月19日
 *      Author: Hualin He
 */

#include "Quadrotor.h"
#include <glog/logging.h>
#include "Eigen/Dense"
#include<string.h>
#include<iostream>
#include<iomanip>
using namespace DJI::onboardSDK;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

Quadrotor::Quadrotor() {
    // TODO Auto-generated constructor stub
    _status = ERROR;
    _model = model_none;
    _x = 0.0; _y = 0.0; _z = 0.0;
    _vx = 0.0; _vy = 0.0; _vz = 0.0;
    _ax = 0.0; _ay = 0.0; _az = 0.0;
    _q0 = 1.0; _q1 = 0.0; _q2 = 0.0; _q3 = 0.0;
    _ux = 0.0; _uy = 0.0; _uz = 0.0; _uyawdegree = 0.0;
    _drone_m100 = nullptr;
    memset(_state_x_limit, 0, sizeof(_state_x_limit));
}

Quadrotor::~Quadrotor() {
    // TODO Auto-generated destructor stub
    _status = ERROR;
    if( _drone_m100 != nullptr ){
        delete _drone_m100;
        _drone_m100 = nullptr;
    }
}

int Quadrotor::init(enum_model model, ros::NodeHandle nh){
    switch(model)
    {
        case model_none:{
            _model = model;
            LOG(ERROR)<<"no quadrotor model has been set!";
            return -1;
        }break;
        case model_m100:{
            _model = model;
            _drone_m100 = new DJIDrone(nh);
            return DroneActivate(_drone_m100);
        }break;
        case model_simulator:{
            _model = model;
            SimulatorInit();
        }break;
    }
    return 0;
}
int Quadrotor::get_pos(double &x, double &y, double &z)
{
    get_feedback();
    x = _x;
    y = _y;
    z = _z;
    return 0;
}
int Quadrotor::get_vel(double &vx, double &vy, double &vz)
{
    get_feedback();
    vx = _vx;
    vy = _vy;
    vz = _vz;
    return 0;
}
int Quadrotor::get_acc(double &ax,double &ay,double &az)
{
    get_feedback();
    ax = _ax;
    ay = _ay;
    az = _az;
    return 0;
}
int Quadrotor::get_quat(double &q0, double &q1, double &q2, double &q3)
{
    get_feedback();
    q0 = _q0;
    q1 = _q1;
    q2 = _q2;
    q3 = _q3;
    return 0;
}
int Quadrotor::get_state_all(double state_x[13])
{
    get_feedback();
    memcpy(_state_x, state_x, 13 * sizeof(double));
    return 0;
}

int Quadrotor::get_feedback(){
    switch(_model)
    {
        case model_none:{

        }break;
        case model_m100:{
            DroneFeedback(_drone_m100);
        }break;
        case model_simulator:{
            SimulatorFeedback();
        }break;
    }

    return 0;
}
int Quadrotor::exe_control_cmd(IARC_COMMAND cmd)
{
    exe_control_uxyz(cmd.mav_vx,cmd.mav_vy,cmd.mav_vz);
    return 0;
}
int Quadrotor::exe_control_uxyz(double ux, double uy, double uz)
{
    _ux = ux;
    _uy = uy;
    _uz = uz;
    _uyawdegree = 0.0;
    return exe_control();
}
int Quadrotor::exe_control_u(double control_u[4]){
    _ux = control_u[0];
    _uy = control_u[1];
    _uz = control_u[2];
    _uyawdegree = control_u[3];
    return exe_control();
}

bool Quadrotor::Valid_Control_u()
{
    for (int k = 0 ; k< 4; k++){
        if (isnan( _control_u[k])){
            return false;
        }
    }
    return true;
}
int Quadrotor::exe_control()
{
    if( ! Valid_Control_u() ) return -1;

    switch(_model)
    {
        case model_none:{

        }break;
        case model_m100:{
            DroneSendCmd(_drone_m100);
        }break;
        case model_simulator:{
            SimulatorSendCmd();
        }break;
    }
    return 0;
}
int Quadrotor::DroneActivate(DJIDrone* drone)
{
    if (drone != NULL) {
        for (int i = 0; i < 10; i++) {
            drone->activate();
            usleep(10000);
            LOG(INFO) << "drone activation times=" << drone->activation;
            if (drone->activation) {
                LOG(INFO) << "drone activation success ok !!";
                return 1;
            }
        }
        LOG(ERROR) << "drone activation failed!";
        return -1;
    }
    else{
        return -1;
    }

    return 0;
}
int Quadrotor::DroneFeedback(DJIDrone* drone)
{
    _x=drone->local_position.x;
    _y=drone->local_position.y;
    _z=drone->local_position.z;
    _vx=drone->velocity.vx;
    _vy=drone->velocity.vy;
    _vz=drone->velocity.vz;
    _ax=drone->acceleration.ax;
    _ay=drone->acceleration.ay;
    _az=drone->acceleration.az;
    _q0=drone->attitude_quaternion.q0;
    _q1=drone->attitude_quaternion.q1;
    _q2=drone->attitude_quaternion.q2;
    _q3=drone->attitude_quaternion.q3;
/*
    LOG(INFO) << std::setiosflags(std::ios::fixed) << std::setprecision(2) << "state: "
            <<_x << _y << _z << _vx << _vy << _vz << _ax << _ay << _az << _q0 << _q1
            << _q2 << _q3;
            */
    return 0;
}
int Quadrotor::DroneSendCmd(DJIDrone* drone)
{
    double cmd_vx, cmd_vy, cmd_vz, cmd_vyawdegree;
    cmd_vx = _ux;
    cmd_vy = _uy;
    cmd_vz = _uz;
    cmd_vyawdegree = _uyawdegree;

    static int drone_permission = 0;
    if (drone_permission == 0 && drone->rc_channels.mode > 4000.0) {
        drone_permission = drone->request_sdk_permission_control();
        usleep(1000);
        if (drone_permission == 1) {
            LOG(INFO)<< "permission obtained!";
        }
        else
        {
            LOG(INFO) << "permission denied!";
        }
    }
    else
    {
        if(drone->rc_channels.mode<=4000.0)
        {
            drone_permission=0;
        }
    }

    if (drone_permission == 1) {
        LOG(INFO)<< std::setiosflags(std::ios::fixed) << std::setprecision(2) << "control: "<< cmd_vx << cmd_vy << cmd_vz << cmd_vyawdegree;

        return drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_PALSTANCE |
                Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                Flight::SmoothMode::SMOOTH_ENABLE,
                cmd_vx, cmd_vy, cmd_vz, cmd_vyawdegree );
    }
    else
    {
        return 0;
    }

}
int Quadrotor::SimulatorInit()
{
    _state_x_sim[0] = 10.0; _state_x_sim[1] = 0.0; _state_x_sim[2] = 2.0;
    _state_x_sim[3] = 0.0; _state_x_sim[4] = 0.0; _state_x_sim[5] = 0.0;
    _state_x_sim[6] = 0.0; _state_x_sim[7] = 0.0; _state_x_sim[8] = 0.0;
    _state_x_sim[9] = 1.0; _state_x_sim[10] = 0.0; _state_x_sim[11] = 0.0;
    _state_x_sim[12] = 0.0;

    _state_x_limit[0] = 11.0; _state_x_limit[1] = 11.0; _state_x_limit[2] = 3.0;
    _state_x_limit[3] = 4.0; _state_x_limit[4] = 4.0; _state_x_limit[5] = 2.0;
    _state_x_limit[6] = 5*9.8; _state_x_limit[7] = 5*9.8; _state_x_limit[8] = 5*9.8;
    return 0;
}
int Quadrotor::SimulatorFeedback()
{
    for(int k=0; k<13; k++){
        _state_x[k]=_state_x_sim[k];
    }

    return 0;
}
int Quadrotor::SimulatorSendCmd()
{
    //loop once for a discrete simulator model.
    static bool simulate_model_built = false;
    static MatrixXd A(9,9), B(9,3), G, H;
    static double dT = 0.02;
    static double ax1 = 22.93;
    static double ax2 = 53.24;
    static double Kx = 60.0;
    static double ay1 = 22.93;
    static double ay2 = 53.24;
    static double Ky = 60.0;
    static double az1 = 17.38;
    static double az2 = 54.72;
    static double Kz = 54.72;

    A <<0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 0, 0, -ax2, 0, 0, -ax1, 0, 0,
            0, 0, 0, 0, -ay2, 0, 0, -ay1, 0,
            0, 0, 0, 0, 0, -az2, 0, 0, -az1;
    B <<0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            Kx, 0, 0,
            0, Ky, 0,
            0, 0, Kz;
    if( !simulate_model_built){
        EigenSolver<Eigen::MatrixXd> evd(A);
        MatrixXcd U = evd.eigenvectors();
        MatrixXcd V = U.inverse();
        VectorXcd S = evd.eigenvalues();
        VectorXcd St = ((VectorXcd)(S * dT)).array().exp();
        VectorXcd iSt(S.size());
       // LOG(INFO) << "U*V'.inv=" << V.transpose() * U;
        for( int k = 0; k<St.size(); k++ ){
            if( S(k) == 0.0 ){
                iSt(k) = dT;
            }
            else{
                iSt(k) = 1.0 / S(k) * (St(k) - 1.0);
            }
        }
        G = (U * (St.asDiagonal()) * V).real();
        H = (U * (iSt.asDiagonal()) * V).real() * B;
        simulate_model_built = true;
    }

    VectorXd x1 = VectorXd::Zero(9);
    x1<<_state_x_sim[0], _state_x_sim[1], _state_x_sim[2],
            _state_x_sim[3], _state_x_sim[4], _state_x_sim[5],
            _state_x_sim[6], _state_x_sim[7], _state_x_sim[8];

    Vector3d u(_ux, _uy, _uz);

    LOG(INFO) << "input u=" <<u.transpose();
    LOG(INFO) <<" state x=" <<x1.transpose();
    VectorXd x2 = G * x1 + H * u;
    LOG(INFO) <<" Gx+Hu=" <<x2.transpose();
    for(int k = 0; k < 9; k++){
        _state_x_sim[k] = x2[k];
    }
    _state_x_sim[9] = 1.0;  _state_x_sim[10] = 0.0;
    _state_x_sim[11] = 0.0; _state_x_sim[12] = 0.0;

    SimulatorLimitStateX();

    LOG(INFO) << "sim state=" <<x2.transpose();
    return 0;
}

int Quadrotor::SimulatorLimitStateX()
{
    for(int k = 0; k< 9; k++){
        if(_state_x_sim[k] > _state_x_limit[k]){
            _state_x_sim[k] = _state_x_limit[k];
            LOG(WARNING) << "state limit reached!";
        }
        else if( _state_x_sim[k] < - _state_x_limit[k]){
            _state_x_sim[k] = -_state_x_limit[k];
            LOG(WARNING) << "state limit reached!";
        }
    }
    return 0;
}
