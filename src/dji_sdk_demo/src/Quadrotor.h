/*
 * Quadrotor.h
 *
 *  Created on: 2017年4月19日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_QUADROTOR_H_
#define DJI_SDK_DEMO_SRC_QUADROTOR_H_

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <ros/ros.h>
#include<string>
#include <dji_sdk/dji_drone.h>
#include "common.h"

enum enum_model{model_none,model_m100,model_simulator};

class Quadrotor {
public:
    Quadrotor();
    virtual ~Quadrotor();
    int init(enum_model model, ros::NodeHandle nh);
    int get_feedback();
    int get_pos(double &x,double &y,double &z);
    int get_vel(double &vx,double &vy,double &vz);
    int get_acc(double &ax,double &ay,double &az);
    int get_quat(double &q0,double &q1,double &q2,double &q3);
    int get_state_all(double state_x[13]);
    int exe_control();
    int exe_control_u(double control_u[4]);
    int exe_control_uxyz(double ux,double uy,double uz);
    int exe_control_cmd(IARC_COMMAND cmd);
private:
    int DroneActivate(DJIDrone* drone);
    int DroneFeedback(DJIDrone* drone);
    int DroneSendCmd(DJIDrone* drone);
    int SimulatorInit();
    int SimulatorFeedback();
    int SimulatorSendCmd();
    int SimulatorLimitStateX();
    bool Valid_Control_u();
public:
    enum {ERROR, OK} _status;
    union{
        struct{
            double _x, _y, _z;
            double _vx, _vy, _vz;
            double _ax, _ay, _az;
            double _q0, _q1, _q2, _q3;
        };
        double _state_x[13];
    };
    union{
        struct{
            double _ux, _uy, _uz, _uyawdegree;
        };
        double _control_u[4];
    };
    enum_model _model;
    DJIDrone* _drone_m100;
    double _state_x_sim[13];
    double _state_x_limit[13];//状态限幅
};
#endif /* DJI_SDK_DEMO_SRC_QUADROTOR_H_ */
