/*
 * common.h
 *
 *  Created on: 2017年4月20日
 *      Author: exbot
 */

#ifndef DJI_SDK_DEMO_SRC_COMMON_H_
#define DJI_SDK_DEMO_SRC_COMMON_H_

#include<stdint.h>
#include<stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include<stdlib.h>
#include "Eigen/Dense"
#include <iarc_arena_simulator/IARCTask.h>
#include <iarc_arena_simulator/IARCWaypoint.h>

namespace PARAM{
static const double ros_rate_cruise = 5;
static const double ros_rate_decision = 5;
static const double ros_rate_path = 15;
static const double ros_rate_tracking = 50;
static const double map_memory_step_x = 1.0;
static const double map_memory_step_y = 1.0;
static const double map_memory_org_x = -10.0;
static const double map_memory_org_y = -10.0;
static const int cruise_circular_sample_count = 10;
static const double cruise_velocity = 1.0;
static const int m = 3; //controller input dimension (vx, vy, vz)  dim = 3
static const int n = 9; // state dimension. (x,y,z,vx,vy,vz,ax,ay,az) dim = 9
static const double const_tf = 0.33;   //constant time in lqr controller.
static const double arena_size = 20.0; // edge length of the arena. is 20.0 meters
static const double angle_field_rad = 120.0/180.0*M_PI;// field of angle.
static const int list_capacity = 5000;  //the max possible waypoint list length
static const int taskslist_capacity = 50; // the max possible tasks list length
static const uint32_t waypoint_dT_ms = 20; // time interval of waypoint.
static const double velocity_obs = 0.33; //velocity of obstacles
static const double velocity_tgt = 0.33; // velocity of targets
static const double velocity_mav_max = 2.0;
static const uint32_t task_period_ms = 1000; // task planning period
static const int tree_search_max_depth = 0; //search max depth
static const double tree_search_gama = 0.8; //gama
static const double tree_search_gama_theta = 0.01;
static const double tree_search_gama_distance = 0.01;
static const double tree_search_gama_inner = 0.01;
static const int tree_search_max_target_counts = 4;
static const double ramda_t = 10.0;
static const double ramda_t_S = 1;
static const double ramda_t_E = -1;
static const double ramda_t_A = 1.0;
static const double ramda_t_SAE = 1.0;
static const double ramda_t_y = 1.0;
static const double ramda_t_theta = 0.5;
static const double ramda_m = 2.0;
static const double ramda_m_dis = 0.0;
static const double ramda_m_i_op = 1.0;
static const double ramda_o = 0.0;
static const double ramda_o_s = 0.0;
static const double ramda_o_a = 1.0;
static const double ramda_cruise = 0.2;
static const double radius_neary = 15.0;
static const double radius_collision = 1.0;
static const double radius_safe = 2.0;
static const double radius_lidar = 5.0;
static const int period_turn_ms = 20000;
static const int time_turn_180_ms = 2600;
static const int time_turn_45_ms = 650;
static const int time_cruise_det_ms = 8000; // cruise det time is 10000 ms
static const int time_turn_distinguish_ms = 1000;
static const int obstacle_time_forget_ms = 10000;
static const int target_time_forget_ms = 40000;
static const bool shouldExpand[20]={0,0,1,0,1,
                                                        0,1,0,1,0,
                                                        1,0,0,1,0,
                                                        0,0,0,0,0};  //supposed expand time in every period.
static const bool shouldCruise[20]={0,0,0,0,1,
                                                        0,0,0,0,0,
                                                        0,1,0,0,1,
                                                        0,0,1,0,0};  //supposed expand time in every period.
static const bool target_mask[10]={1,1,1,1,1,1,1,1,1,1};
static const bool obstacle_mask[4]={ 1,1,1,1};

static const int edge_num_direction = 8;
static const double edge_angle_limited = M_PI/2.0;
static const int edge_offx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int edge_offy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const double edge_weight[8] = {1.0, 1.4, 1.0, 1.4, 1.0, 1.4, 1.0, 1.4};//by 0.1 meters
static const double edge_velocity_static = 0.10;
static const double edge_time_one_edge = 1.0;
static const double edge_nosolution_jump = 1e5;
static const double edge_coef_angle = 0.001;
static const double edge_angle_de = M_PI/4.0;   //keep this param the same as 'de'
static const double edge_bazier_rate = 0.1;
#define de (M_PI/4.0)
static const double edge_angel[8][8]={
		{de * +0.0, de * +1.0, de * +2.0, de * +3.0, de * +4.0, de * +5.0, de * +6.0, de * +7.0},
	    {de * -1.0, de * +0.0, de * +1.0, de * +2.0, de * +3.0, de * +4.0, de * +5.0 , de * +6.0 },
		{de * -2.0, de * -1.0, de * +0.0, de * +1.0, de * +2.0, de * +3.0, de * +4.0, de * +5.0},
		{de * -3.0, de * -2.0, de * -1.0, de * +0.0, de * +1.0, de * +2.0, de * +3.0, de * +4.0},
		{de * -4.0, de * -3.0, de * -2.0, de * -1.0, de * +0.0, de * +1.0, de * +2.0, de * +3.0},
		{de * -5.0, de * -4.0, de * -3.0, de * -2.0, de * -1.0, de * +0.0, de * +1.0, de * +2.0},
		{de * -6.0, de * -5.0, de * -4.0, de * -3.0, de * -2.0, de * -1.0, de * +0.0, de * +1.0},
		{de * -7.0, de * -6.0, de * -5.0, de * -4.0, de * -3.0, de * -2.0, de * -1.0, de * +0.0}
};
#undef de

static const std::string str_arena_frame = "/arena_frame";
static const std::string file_name_lqr_plist = ros::package::getPath("dji_sdk_demo")+"/params/plist.txt";
static const std::string file_name_tracking = ros::package::getPath("dji_sdk_demo")+"/../../build/test_results" +"/rec_tracking.txt";
static const std::string file_name_controller = ros::package::getPath("dji_sdk_demo")+"/../../build/test_results" +"/rec_controller.txt";
static const std::string file_name_path = ros::package::getPath("dji_sdk_demo") + "/../../build/test_results"+"/rec_path.txt";
static const std::string file_name_path_env = ros::package::getPath("dji_sdk_demo") + "/../../build/test_results"+"/rec_path_env.txt";
static const std::string file_name_task = ros::package::getPath("dji_sdk_demo")+"/../../build/test_results"+"/rec_task.txt";
static const std::string file_name_task_env = ros::package::getPath("dji_sdk_demo")+"/../../build/test_results"+"/rec_task_env.txt";
static const std::string file_name_map = ros::package::getPath("dji_sdk_demo")+"/../../build/test_results"+"/rec_map.txt";
static const std::string file_name_bazier_in = ros::package::getPath("dji_sdk_demo")+"/../../test/"+"/bazier_in.txt";
static const std::string file_name_bazier_out = ros::package::getPath("dji_sdk_demo")+"/../../test/"+"/bazier_out.txt";
static const std::string file_name_edgemap = ros::package::getPath("dji_sdk_demo")+"/../../test/"+"/edgemap_out.txt";
static const std::string file_name_edgepath = ros::package::getPath("dji_sdk_demo")+"/../../test/"+"/edgepath_out.txt";
static const std::string file_name_timecost = ros::package::getPath("dji_sdk_demo")+"/../../test/"+"/timecost_out.txt";
}

#define N_OBS 4
#define N_TRG 3

#define KIND_TURN_NONE 0
#define KIND_TURN_180 1
#define KIND_TURN_45 2

#define DEFAULT_PARAM_AX 3.6
#define DEFAULT_PARAM_KX 3.0
#define DEFAULT_PARAM_VMX 2.0
#define DEFAULT_PARAM_AY 3.6
#define DEFAULT_PARAM_KY 3.0
#define DEFAULT_PARAM_VMY 2.0

#define SQR(a) ((a)*(a))
#define MAX(a,b) ((a)>(b)?(a):(b))

typedef struct IARC_POSITION{
      double x;
      double y;
      double z;
}IARC_POSITION;

typedef struct IARC_Quaternion{
      double x,y,z,w;
} IARC_Quaternion;

static const char* str_task_type[5]={"hover ","reach ","follow","in.act", "cruise"};
static const char* str_kind_turn[3]={"turn_none","turn_180","turn_45"};

enum task_type{
      type_hover,
      type_reach,
      type_follow,
      type_interact,
	  type_cruise
};
enum robotcmd_kind{
      turn_none=KIND_TURN_NONE,
      turn_180=KIND_TURN_180,
      turn_45=KIND_TURN_45
};
enum robot_id{
    robot_0 = 0,
    robot_1 = 1,
    robot_2 = 2,
    robot_3 = 3,
    robot_4 = 4,
    robot_5 = 5,
    robot_6 = 6,
    robot_7 = 7,
    robot_8 = 8,
    robot_9 = 9,
    obs_10 = 10,
    obs_11 = 11,
    obs_12 = 12,
    obs_13 = 13,
    robot_arena = 255,
	robot_none = 127,
};
static const char* str_task_result[4]={"error ","going..","reached","failed!"};

enum task_result{
      task_error=0,
      task_going=1,
      task_reached=2,
      task_failed=3
};


typedef struct IARC_MAV{
      uint32_t time_ms;
      double x,y,z;
      double vx,vy,vz;
      double ax,ay,az;
      IARC_Quaternion orientation;
      double param_ax,param_ay;
      double param_kx,param_ky;
      double param_vmx,param_vmy,param_vm;
}IARC_MAV;

typedef struct IARCQuad{
      uint32_t time_ms;
      double x,y,z;
      double vx,vy,vz;
      double ax,ay,az;
      double qw, qx, qy, qz;
}IARCQuad;

typedef struct IARCRobot{
    uint32_t time_ms;
    double x;
    double y;
    double velocity;
    double theta;
    enum robot_id id;
    bool visible;
    uint32_t lost_time_ms;
    double belief_rate;
    double lost_x;
    double lost_y;
    double lost_velocity;
    double lost_theta;
} IARCRobot;

typedef struct IARC_ROBOT{
      uint32_t time_ms;
      double x;
    double y;
    double velocity;
    IARC_Quaternion orientation;
}IARC_ROBOT;

//与某个目标交互，交互点应该是实时变化的，所以目标点实时随着目标的运动反馈变化．
//所以交互点不应该是一个固定点，而是i,op,t,(p只作为噪声容限进行考虑).
//跟随某一个目标运动，也是一种任务，其收益是怎么衡量呢？什么情况下跟随而不作交互？
//飞抵某一个点，这是一个静态的任务．．．
//路径记忆机器人，预测的交互点，就可以用于巡航，执行的是空操作．
//跟随等操作，也都用交互任务进行表示．．．
//task 中的　具体的坐标值，是可以实时变化的．．．
//task=(p, t, i, op, deps)
typedef struct IARC_TASK{
      uint32_t seq;//task_unique ID sequence
      enum task_type type;//task_type
      uint32_t start_time;
      uint32_t  cost_time;
      double value;
      union{
            struct{
                  double reach_x;
                  double reach_y;
                  double reach_tor_x;
                  double reach_tor_y;
                  uint32_t reach_tor_time;
            };
            struct{
                  uint8_t follow_id;
                  double follow_tor_x;
                  double follow_tor_y;
            };
            struct{
                  uint8_t interact_id;
                  enum robotcmd_kind interact_cmdkind;
                  bool interact_command_sent;
                  double interact_exp_x;
                  double interact_exp_y;
                  double interact_exp_azimuth;
                  double interact_tor_x;
                  double interact_tor_y;
                  double interact_tor_azimuth;
            };
      } content;
}IARC_TASK;

typedef struct IARC_WAYPOINT{
      uint32_t    t;
      double x;
      double y;
}IARC_WAYPOINT;

typedef struct IARC_COMMAND{
      double mav_vx;
      double mav_vy;
      double mav_vz;
      double mav_vyawdegree;
      uint8_t  robot_turn_kind;
      uint8_t robot_turn_id;
}IARC_COMMAND;

uint32_t arena_time_now();
void arena_set_startnow();
void arena_set_startnow2(std::string filename);
double quaternion_to_theta_z(double q_w, double q_z);
int theta2quaternion(double theta, double &w, double &x, double &y, double &z);
iarc_arena_simulator::IARCTask make_task_type_reach(std::string frame_id,
        ros::Time stamp, double aimx,
        double aimy, double aimz, uint32_t start_time_ms, uint32_t aim_time_ms);
iarc_arena_simulator::IARCTask make_task_type_hover(std::string frame_id,
        ros::Time stamp, uint32_t start_time_ms, uint32_t aim_time_ms);
iarc_arena_simulator::IARCTask make_task_type_follow(std::string frame_id,
        ros::Time stamp, uint32_t robot_id, uint32_t start_time_ms, uint32_t end_time_ms);
IARC_COMMAND make_zero_cmd();
iarc_arena_simulator::IARCWaypoint make_waypoint(const std::string &frame_id, const ros::Time &stamp, uint32_t seq, uint32_t time_now, const Eigen::VectorXd & _state_x,
                robotcmd_kind  turn_kind, robot_id id);
IARCRobot make_prediction_robot(uint32_t rtime, IARCRobot robot, double final_turn);
IARCRobot get_robot_from_id(const std::vector<IARCRobot> &tgt, uint32_t id);
double forget_function(uint32_t det_time_ms);

#endif /* DJI_SDK_DEMO_SRC_COMMON_H_ */
