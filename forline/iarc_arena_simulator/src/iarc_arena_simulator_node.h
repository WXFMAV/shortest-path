#ifndef __IARC_ARENA_SIMULATOR_NODE_H__
#define __IARC_ARENA_SIMULATOR_NODE_H__

#include<ros/ros.h>
#include<cstdlib>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<iarc_arena_simulator/IARCCommand.h>
#include<iarc_arena_simulator/IARCQuadStatus.h>
#include <ros/package.h>
#include<time.h>
#include<math.h>

#define N_OBS 4
#define N_TRG 10
#define TIME_TURN_180 2600
#define TIME_TURN_45 650
#define PERIOD_TURN 20000

#define KIND_TURN_NONE 0
#define KIND_TURN_180 1
#define KIND_TURN_45 2

#define THETA_TURN_180 M_PI
#define THETA_TURN_45 (M_PI/4)

//#define quit_when_collision

static const char* str_kind_turn[3]={"turn_none","turn_180","turn_45"};
static std::string file_name_arena_info =  ros::package::getPath("dji_sdk_demo") + "/../../build/test_results"+"/rec_arena_info.txt";
static std::string file_name_arena_cmd_received =  ros::package::getPath("dji_sdk_demo") + "/../../build/test_results"+"/rec_arena_cmd_received.txt";

#define PI 3.141592653897932384626

#endif
