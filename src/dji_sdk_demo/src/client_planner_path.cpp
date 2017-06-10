#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <iarc_arena_simulator/IARCWaypoint.h>
#include <iarc_arena_simulator/IARCWaypointsList.h>
#include <iarc_arena_simulator/IARCQuadStatus.h>
#include <iarc_arena_simulator/IARCTask.h>
#include<iarc_arena_simulator/IARCTasksList.h>
#include <ros/package.h>
#include <glog/logging.h>
#include "PlannerPath.h"
#include "common.h"

static PlannerPath thePath;
static ros::Publisher waypoints_list_pub;
static ros::Publisher virtual_obstacles_pub;
static ros::Publisher virtual_targets_pub;

static ros::Subscriber quad_sub;
static ros::Subscriber obstacles_sub;
static ros::Subscriber targets_sub;
static ros::Subscriber tasks_list_sub;
static ros::Subscriber cmd_sub;

void IARCCommand_callback(const iarc_arena_simulator::IARCCommand::ConstPtr & cmd);
void QuadStatus_callback(const iarc_arena_simulator::IARCQuadStatus::ConstPtr & status);
void IARCObstacles_callback(const geometry_msgs::PoseArray::ConstPtr& obstacles);
void IARCTargets_callback(const geometry_msgs::PoseArray::ConstPtr& targets);
void IARCTasksList_callback(const iarc_arena_simulator::IARCTasksList::ConstPtr & tasklist);

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir=ros::package::getPath("dji_sdk_demo")+"/../../build/test_results";
    std::cout<<FLAGS_log_dir<<std::endl;
    LOG(INFO) << "record info to this file";

    ros::init(argc, argv, "path_plan");
    ros::NodeHandle nh;

    std::string filename =  ros::package::getPath("dji_sdk_demo")+"/../../build/test_results/rec_ast_path.txt";
    arena_set_startnow2(filename);
    LOG(ERROR) <<" arena_time_now "<< arena_time_now();

    cmd_sub = nh.subscribe<iarc_arena_simulator::IARCCommand>("/iarc_arena/IARCCommand", 10, IARCCommand_callback);
    tasks_list_sub = nh.subscribe<iarc_arena_simulator::IARCTasksList>("/iarc_arena/IARCTasksList",10,IARCTasksList_callback);
    quad_sub = nh.subscribe<iarc_arena_simulator::IARCQuadStatus>("/iarc_arena/IARCQuadStatus",10, QuadStatus_callback);
    obstacles_sub = nh.subscribe<geometry_msgs::PoseArray>("/iarc_arena/IARCObstacles",10,IARCObstacles_callback);
    targets_sub = nh.subscribe<geometry_msgs::PoseArray>("/iarc_arena/IARCTargets",10,IARCTargets_callback);
    waypoints_list_pub = nh.advertise<iarc_arena_simulator::IARCWaypointsList>("/iarc_arena/IARCWaypointsList",10);
    virtual_targets_pub = nh.advertise<geometry_msgs::PoseArray>("/iarc_arena/IARCTargets_virtual",10);
    virtual_obstacles_pub = nh.advertise<geometry_msgs::PoseArray>("/iarc_arena/IARCObstacles_virtual",10);

    thePath.init();

    ros::Rate r0(PARAM::ros_rate_path);
    int cnt=0;

    while(ros::ok())
    {
            cnt++;
            LOG(INFO) <<"loop cnt=" <<cnt;

            iarc_arena_simulator::IARCWaypointsList wp_list;
            thePath.plan(wp_list);
            if(wp_list.list.size()>0){
                waypoints_list_pub.publish(wp_list);
            }
            geometry_msgs::PoseArray obs;
            thePath.get_array_obstacles(obs);
            virtual_obstacles_pub.publish(obs);

            geometry_msgs::PoseArray tgt;
            thePath.get_array_targets(tgt);
            virtual_targets_pub.publish(tgt);

            ros::spinOnce();
            r0.sleep();
    }

	LOG(INFO) << "quit!";
	printf("Quit Path!\n");


    return 0;
}

void IARCCommand_callback(const iarc_arena_simulator::IARCCommand::ConstPtr & cmd)
{
	thePath.update_latest_cmdtime(cmd);
}
void IARCTasksList_callback(const iarc_arena_simulator::IARCTasksList::ConstPtr & tasklist)
{
    if(tasklist->list.size()!=0){
        thePath.update_taskslist(tasklist);
    }
}
void QuadStatus_callback(const iarc_arena_simulator::IARCQuadStatus::ConstPtr & status)
{
    thePath.update_quad(status);
}
void IARCObstacles_callback(const geometry_msgs::PoseArray::ConstPtr& obstacles)
{
      if(obstacles->poses.size()>0){
          thePath.update_obs(obstacles);
      }
}
void IARCTargets_callback(const geometry_msgs::PoseArray::ConstPtr& targets)
{
      if(targets->poses.size()>0){
          thePath.update_tgt(targets);
      }
}
