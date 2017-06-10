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
#include "PlannerDecision.h"
#include "common.h"

static PlannerDecision theDecision;
static ros::Publisher tasks_list_pub;
static ros::Subscriber quad_sub;
static ros::Subscriber obstacles_sub;
static ros::Subscriber targets_sub;
static ros::Subscriber cmd_sub;
static ros::Publisher tasks_pub;
void IARCCommand_callback(const iarc_arena_simulator::IARCCommand::ConstPtr & cmd);
void QuadStatus_callback(const iarc_arena_simulator::IARCQuadStatus::ConstPtr & status);
void IARCObstacles_callback(const geometry_msgs::PoseArray::ConstPtr& obstacles);
void IARCTargets_callback(const geometry_msgs::PoseArray::ConstPtr& targets);
void GenerateTaskViewer(const iarc_arena_simulator::IARCTasksList &tasks_list, geometry_msgs::PoseArray &tasks);

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "decision_plan");
    ros::NodeHandle nh;

    arena_set_startnow();

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir=ros::package::getPath("dji_sdk_demo")+"/../../build/test_results";
    std::cout<<FLAGS_log_dir<<std::endl;
    LOG(INFO) << "record info to this file";

    cmd_sub = nh.subscribe<iarc_arena_simulator::IARCCommand>("/iarc_arena/IARCCommand", 10, IARCCommand_callback);
    quad_sub = nh.subscribe<iarc_arena_simulator::IARCQuadStatus>("/iarc_arena/IARCQuadStatus",10, QuadStatus_callback);
    obstacles_sub = nh.subscribe<geometry_msgs::PoseArray>("/iarc_arena/IARCObstacles",10,IARCObstacles_callback);
    targets_sub = nh.subscribe<geometry_msgs::PoseArray>("/iarc_arena/IARCTargets",10,IARCTargets_callback);
    tasks_list_pub = nh.advertise<iarc_arena_simulator::IARCTasksList>("/iarc_arena/IARCTasksList",10);
    tasks_pub = nh.advertise<geometry_msgs::PoseArray>("/iarc_arena/IARCTask",10);
    theDecision.init();

    ros::Rate r0(PARAM::ros_rate_decision);
    int cnt=0;

    while(ros::ok())
    {
            cnt++;
            LOG(INFO) <<"loop cnt=" <<cnt;

            iarc_arena_simulator::IARCTasksList tasks_list;
            theDecision.plan(tasks_list);
            if(tasks_list.list.size()!=0){
                tasks_list_pub.publish(tasks_list);
            }

            geometry_msgs::PoseArray tasks;
            GenerateTaskViewer(tasks_list,tasks);
            if(tasks.poses.size()!=0){
            	tasks_pub.publish(tasks);
            }


            ros::spinOnce();
            r0.sleep();
    }

    LOG(INFO) << "quit!";
   	printf("Quit Decision!\n");


    return 0;
}

void IARCCommand_callback(const iarc_arena_simulator::IARCCommand::ConstPtr & cmd)
{
    theDecision.update_latest_cmdtime(cmd);
}
void QuadStatus_callback(const iarc_arena_simulator::IARCQuadStatus::ConstPtr & status)
{
    theDecision.update_quad(status);
}
void IARCObstacles_callback(const geometry_msgs::PoseArray::ConstPtr& obstacles)
{
      theDecision.update_obs(obstacles);
}
void IARCTargets_callback(const geometry_msgs::PoseArray::ConstPtr& targets)
{
      theDecision.update_tgt(targets);
}

void GenerateTaskViewer(const iarc_arena_simulator::IARCTasksList &tasks_list, geometry_msgs::PoseArray &tasks)
{
	tasks.header.frame_id = PARAM::str_arena_frame;
	tasks.header.stamp = ros::Time::now();
	tasks.poses.clear();
	for(int k = 0; k< tasks_list.list.size(); k++){
		geometry_msgs::Pose p;
		if (tasks_list.list[k].robot_cmd == turn_180){
			p.position.x = tasks_list.list[k].final_pose.position.x;
			p.position.y = tasks_list.list[k].final_pose.position.y;
			p.position.z = 0.0;
	    	p.orientation.x = 0.0;
	    	p.orientation.y = sin( -M_PI / 4);
	    	p.orientation.z = 0.0;
	    	p.orientation.w = cos(-M_PI/4);
			tasks.poses.push_back(p);
		}
		if(tasks_list.list[k].robot_cmd == turn_45){
					p.position.x = tasks_list.list[k].final_pose.position.x;
					p.position.y = tasks_list.list[k].final_pose.position.y;
					p.position.z = 1.0;
				    p.orientation.x = 0.0;
				    p.orientation.y = sin( M_PI / 4);
				    p.orientation.z = 0.0;
				    p.orientation.w = cos(M_PI/4);
					tasks.poses.push_back(p);
		}
	}
}
