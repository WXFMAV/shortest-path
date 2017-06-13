#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Range.h>
#include <iarc_arena_simulator/IARCWaypoint.h>
#include <iarc_arena_simulator/IARCWaypointsList.h>
#include <iarc_arena_simulator/IARCQuadStatus.h>
#include <iarc_arena_simulator/IARCTask.h>
#include<iarc_arena_simulator/IARCTasksList.h>
#include <ros/package.h>
#include <glog/logging.h>
#include "PlannerCruise.h"
#include "common.h"

static PlannerCruise theCruise;
static ros::Publisher tasks_list_pub;
static ros::Subscriber quad_sub;
static ros::Subscriber obstacles_sub;
static ros::Subscriber targets_sub;
static ros::Subscriber cmd_sub;
static ros::Publisher tasks_pub;
static ros::Publisher view_pub;
static ros::Publisher map_pub;
static ros::Publisher viewdanger_pub[4];

void IARCCommand_callback(const iarc_arena_simulator::IARCCommand::ConstPtr & cmd);
void QuadStatus_callback(const iarc_arena_simulator::IARCQuadStatus::ConstPtr & status);
void IARCObstacles_callback(const geometry_msgs::PoseArray::ConstPtr& obstacles);
void IARCTargets_callback(const geometry_msgs::PoseArray::ConstPtr& targets);
void GenerateTaskViewer(const iarc_arena_simulator::IARCTasksList &tasks_list, geometry_msgs::PoseArray &tasks);
void GenerateView(const  IARCQuad &quad,  geometry_msgs::PolygonStamped &view);
void GenerateDangerView(const IARCRobot &obs,  geometry_msgs::PolygonStamped &view);

int main(int argc, char **argv)
{  
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir=ros::package::getPath("dji_sdk_demo")+"/../../build/test_results";
    std::cout<<FLAGS_log_dir<<std::endl;
    LOG(INFO) << "record info to this file";

    ros::init(argc, argv, "cruise_plan");
    ros::NodeHandle nh;

    std::string filename =  ros::package::getPath("dji_sdk_demo")+"/../../build/test_results/rec_ast_cruise.txt";
    arena_set_startnow2(filename);
    LOG(ERROR) <<" arena_time_now "<< arena_time_now();

    cmd_sub = nh.subscribe<iarc_arena_simulator::IARCCommand>("iarc_arena/IARCCommand", 10, IARCCommand_callback);
    quad_sub = nh.subscribe<iarc_arena_simulator::IARCQuadStatus>("iarc_arena/IARCQuadStatus",10, QuadStatus_callback);
    obstacles_sub = nh.subscribe<geometry_msgs::PoseArray>("iarc_arena/IARCObstacles",10,IARCObstacles_callback);
    targets_sub = nh.subscribe<geometry_msgs::PoseArray>("iarc_arena/IARCTargets",10,IARCTargets_callback);
    tasks_list_pub = nh.advertise<iarc_arena_simulator::IARCTasksList>("iarc_arena/IARCTasksList",10);
    tasks_pub = nh.advertise<geometry_msgs::PoseArray>("iarc_arena/IARCTask",10);
    view_pub = nh.advertise<geometry_msgs::PolygonStamped>("iarc_arena/IARCView",10);
    viewdanger_pub[0] = nh.advertise<geometry_msgs::PolygonStamped>("iarc_arena/IARCView_obs1",10);
    viewdanger_pub[1] = nh.advertise<geometry_msgs::PolygonStamped>("iarc_arena/IARCView_obs2",10);
    viewdanger_pub[2] = nh.advertise<geometry_msgs::PolygonStamped>("iarc_arena/IARCView_obs3",10);
    viewdanger_pub[3] = nh.advertise<geometry_msgs::PolygonStamped>("iarc_arena/IARCView_obs4",10);
    theCruise.init();

    sensor_msgs::Range rg;

    ros::Rate r0(PARAM::ros_rate_cruise);
    int cnt=0;

    while(ros::ok())
    {
            cnt++;
            LOG(INFO) <<"loop cnt=" <<cnt;

            iarc_arena_simulator::IARCTasksList tasks_list;
            theCruise.plan(tasks_list);
            if(tasks_list.list.size()!=0){
                tasks_list_pub.publish(tasks_list);
            }

            geometry_msgs::PoseArray tasks;
            GenerateTaskViewer(tasks_list,tasks);
            if(tasks.poses.size()!=0){
            	tasks_pub.publish(tasks);
            }

            if(1){
            	geometry_msgs::PolygonStamped quadview;
                GenerateView(theCruise._quad_status, quadview);
                view_pub.publish(quadview);
            }

            if(1){
            	geometry_msgs::PolygonStamped obsview;
            	for(int k = 0; k<4; k++){
            		if(k < theCruise._obs_status.size()){
            			geometry_msgs::PolygonStamped view;
            			GenerateDangerView(theCruise._obs_status[k], view);
            			viewdanger_pub[k].publish(view);
            		}
            		else{
            			geometry_msgs::PolygonStamped view;
            			view.header.frame_id = PARAM::str_arena_frame;
            			view.header.stamp= ros::Time::now();
            			view.polygon.points.clear();
            			viewdanger_pub[k].publish(view);
            		}
            	}
            }

            ros::spinOnce();
            r0.sleep();
    }

    LOG(INFO) << "quit!";
   	printf("Quit Cruise!\n");


    return 0;
}

void IARCCommand_callback(const iarc_arena_simulator::IARCCommand::ConstPtr & cmd)
{

	theCruise.update_latest_cmdtime(cmd);

}
void QuadStatus_callback(const iarc_arena_simulator::IARCQuadStatus::ConstPtr & status)
{

	theCruise.update_quad(status);

}
void IARCObstacles_callback(const geometry_msgs::PoseArray::ConstPtr& obstacles)
{

	theCruise.update_obs(obstacles);

}
void IARCTargets_callback(const geometry_msgs::PoseArray::ConstPtr& targets)
{

	theCruise.update_tgt(targets);

	theCruise.update_map_memory();

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
		else if(tasks_list.list[k].robot_cmd == turn_45){
					p.position.x = tasks_list.list[k].final_pose.position.x;
					p.position.y = tasks_list.list[k].final_pose.position.y;
					p.position.z = 1.0;
				    p.orientation.x = 0.0;
				    p.orientation.y = sin( M_PI / 4);
				    p.orientation.z = 0.0;
				    p.orientation.w = cos(M_PI / 4);
					tasks.poses.push_back(p);
		}
		else{
			p.position.x = tasks_list.list[k].final_pose.position.x;
			p.position.y = tasks_list.list[k].final_pose.position.y;
			p.position.z = 2.0;
			p.orientation.x = 0.0;
			p.orientation.y = sin(0.0);
			p.orientation.z = 0.0;
			p.orientation.w = cos(0.0);
			tasks.poses.push_back(p);
		}
	}
}

void GenerateView(const IARCQuad &quad,  geometry_msgs::PolygonStamped &view)
{
	double x = quad.x;
	double y = quad.y;
	double z = quad.z;
	double de = 2.0*M_PI/36.0;
	double radius =  fabs(z)*tan(PARAM::angle_field_rad/2.0);
	view.header.frame_id = PARAM::str_arena_frame;
	view.header.stamp = ros::Time::now();
    view.polygon.points.clear();


	for(int k = 0; k * de < 2.0*M_PI; k++){
		geometry_msgs::Point32 pt;
		pt.x = x + radius * cos(((double)k)*de);
		pt.y = y + radius * sin(((double)k)*de);
		pt.z = 0.2;
		view.polygon.points.push_back(pt);
	}
}

void GenerateDangerView(const IARCRobot &obs,  geometry_msgs::PolygonStamped &view)
{
	double x = obs.x;
	double y = obs.y;
	double z = 3.0;
	double de = 2.0*M_PI/10.0;
	double radius =  (PARAM::radius_safe);
	view.header.frame_id = PARAM::str_arena_frame;
	view.header.stamp = ros::Time::now();
    view.polygon.points.clear();

	for(int k = 0; k * de < 2.0*M_PI; k++){
		geometry_msgs::Point32 pt;
		pt.x = x + radius * cos(((double)k)*de);
		pt.y = y + radius * sin(((double)k)*de);
		pt.z = 0.2;
		view.polygon.points.push_back(pt);
	}
}
