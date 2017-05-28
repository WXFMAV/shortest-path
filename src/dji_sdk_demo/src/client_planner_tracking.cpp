#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <iarc_arena_simulator/IARCCommand.h>
#include <iarc_arena_simulator/IARCWaypoint.h>
#include <iarc_arena_simulator/IARCWaypointsList.h>
#include <iarc_arena_simulator/IARCQuadStatus.h>
#include <ros/package.h>
#include <glog/logging.h>
#include "PlannerTracking.h"
#include "Quadrotor.h"
#include "common.h"

static PlannerTracking theTracker;
static Quadrotor theQuad;
static ros::Publisher waypoints_path_pub;
static ros::Publisher path_pub;
static ros::Publisher robotcmd_pub;
static ros::Publisher mav_pub;
static ros::Publisher quad_status_pub;
static ros::Subscriber waypoints_list_sub;

int GenerateQuadStatus(Quadrotor *theQuad, iarc_arena_simulator::IARCQuadStatus &quad_status);
int GenerateUAVPath3(Quadrotor *theQuad, nav_msgs::Path &path);
int GenerateUAVPose(Quadrotor *theQuad, geometry_msgs::PoseStamped &pose);
void IARCWaypointsList_callback(const iarc_arena_simulator::IARCWaypointsList::ConstPtr& waypoints_list);

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "flight_control");
    ros::NodeHandle nh;

    arena_set_startnow();

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir=ros::package::getPath("dji_sdk_demo")+"/../../build/test_results";
    std::cout<<FLAGS_log_dir<<std::endl;
    LOG(INFO) << "record info to this file";

    robotcmd_pub = nh.advertise<iarc_arena_simulator::IARCCommand>("/iarc_arena/IARCCommand", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/iarc_arena/nav_path", 10);
    mav_pub = nh.advertise<geometry_msgs::PoseStamped>("/iarc_arena/IARCMav", 10);
    quad_status_pub = nh.advertise<iarc_arena_simulator::IARCQuadStatus>("/iarc_arena/IARCQuadStatus", 10);
    waypoints_path_pub = nh.advertise<nav_msgs::Path>("/iarc_arena/waypoints_path", 10);
    waypoints_list_sub = nh.subscribe<iarc_arena_simulator::IARCWaypointsList>("/iarc_arena/IARCWaypointsList", 10, IARCWaypointsList_callback);

    if( theQuad.init(model_simulator, nh) < 0){
        LOG(ERROR) << "Quadrotor model init error!";
        return -1;
    }
    LOG(INFO) << "Quadrotor model init success!";

    if (theTracker.init() < 0){
        LOG(ERROR) << "Tracking controller init error!";
        return -1;
    }
    LOG(INFO) << "Tracking controller init success!";

    ros::Rate r0(PARAM::ros_rate_tracking);
	int cnt=0;

	LOG(INFO) << "ros ok=" <<ros::ok();

	while(ros::ok())
	{
	        cnt++;
	        LOG(INFO) << "loop:" <<cnt;

            IARC_COMMAND cmd = make_zero_cmd();

            theTracker.update_quad(&theQuad);
            theTracker.gen_control_cmd(cmd);
            theQuad.exe_control_cmd(cmd);

            iarc_arena_simulator::IARCQuadStatus quad_status;
            GenerateQuadStatus(&theQuad,quad_status);
            quad_status_pub.publish(quad_status);

            if(cmd.robot_turn_kind != robotcmd_kind::turn_none)
            {
                  iarc_arena_simulator::IARCCommand robotcmd;
                  robotcmd.header.frame_id = PARAM::str_arena_frame;
                  robotcmd.header.stamp = ros::Time::now();
                  robotcmd.command_kind = cmd.robot_turn_kind;
                  robotcmd.robot_id = cmd.robot_turn_id;
                  robotcmd_pub.publish(robotcmd);
            }

            nav_msgs::Path path;
            GenerateUAVPath3(&theQuad, path);
            path_pub.publish(path);

            geometry_msgs::PoseStamped pose;
            GenerateUAVPose(&theQuad, pose);
            mav_pub.publish(pose);

            ros::spinOnce();
            r0.sleep();
	}
	LOG(INFO) << "quit!";
	printf("Quit Tracking!\n");

	return 0;
}

void IARCWaypointsList_callback(const iarc_arena_simulator::IARCWaypointsList::ConstPtr& waypoints_list)
{
    if(waypoints_list->list.size() <=0) return ;

    theTracker.update_waypoints_list(waypoints_list);

    nav_msgs::Path path;
    path.header.frame_id = "/arena_frame";
    path.header.stamp = ros::Time::now();
    double x, y, z;

    for(int i = 0; i <waypoints_list->list.size();  i++)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = path.header.frame_id;
        ps.header.stamp = ros::Time::now();
        ps.pose.position.x = waypoints_list->list[i].x;
        ps.pose.position.y = waypoints_list->list[i].y;
        ps.pose.position.z = waypoints_list->list[i].z;
        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = 0.0;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
      }
    waypoints_path_pub.publish(path);
}
int  GenerateQuadStatus(Quadrotor*theQuad,iarc_arena_simulator::IARCQuadStatus &quad_status)
{
    double x, y, z, vx, vy, vz, ax, ay, az;
    theQuad->get_pos(x,y,z);
    theQuad->get_vel(vx,vy,vz);
    theQuad->get_acc(ax,ay,az);
    quad_status.x = x; quad_status.y = y; quad_status.z = z;
    quad_status.vx = vx; quad_status.vy = vy; quad_status.vz = vz;
    quad_status.ax = ax; quad_status.ay = ay; quad_status.az = az;
    quad_status.header.frame_id = "/arena_frame";
    quad_status.header.stamp = ros::Time::now();
    quad_status.seq = 0;
    quad_status.tms = arena_time_now();
    return 0;
}
int GenerateUAVPath3(Quadrotor *theQuad, nav_msgs::Path &path)
{
#define  QN  2
static double Q_path[QN][3];
static unsigned int Q_cnt=0;

      path.header.frame_id = "/arena_frame";
      path.header.stamp = ros::Time::now();
      double x, y, z;

      theQuad->get_pos(x, y, z);
      Q_path[Q_cnt % QN][0] = x;
      Q_path[Q_cnt % QN][1] = y;
      Q_path[Q_cnt % QN][2] = z;
      Q_cnt++;

      path.poses.clear();
      for(int i = (Q_cnt-QN >= 0 ? Q_cnt - QN : 0); i < Q_cnt; i++)
      {
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = path.header.frame_id;
            ps.header.stamp = ros::Time::now();
            ps.pose.position.x = Q_path[i % QN][0];
            ps.pose.position.y = Q_path[i % QN][1];
            ps.pose.position.z = Q_path[i % QN][2];
            ps.pose.orientation.x = 0.0;
            ps.pose.orientation.y = 0.0;
            ps.pose.orientation.z = 0.0;
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(ps);
      }
#undef QN

    return 0;
}
int GenerateUAVPose(Quadrotor *theQuad, geometry_msgs::PoseStamped &pose)
{
    pose.header.frame_id = "/arena_frame";
    pose.header.stamp = ros::Time::now();
    double x, y, z;
    theQuad->get_pos(x, y, z);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = sin( -M_PI / 4);
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = cos(-M_PI/4);
    return 0;
}
