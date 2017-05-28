#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <iarc_arena_simulator/IARCCommand.h>

#include "iarc5.h"

using namespace DJI::onboardSDK;
extern class CIARC theIARC;
ros::Time time_start;

uint32_t TimePassedMs(ros::Time start,ros::Time now);
int GenerateUAVPath(DJIDrone *drone,nav_msgs::Path &path);
int GenerateUAVPath2(CIARC *theIARC,nav_msgs::Path &path);
void IARCObstacles_callback(const geometry_msgs::PoseArray::ConstPtr& obstacles);
void IARCTargets_callback(const geometry_msgs::PoseArray::ConstPtr& targets);

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "flight_control");

    ros::NodeHandle nh;

    time_start=ros::Time::now();
    ros::Publisher robotcmd_pub = nh.advertise<iarc_arena_simulator::IARCCommand>("/iarc_arena/IARCCommand",10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("nav_path",10);
    ros::Publisher mav_pub = nh.advertise<geometry_msgs::PoseStamped>("/iarc_arena/IARCMav",10);
    ros::Publisher predicted_path_pub = nh.advertise<nav_msgs::Path>("/iarc_arena/predicted_path",10);
    ros::Publisher task_pub = nh.advertise<geometry_msgs::PoseStamped>("/iarc_arena/IARCTask",10);
    ros::Subscriber obstacles_sub = nh.subscribe<geometry_msgs::PoseArray>("/iarc_arena/IARCObstacles",10,IARCObstacles_callback);
    ros::Subscriber targets_sub = nh.subscribe<geometry_msgs::PoseArray>("/iarc_arena/IARCTargets",10,IARCTargets_callback);

    ros::Rate r0(50);
	int cnt=0;

	theIARC.mav.x=0.0;
    theIARC.mav.y=0.0;
    theIARC.mav.vx=0.0;
    theIARC.mav.vy=0.0;

	while(ros::ok())
	{
            cnt++;

            IARC_COMMAND cmd;

            //theIARC.mav.x=0.0;
            //theIARC.mav.y=0.0;
            //theIARC.mav.vx=0.0;
            //theIARC.mav.vy=0.0;

            theIARC.mav.z=0.0;
            theIARC.mav.vz=0.0;

            theIARC.mav.time_ms=TimePassedMs(time_start,ros::Time::now());

            theIARC.mav.orientation.x=0.0;
            theIARC.mav.orientation.y=0.0;
            theIARC.mav.orientation.z=0.0;
            theIARC.mav.orientation.w=1.0;

            theIARC.mav.param_ax    =DEFAULT_PARAM_AX;
            theIARC.mav.param_kx    =DEFAULT_PARAM_KX;
            theIARC.mav.param_vmx =DEFAULT_PARAM_VMX;
            theIARC.mav.param_ay    =DEFAULT_PARAM_AY;
            theIARC.mav.param_ky    =DEFAULT_PARAM_KY;
            theIARC.mav.param_vmy =DEFAULT_PARAM_VMY;
            theIARC.mav.param_vm   =DEFAULT_PARAM_VM;

            theIARC.MainLoop(cmd);

            if(cmd.robot_turn_kind!=KIND_TURN_NONE)
            {
                  iarc_arena_simulator::IARCCommand robotcmd;
                  robotcmd.header.frame_id="/arena_frame";
                  robotcmd.header.stamp = ros::Time::now();
                  robotcmd.command_kind=cmd.robot_turn_kind;
                  robotcmd.robot_id=cmd.robot_turn_id;
                  robotcmd_pub.publish(robotcmd);
            }
            printf("%d %.2lf %.2lf %.2lf %.2lf\n",cnt,cmd.mav_vx,cmd.mav_vy,cmd.mav_vz,cmd.mav_vyawdegree);

            theIARC.Simulator(1/50.0);
           // printf("here\n");

            nav_msgs::Path path;
            GenerateUAVPath2(&theIARC,path);
            path_pub.publish(path);

            nav_msgs::Path pre_path;
            pre_path.header.frame_id="/arena_frame";
            pre_path.header.stamp=ros::Time::now();
            for(int i=0;i<theIARC.pre_path.size(); i++)
                  {
                        geometry_msgs::PoseStamped ps;
                        ps.header.frame_id = pre_path.header.frame_id;
                        ps.header.stamp = pre_path.header.stamp;
                        ps.pose.position.x=theIARC.pre_path[i].x;
                        ps.pose.position.y=theIARC.pre_path[i].y;
                        ps.pose.position.z=theIARC.pre_path[i].z;
                        ps.pose.orientation.x=0.0;
                        ps.pose.orientation.y=0.0;
                        ps.pose.orientation.z=0.0;
                        ps.pose.orientation.w=1.0;

                        pre_path.poses.push_back(ps);
                  }

            predicted_path_pub.publish(pre_path);

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id="/arena_frame";
            pose.header.stamp=ros::Time::now();
            pose.pose.position.x=theIARC.mav.x;
            pose.pose.position.y=theIARC.mav.y;
            pose.pose.position.z=theIARC.mav.z+1.5;

            pose.pose.orientation.x=0.0;
            pose.pose.orientation.y=sin(-M_PI/4);
            pose.pose.orientation.z=0.0;
            pose.pose.orientation.w=cos(-M_PI/4);

            mav_pub.publish(pose);
            for(int p=theIARC.task_pos; p<theIARC.task_list.size(); p++)
            {
                  if(theIARC.task_list[p].type==type_interact)
                  {
                        pose.pose.position.x=theIARC.task_list[p].content.interact_exp_x;
                        pose.pose.position.y=theIARC.task_list[p].content.interact_exp_y;
                        pose.pose.position.z=0.5;
                        pose.pose.orientation.x=0.0;
                        pose.pose.orientation.y=sin(-M_PI/4);
                        pose.pose.orientation.z=0.0;
                        pose.pose.orientation.w=cos(-M_PI/4);
                        task_pub.publish(pose);
                        break;
                  }
            }
            for(vector<IARC_TASK>::iterator it=theIARC.task_list.begin(); it!=theIARC.task_list.end(); it++)
            {
                  if(it->type==type_interact)
                  {
                        pose.pose.position.x=it->content.interact_exp_x;
                        pose.pose.position.y=it->content.interact_exp_y;
                        pose.pose.position.z=0.5;
                        pose.pose.orientation.x=0.0;
                        pose.pose.orientation.y=sin(-M_PI/4);
                        pose.pose.orientation.z=0.0;
                        pose.pose.orientation.w=cos(-M_PI/4);
                        task_pub.publish(pose);
                        break;
                  }
            }
            //pose.pose.position.x=
            ros::spinOnce();
            r0.sleep();
	}
	return 0;
	
    DJIDrone* drone = new DJIDrone(nh);    
   	for(int i=0;i<10;i++)
	{
		drone->activate();
		usleep(10000);
		printf("adrone->activation==%d\n",drone->activation);
		if(!drone->activation)		
		{
		    ROS_INFO("active success ok !!");
			break;
			
		} 
	}
	
	/* request control ability*/	
	for(int i=0;i<10;i++)
	{
		bool success=false;
		success=drone->request_sdk_permission_control();
		usleep(10000);
		printf("obtain control success==%d\n",success);
		if(success==1)		
		{
		    printf("obtain control success");
			break;
			
		} 
	}
			                
	/* take off */
	drone->takeoff();
	
    sleep(8);
			
	ros::Rate r(50);				//the frequency = 50Hz
	//ros::spin();
	
	while(ros::ok())
	{	
		double cmd_vx,cmd_vy,cmd_vz,cmd_vyawdegree;
		cmd_vx=0.0;
		cmd_vy=0.0;
		cmd_vz=0.0;
		cmd_vyawdegree=0.0;
											
	 	//Quaternion_To_Euler(drone->attitude_quaternion.q0,drone->attitude_quaternion.q1,
	 	//	drone->attitude_quaternion.q2,drone->attitude_quaternion.q3,&yaw_uav,&pitch_uav,&roll_uav);	 						
	 	cmd_vx=0.5;
	 		 
		drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |		
                        Flight::VerticalLogic::VERTICAL_VELOCITY |
                        Flight::YawLogic::YAW_PALSTANCE |
                        Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                        Flight::SmoothMode::SMOOTH_ENABLE,
                        cmd_vx, cmd_vy, cmd_vz, cmd_vyawdegree );                        
                       
		//drone->gimbal_speed_control(0.0,pid_v.v,-pid_u.v);  
		
		nav_msgs::Path path;		
		GenerateUAVPath(drone,path);			
		//GenerateArenaMarker(marker);				
		path_pub.publish(path);			

		ros::spinOnce();
		r.sleep();
	}	
	
	return 0;
}
uint32_t TimePassedMs(ros::Time start,ros::Time now)
{
      if(now.sec<start.sec) return 0;
      uint32_t ans_sec,ans_nsec;
      ans_sec=now.sec-start.sec;
      if(now.nsec>start.nsec)
      {
            ans_nsec=now.nsec-start.nsec;
      }
      else
      {
            ans_nsec=now.nsec+1e9-start.nsec;
            ans_sec--;
      }
//    ans_nsec=now.nsec>start.nsec?now.nsec-start.nsec:now.nsec+1e9-start.nsec;
      return uint32_t(((uint64_t)ans_sec)*1000+((uint64_t)ans_nsec)/1000000);
}
void IARCObstacles_callback(const geometry_msgs::PoseArray::ConstPtr& obstacles)
{
      uint32_t time_now=TimePassedMs(time_start,ros::Time::now());
      for(int i=0; i<N_OBS; i++)
      {
            theIARC.obstacles[i].x=obstacles->poses[i].position.x;
            theIARC.obstacles[i].y=obstacles->poses[i].position.y;
            theIARC.obstacles[i].orientation.x=obstacles->poses[i].orientation.x;
            theIARC.obstacles[i].orientation.y=obstacles->poses[i].orientation.y;
            theIARC.obstacles[i].orientation.z=obstacles->poses[i].orientation.z;
            theIARC.obstacles[i].orientation.w=obstacles->poses[i].orientation.w;
            theIARC.obstacles[i].velocity=0.33;
            theIARC.obstacles[i].time_ms=time_now;
            theIARC.seq_obstacles=obstacles->header.seq;
            //printf("seq o:%d\n",obstacles->header.seq);
      }
}
void IARCTargets_callback(const geometry_msgs::PoseArray::ConstPtr& targets)
{
      uint32_t time_now=TimePassedMs(time_start,ros::Time::now());
      for(int i=0; i<N_TRG; i++)
      {
            theIARC.targets[i].x=targets->poses[i].position.x;
            theIARC.targets[i].y=targets->poses[i].position.y;
            theIARC.targets[i].orientation.x=targets->poses[i].orientation.x;
            theIARC.targets[i].orientation.y=targets->poses[i].orientation.y;
            theIARC.targets[i].orientation.z=targets->poses[i].orientation.z;
            theIARC.targets[i].orientation.w=targets->poses[i].orientation.w;
            theIARC.targets[i].velocity=0.33;
            theIARC.targets[i].time_ms=time_now;
            theIARC.seq_targets=targets->header.seq;
         //   printf("seq t:%d\n",targets->header.seq);
      }
}

int GenerateUAVPath2(CIARC *theIARC,nav_msgs::Path &path)
{
#define QN 1000
static double Q_path[QN][3];
static unsigned int Q_cnt=0;

      path.header.frame_id = "/arena_frame";
      path.header.stamp = ros::Time::now();

      printf("path:%.2lf %.2lf %.2lf\n",theIARC->mav.x,theIARC->mav.y,theIARC->mav.z);

      Q_path[Q_cnt%QN][0]=theIARC->mav.x;
      Q_path[Q_cnt%QN][1]=theIARC->mav.y;
      Q_path[Q_cnt%QN][2]=theIARC->mav.z;
      Q_cnt++;

      for(int i=Q_cnt-QN>=0?Q_cnt-QN:0; i<Q_cnt; i++)
      {
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = path.header.frame_id;
            ps.header.stamp = ros::Time::now();
            ps.pose.position.x=Q_path[i%QN][0];
            ps.pose.position.y=Q_path[i%QN][1];
            ps.pose.position.z=Q_path[i%QN][2];
            ps.pose.orientation.x=0.0;
            ps.pose.orientation.y=0.0;
            ps.pose.orientation.z=0.0;
            ps.pose.orientation.w=1.0;

            path.poses.push_back(ps);
      }

      return 0;
}

int GenerateUAVPath(DJIDrone *drone,nav_msgs::Path &path)
{
#define QN 1000
static double Q_path[QN][3];
static unsigned int Q_cnt=0;

	path.header.frame_id = "/arena_frame";			
	path.header.stamp = ros::Time::now();
	
	Q_path[Q_cnt%QN][0]=drone->local_position.x;
	Q_path[Q_cnt%QN][1]=drone->local_position.y;
	Q_path[Q_cnt%QN][2]=drone->local_position.z;
	Q_cnt++;
	
	for(int i=Q_cnt-QN>=0?Q_cnt-QN:0; i<Q_cnt; i++)
	{
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = path.header.frame_id;
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x=Q_path[i%QN][0];
		ps.pose.position.y=Q_path[i%QN][1];
		ps.pose.position.z=Q_path[i%QN][2];
		ps.pose.orientation.x=0.0;
		ps.pose.orientation.y=0.0;		
		ps.pose.orientation.z=0.0;				
		ps.pose.orientation.w=1.0;
		
		path.poses.push_back(ps);	
	}
	
	return 0;
}
