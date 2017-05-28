/*
 * common.cpp
 *
 *  Created on: 2017年4月21日
 *      Author: Hualin He
 */

#include "common.h"
#include <math.h>
#include <ros/ros.h>
#include <glog/logging.h>

static ros::Time time_start;
static bool visited = false;
using namespace iarc_arena_simulator;
uint32_t arena_time_now()
{
    if (!visited){
        LOG(ERROR) << "arena start time not set!";
        return 0;
    }
    ros::Time now = ros::Time::now();
    if(now.sec<time_start.sec) return 0;
    uint32_t ans_sec,ans_nsec;
    ans_sec=now.sec-time_start.sec;
    if(now.nsec>time_start.nsec)
    {
          ans_nsec=now.nsec-time_start.nsec;
    }
    else
    {
          ans_nsec=now.nsec+1e9-time_start.nsec;
          ans_sec--;
    }
//    ans_nsec=now.nsec>start.nsec?now.nsec-start.nsec:now.nsec+1e9-start.nsec;
    return uint32_t(((uint64_t)ans_sec)*1000+((uint64_t)ans_nsec)/1000000);
}
void arena_set_startnow()
{
    if (visited){
        LOG(ERROR) << "arena time can only be set once!";
        return ;
    }
    time_start = ros::Time::now();
    visited = true;
}

double quaternion_to_theta_z(double q_w, double q_z)
{
      double eq=0.0;
      eq=atan2(q_w,q_z);
      return M_PI-2*eq;
}

IARCTask make_task_type_reach(std::string frame_id, ros::Time stamp, double aimx,
        double aimy, double aimz, uint32_t start_time_ms, uint32_t aim_time_ms)
{
    IARCTask task;
    task.header.frame_id = frame_id;
    task.header.stamp = stamp;
    task.task_seq = 0;
    task.task_type = task_type::type_reach;
    task.task_value = 0.0;
    task.robot_id = robot_id::robot_arena;
    task.robot_cmd = robotcmd_kind::turn_none;
    task.time_start = start_time_ms;
    task.time_end = aim_time_ms;
    task.final_pose.position.x = aimx;
    task.final_pose.position.y = aimy;
    task.final_pose.position.z = aimz;
    task.final_pose.orientation.w = 1.0;
    task.final_pose.orientation.x = 0.0;
    task.final_pose.orientation.y = 0.0;
    task.final_pose.orientation.z = 0.0;
    return task;
}
IARCTask make_task_type_hover(std::string frame_id, ros::Time stamp, uint32_t start_time_ms, uint32_t aim_time_ms)
{
    IARCTask task;
    task.header.frame_id = frame_id;
    task.header.stamp = stamp;
    task.task_seq = 0;
    task.task_type = task_type::type_hover;
    task.task_value = 0.0;
    task.robot_id = robot_id::robot_arena;
    task.robot_cmd = robotcmd_kind::turn_none;
    task.time_start = start_time_ms;
    task.time_end = aim_time_ms;
    task.final_pose.position.x = 0.0;
    task.final_pose.position.y = 0.0;
    task.final_pose.position.z = 0.0;
    task.final_pose.orientation.w = 1.0;
    task.final_pose.orientation.x = 0.0;
    task.final_pose.orientation.y = 0.0;
    task.final_pose.orientation.z = 0.0;
    return task;
}

iarc_arena_simulator::IARCTask make_task_type_follow(std::string frame_id,
        ros::Time stamp, uint32_t robot_id, uint32_t start_time_ms, uint32_t end_time_ms)
{
    IARCTask task;
    task.header.frame_id = frame_id;
    task.header.stamp = stamp;
    task.task_seq = 0;
    task.task_type = task_type::type_follow;
    task.task_value = 0.0;
    task.robot_id = robot_id;
    task.robot_cmd = robotcmd_kind::turn_none;
    task.time_start = start_time_ms;
    task.time_end = end_time_ms;
    task.final_pose.position.x = 0.0;
    task.final_pose.position.y = 0.0;
    task.final_pose.position.z = 0.0;
    task.final_pose.orientation.w = 1.0;
    task.final_pose.orientation.x = 0.0;
    task.final_pose.orientation.y = 0.0;
    task.final_pose.orientation.z = 0.0;
    return task;
}

IARC_COMMAND make_zero_cmd()
{
    IARC_COMMAND cmd;
    cmd.mav_vx = 0.0;
    cmd.mav_vy = 0.0;
    cmd.mav_vz = 0.0;
    cmd.mav_vyawdegree = 0.0;
    cmd.robot_turn_kind = robotcmd_kind::turn_none;
    cmd.robot_turn_id = robot_id::robot_arena;
    return cmd;
}

iarc_arena_simulator::IARCWaypoint make_waypoint(const std::string &frame_id, const ros::Time &stamp, uint32_t seq, uint32_t time_now, const Eigen::VectorXd & _state_x,
                robotcmd_kind  turn_kind, robot_id id)
{
    iarc_arena_simulator::IARCWaypoint wp;
    wp.header.frame_id = frame_id;
    wp.header.stamp = stamp;
    wp.seq = seq;
    wp.tms = time_now;
    wp.x = _state_x[0]; wp.y = _state_x[1]; wp.z = _state_x[2];
    wp.vx = _state_x[3]; wp.vy = _state_x[4]; wp.vz = _state_x[5];
    wp.ax = _state_x[6]; wp.ay = _state_x[7]; wp.az = _state_x[8];
    wp.robot_cmd = turn_kind;
    wp.robot_id = id;
    return wp;
}

IARCRobot make_prediction_robot(uint32_t rtime, IARCRobot robot, double final_turn)
{
    if(robot.time_ms <= rtime)
    {
          double theta = robot.theta;
          int cntloop_times=0;
          while(1)
          {
                int32_t newtime = (robot.time_ms/PARAM::period_turn_ms + 1) * PARAM::period_turn_ms;

                if(newtime <= rtime)
                {
                      if(newtime - robot.time_ms > PARAM::time_turn_180_ms)
                      {
                            robot.x = robot.x + robot.velocity*cos(theta)*((double)(newtime-robot.time_ms-PARAM::time_turn_180_ms)/1000.0);
                            robot.y=robot.y+robot.velocity*sin(theta)*((double)(newtime-robot.time_ms-PARAM::time_turn_180_ms)/1000.0);
                            robot.time_ms = newtime;
                            theta=theta+M_PI;
                      }
                      else
                      {
                            robot.x=robot.x+0.0;
                            robot.y=robot.y+0.0;
                            robot.time_ms=newtime;
                            theta=theta+M_PI;//theta=the+M_PI;
                      }
                }
                else
                {
                      break;
                }
                cntloop_times++;
          }

          if(robot.time_ms <= rtime && rtime%PARAM::period_turn_ms<=PARAM::period_turn_ms-PARAM::time_turn_180_ms)
              {
                   //   printf("dec:%d\n",(rtime-robot.time_ms));
                    double lftime=((double)(rtime-robot.time_ms))/1000.0;
                    robot.x=robot.x+robot.velocity*cos(theta)*lftime;
                    robot.y=robot.y+robot.velocity*sin(theta)*lftime;
                    robot.time_ms=rtime;
              }
              else if(robot.time_ms<=rtime && robot.time_ms%PARAM::period_turn_ms<PARAM::period_turn_ms-PARAM::time_turn_180_ms)
              {
                   // printf("dec2:%d\n",(PERIOD_TURN-TIME_TURN_180-robot.time_ms%PERIOD_TURN));
                    double lftime=((double)(PARAM::period_turn_ms-PARAM::time_turn_180_ms-robot.time_ms%PARAM::period_turn_ms))/1000.0;
                    robot.x=robot.x+robot.velocity*cos(theta)*lftime;
                    robot.y=robot.y+robot.velocity*sin(theta)*lftime;
                    robot.time_ms=rtime;
                    //theta=//already turnning... but I don't turn this one!
              }
              else
              {
                    robot.time_ms=rtime;
              }
          theta=theta+final_turn;
          robot.theta=theta;
    }
    else
    {
          LOG(ERROR) << "error! can not predict history!"<< robot.time_ms<<" "<<rtime;
    }
    return robot;

}

IARCRobot get_robot_from_id(const std::vector<IARCRobot> &tgt, uint32_t id)
{
	for( int k = 0; k< tgt.size(); k++)
		if ( tgt[k].id == id ){
			return tgt[k];
		}
	LOG(ERROR) << " seeking for unexist robot id="<< id;

	IARCRobot robot;
	robot.id = robot_none;
	robot.x = 0.0;
	robot.y = 0.0;
	robot.theta = 0.0;
	robot.time_ms = 0;
	robot.velocity = 0.0;
	return robot;
}

double forget_function(uint32_t det_time_ms){
	if(det_time_ms > PARAM::target_time_forget_ms) return 0.0;
	return 1.0-( (double)(det_time_ms))/((double)(PARAM::target_time_forget_ms));
}

