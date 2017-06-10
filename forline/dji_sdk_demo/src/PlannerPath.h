/*
 * PlannerPath.h
 *
 *  Created on: 2017年4月19日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_PLANNERPATH_H_
#define DJI_SDK_DEMO_SRC_PLANNERPATH_H_

#include <iarc_arena_simulator/IARCWaypoint.h>
#include <iarc_arena_simulator/IARCWaypointsList.h>
#include <iarc_arena_simulator/IARCQuadStatus.h>
#include <iarc_arena_simulator/IARCTasksList.h>
#include <iarc_arena_simulator/IARCTask.h>
#include <geometry_msgs/PoseArray.h>
#include "common.h"
#include <vector>
#include "PlannerVirtual.h"
#include "G_Graph.h"

class PlannerPath : public PlannerVirtual{
public:
    PlannerPath();
    virtual ~PlannerPath();
    int init();
    int plan(iarc_arena_simulator::IARCWaypointsList &wplist);
    int update_taskslist(const iarc_arena_simulator::IARCTasksList::ConstPtr &taskslist);
    int test_main();
    int test_main_plan_dijkstra(iarc_arena_simulator::IARCWaypointsList &wplist);
private:
    int planner_test(iarc_arena_simulator::IARCWaypointsList &wplist);
    int plan_with_obs(iarc_arena_simulator::IARCWaypointsList &wplist);
    int plan_PureTracking(iarc_arena_simulator::IARCWaypointsList &wplist);
    int plan_djikstra(iarc_arena_simulator::IARCWaypointsList &wplist);
    int planner_test_auto_task();
    bool ready_for_plan();
    iarc_arena_simulator::IARCTask get_task_now();
    std::vector<double> get_diff(std::vector<double> a, std::vector<double> time);
    int smooth_waypoints(std::vector<iarc_arena_simulator::IARCWaypoint> &list);
    int add_one_waypoint( std::vector<iarc_arena_simulator::IARCWaypoint> &list, uint32_t time_ms,
            double x, double y, double z, double vx, double vy, double vz ,double ax, double ay, double az,
            robot_id id, robotcmd_kind cmd);
    bool require_generate(double &dst_x, double &dst_y, double &dst_z, double &dst_time, enum robot_id &dst_robotid, enum robotcmd_kind &dst_turncmd);
	bool generate_path(std::vector<iarc_arena_simulator::IARCWaypoint> &list,  double time_start, double dst_x, double dst_y, double dst_z, double dst_time);
	bool generate_path(std::vector<iarc_arena_simulator::IARCWaypoint> &list,  double time_start, double dst_x, double dst_y, double dst_z, double dst_time, robot_id dst_robotid, robotcmd_kind dst_turncmd);
	bool graph_planing_path(std::vector<IARC_POSITION> &path,  double dst_x, double dst_y, double &total_len);
public:
    enum{ERROR, OK} _status;
    enum{test, A_star, BiBFS, PureTracking, Dijkstra} _planner;
    uint32_t _wp_seq;
    std::vector< iarc_arena_simulator::IARCTask> _tasks_list;
    uint32_t _pos_now;
    uint32_t _pos_end;
    uint32_t _task_seq_saved;
private:
    FILE *_fp_path;
    FILE *_fp_path_env;
};

#endif /* DJI_SDK_DEMO_SRC_PLANNERPATH_H_ */
