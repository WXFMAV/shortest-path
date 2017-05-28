/*
 * PlannerVirtual.h
 *
 *  Created on: 2017年5月23日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_PLANNERVIRTUAL_H_
#define DJI_SDK_DEMO_SRC_PLANNERVIRTUAL_H_

#include <iarc_arena_simulator/IARCWaypoint.h>
#include <iarc_arena_simulator/IARCWaypointsList.h>
#include <iarc_arena_simulator/IARCQuadStatus.h>
#include <iarc_arena_simulator/IARCTasksList.h>
#include <iarc_arena_simulator/IARCTask.h>
#include <iarc_arena_simulator/IARCCommand.h>
#include <geometry_msgs/PoseArray.h>
#include "common.h"
#include <vector>
#include <map>


class PlannerVirtual {

public:
    std::vector<IARCRobot> _tgt_status;
    std::vector<IARCRobot> _obs_status;
    IARCQuad _quad_status;
    std::map<int, IARCRobot> _tgts_saved;
    FILE *_fp_map;
    std::vector<std::vector<double>> _map_memory;
    int32_t _map_size_x;
    int32_t _map_size_y;
    uint32_t _latest_cmdtime;
    uint32_t _latest_cmd_continue_time;
    iarc_arena_simulator::IARCCommand _latest_cmd;

public:
	PlannerVirtual();
	virtual ~PlannerVirtual();
    int update_quad(const iarc_arena_simulator::IARCQuadStatus::ConstPtr &quad_status);
    int update_obs(const geometry_msgs::PoseArray::ConstPtr& obstacles);
    int update_tgt(const geometry_msgs::PoseArray::ConstPtr& targets);
    int update_latest_cmdtime(const iarc_arena_simulator::IARCCommand::ConstPtr& cmd);
    int update_map_memory();
    int arena2map(double x, double y, int &x_i, int &y_i);
    int map2arena(int x_i, int y_i, double &x, double &y);
    bool in_grid_map(int xi, int yi);
    bool in_sight(const IARCRobot &r);
    bool in_sight_xy(double x, double y, double x0, double y0, double h0, double angle_rad);
    bool in_arena(const IARCRobot & r);
    bool in_arena_xy(double x, double y);
    bool in_esacaped(const IARCRobot &r);
    bool in_sheephold(const IARCRobot &r);
    bool will_in_sheephold(const IARCRobot & robot);
    IARCRobot get_robot_by_id(uint32_t robot_id);
    IARCRobot get_saved_robot_by_id(uint32_t robot_id);
    int set_saved_robot(const std::vector<IARCRobot> &tgt);
    int save_map();
    double robot_dis(const IARCRobot &r1, const IARCRobot &r2);
    uint32_t get_time_sendcmd(const iarc_arena_simulator::IARCTask &task);
    bool found_tgt();
    int get_x_of_edge_id(int id);
    int get_y_of_edge_id(int id);
    int get_k_of_edge_id(int id);
    int get_nodes_edge_id(int ki, int xi, int yi);
    int get_edge_first_grid_by_id(int id, int &xi, int &yi);
    bool edge_in_safe_area(int xi1, int yi1, int xi2, int yi2);
    int segment_in_circle(IARC_POSITION *p1, IARC_POSITION *p2, IARC_POSITION O, double r);
};

#endif /* DJI_SDK_DEMO_SRC_PLANNERVIRTUAL_H_ */
