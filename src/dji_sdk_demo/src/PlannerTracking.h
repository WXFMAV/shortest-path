/*
 * PlannerTracking.h
 *
 *  Created on: 2017年4月19日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_PLANNERTRACKING_H_
#define DJI_SDK_DEMO_SRC_PLANNERTRACKING_H_

#include <iarc_arena_simulator/IARCWaypointsList.h>
#include "Quadrotor.h"
#include "common.h"
#include "G_Controller.h"
#include "G_Controller_LQR.h"
#include "G_Controller_PID.h"
#include <vector>
#include "PlannerVirtual.h"

enum enum_controller{none,lqr,pid};

class PlannerTracking : public PlannerVirtual{
public:
    PlannerTracking();
    virtual ~PlannerTracking();
    int init();
    int update_quad(Quadrotor *theQuad);
    int update_waypoints_list(const iarc_arena_simulator::IARCWaypointsList::ConstPtr& waypoints_list);
    int gen_control_cmd(IARC_COMMAND &cmd);
    iarc_arena_simulator::IARCWaypoint get_waypoint_now();
    Eigen::VectorXd get_vector_from_waypoint(const iarc_arena_simulator::IARCWaypoint &wp);
    int set_y();
    int set_state(Eigen::VectorXd state_x);
    int set_model(enum_controller controller_model);
    int get_control(Eigen::VectorXd control_u);
public:
    enum {ERROR,OK} _status;
    enum_controller _controller_model;
    G_Controller* _controller;
    Eigen::VectorXd _state_x;
    Eigen::VectorXd _control_u;
    std::vector<iarc_arena_simulator::IARCWaypoint> _wp_list;
    //std::vector<Eigen::VectorXd> _path_list;
    //std::vector<uint32_t> _time_list;
    uint32_t _pos_now;
    uint32_t _pos_end;
    uint32_t _seq_saved;
    uint32_t _saved_sent_cmd_task_seq;
private:
    FILE *_fp_tracking;
};

#endif /* DJI_SDK_DEMO_SRC_PLANNERTRACKING_H_ */
