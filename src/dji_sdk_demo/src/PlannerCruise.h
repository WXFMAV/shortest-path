/*
 * PlannerCruise.h
 *
 *  Created on: 2017年5月15日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_PLANNERCRUISE_H_
#define DJI_SDK_DEMO_SRC_PLANNERCRUISE_H_

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
#include "PlannerVirtual.h"

class PlannerCruise : public PlannerVirtual{
public:
    enum{ERROR, OK} _status;
    enum{NONE,MAXQ_OP,REACTIVE} _planner;
    uint32_t _tasks_seq;
    uint32_t _plan_seq;
public:
    PlannerCruise();
    virtual ~PlannerCruise();
    int init();
    int plan(iarc_arena_simulator::IARCTasksList &taskslist);
private:
    std::vector<iarc_arena_simulator::IARCTask> _tree;
    std::vector<std::vector<iarc_arena_simulator::IARCTask> > _tree_predictor;
    std::vector<iarc_arena_simulator::IARCTask> _tree_best;
    std::vector<IARCRobot> _tree_robot_predictor;
    double _tree_best_value;
    double _tree_value;
    std::vector<double> _tree_predictor_value;
    std::vector<iarc_arena_simulator::IARCTask> _list_saved;
    FILE *_fp_task;
    FILE *_fp_task_env;
    uint32_t _loop_start_time;
private:
    int maxq_op(iarc_arena_simulator::IARCTasksList &taskslist);
    int generate_list(std::vector<iarc_arena_simulator::IARCTask> &taskslist);
    int add_one_task(iarc_arena_simulator::IARCTask task, std::vector<iarc_arena_simulator::IARCTask> &taskslist);
    int tree_search(int depth,double coef);
    int tree_search_simple(int depth, double coef);
    int tree_search_map(int depth, double coef);
    IARCRobot tree_predictor_zero_input(uint32_t rtime, IARCRobot robot, double final_turn = 0.0);
    double reward_tgt_one(const IARCRobot &robot);
    double reward(uint32_t time_ms,  const std::vector<IARCRobot> &targets, const std::vector<IARCRobot> &obstacles, const IARCRobot &mav_robot, robotcmd_kind cmd);
    double reward_Rm(uint32_t time_ms, const IARCRobot & mav_robot, robotcmd_kind cmd);
    double reward_Ro(uint32_t time_ms, const std::vector<IARCRobot> &obstacles, const IARCRobot &mav_robot);
    double reward_Rt(uint32_t time_ms, const std::vector<IARCRobot> &targets);
    double reward_Rt2(uint32_t time_ms, const IARCRobot &targets);
    double reward2(uint32_t time_ms, const IARCRobot & focus_robot, const std::vector<IARCRobot> &obstacles, robotcmd_kind cmd);
    double reward_map(const std::vector<iarc_arena_simulator::IARCTask> &tree, double time, double &bestx, double &besty, double &bestz);
    int legal_task(const iarc_arena_simulator::IARCTask &node1,const iarc_arena_simulator::IARCTask &node2);
    bool require_generate_list();
    iarc_arena_simulator::IARCTask get_current_task();
};

#endif /* DJI_SDK_DEMO_SRC_PLANNERCRUISE_H_ */
