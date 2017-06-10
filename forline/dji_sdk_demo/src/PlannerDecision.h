/*
 * PlannerDecision.h
 *
 *  Created on: 2017年4月19日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_PLANNERDECISION_H_
#define DJI_SDK_DEMO_SRC_PLANNERDECISION_H_

#include <iarc_arena_simulator/IARCWaypoint.h>
#include <iarc_arena_simulator/IARCWaypointsList.h>
#include <iarc_arena_simulator/IARCQuadStatus.h>
#include <iarc_arena_simulator/IARCTasksList.h>
#include <iarc_arena_simulator/IARCTask.h>
#include <iarc_arena_simulator/IARCCommand.h>
#include <geometry_msgs/PoseArray.h>
#include "common.h"
#include <vector>

class PlannerDecision {
public:
    enum{ERROR, OK} _status;
    enum{NONE,MAXQ_OP,REACTIVE} _planner;
    uint32_t _tasks_seq;
    uint32_t _plan_seq;
    Eigen::VectorXd _quad_status;
    std::vector<IARCRobot> _tgt_status;
    std::vector<IARCRobot> _obs_status;
public:
    PlannerDecision();
    virtual ~PlannerDecision();
    int init();
    int plan(iarc_arena_simulator::IARCTasksList &taskslist);
    int update_quad(const iarc_arena_simulator::IARCQuadStatus::ConstPtr &quad_status);
    int update_obs(const geometry_msgs::PoseArray::ConstPtr& obstacles);
    int update_tgt(const geometry_msgs::PoseArray::ConstPtr& targets);
    int update_latest_cmdtime(const iarc_arena_simulator::IARCCommand::ConstPtr& cmd);
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
    uint32_t _latest_cmdtime;
    uint32_t _latest_cmd_continue_time;
    iarc_arena_simulator::IARCCommand _latest_cmd;
    uint32_t _loop_start_time;
private:
    int maxq_op(iarc_arena_simulator::IARCTasksList &taskslist);
    int generate_list(std::vector<iarc_arena_simulator::IARCTask> &taskslist);
    int add_one_task(iarc_arena_simulator::IARCTask task, std::vector<iarc_arena_simulator::IARCTask> &taskslist);
    bool in_arena(const IARCRobot & r);
    bool in_esacaped(const IARCRobot &r);
    bool in_sheephold(const IARCRobot &r);
    bool will_in_sheephold(const IARCRobot & robot);
    int tree_search(int depth,double coef);
    int tree_search_simple(int depth, double coef);
    IARCRobot tree_predictor_zero_input(uint32_t rtime, IARCRobot robot, double final_turn = 0.0);
    double reward_tgt_one(const IARCRobot &robot);
    double reward(uint32_t time_ms,  const std::vector<IARCRobot> &targets, const std::vector<IARCRobot> &obstacles, const IARCRobot &mav_robot, robotcmd_kind cmd);
    double reward_Rm(uint32_t time_ms, const IARCRobot & mav_robot, robotcmd_kind cmd);
    double reward_Ro(uint32_t time_ms, const std::vector<IARCRobot> &obstacles, const IARCRobot &mav_robot);
    double reward_Rt(uint32_t time_ms, const std::vector<IARCRobot> &targets);
    double reward_Rt2(uint32_t time_ms, const IARCRobot &targets);
    double reward2(uint32_t time_ms, const IARCRobot & focus_robot, const std::vector<IARCRobot> &obstacles, robotcmd_kind cmd);
    int legal_task(const iarc_arena_simulator::IARCTask &node1,const iarc_arena_simulator::IARCTask &node2);
    double robot_dis(const IARCRobot &r1, const IARCRobot &r2);
    bool require_generate_list();
    IARCRobot get_robot_by_id(uint32_t robot_id);
    iarc_arena_simulator::IARCTask get_current_task();

};

#endif /* DJI_SDK_DEMO_SRC_PLANNERDECISION_H_ */
