/*
 * PlannerDecision.cpp
 *
 *  Created on: 2017年4月19日
 *      Author: Hualin He
 */

#include "PlannerDecision.h"
#include <glog/logging.h>

using namespace std;
using namespace Eigen;
using namespace iarc_arena_simulator;

PlannerDecision::PlannerDecision() {
    // TODO Auto-generated constructor stub
    _status = ERROR;
    _tasks_seq = 0;
    _planner = NONE;
    _plan_seq = 0;
    _tasks_seq = 0;
    _quad_status.setZero(9);
    _tgt_status.clear();
    _obs_status.clear();
    _latest_cmdtime = 0;
    _latest_cmd.command_kind = robotcmd_kind::turn_none;
    _latest_cmd.robot_id = robot_id::robot_arena;
    _latest_cmd_continue_time = 0;
    _fp_task = NULL;
    _fp_task_env = NULL;
    _list_saved.clear();
}

PlannerDecision::~PlannerDecision() {
    // TODO Auto-generated destructor
    _tgt_status.clear();
    _obs_status.clear();
    if(_fp_task != NULL){
        fclose(_fp_task);
        _fp_task = NULL;
    }
    if(_fp_task_env != NULL){
        fclose(_fp_task_env);
        _fp_task_env = NULL;
    }
    _loop_start_time = 0;
}

int PlannerDecision::init(){
    _status = OK;
    _tasks_seq = 0;
    _plan_seq = 0;
    _planner = MAXQ_OP;
    _quad_status.setZero(9);
    _tgt_status.clear();
    _obs_status.clear();
    _list_saved.clear();
    _latest_cmdtime = 0;
    _latest_cmd.command_kind = robotcmd_kind::turn_none;
    _latest_cmd.robot_id = robot_id::robot_arena;
    _latest_cmd_continue_time = 0;
    _loop_start_time = 0;
    _fp_task = fopen(PARAM::file_name_task.c_str(), "w");
    if( _fp_task == NULL ){
        LOG(ERROR) << "file open error! " << PARAM::file_name_task.c_str();
        return -1;
    }

    _fp_task_env = fopen(PARAM::file_name_task_env.c_str(), "w");
    if( _fp_task == NULL){
        LOG(ERROR) << "file open error!" << PARAM::file_name_task.c_str();
    }
    return 0;
}

int PlannerDecision::update_latest_cmdtime(const iarc_arena_simulator::IARCCommand::ConstPtr& cmd)
{
    if(cmd->command_kind != robotcmd_kind::turn_none){
        _latest_cmdtime = arena_time_now();
        _latest_cmd = *cmd;
        _latest_cmd_continue_time = ((cmd->command_kind==turn_180) ?
                PARAM::time_turn_180_ms: PARAM::time_turn_45_ms);
        LOG(INFO) << "latest cmd="<<_latest_cmd.command_kind<< " time = "<<_latest_cmdtime <<" continue= "<<_latest_cmd_continue_time;
    }
    return 0;
}
int PlannerDecision::update_quad(const iarc_arena_simulator::IARCQuadStatus::ConstPtr &quad_status)
{
    _quad_status[0] = quad_status->x;
    _quad_status[1] = quad_status->y;
    _quad_status[2] = quad_status->z;
    _quad_status[3] = quad_status->vx;
    _quad_status[4] = quad_status->vy;
    _quad_status[5] = quad_status->vz;
    _quad_status[6] = quad_status->ax;
    _quad_status[7] = quad_status->ay;
    _quad_status[8] = quad_status->az;
    return 0;
}

int PlannerDecision::update_obs(const geometry_msgs::PoseArray::ConstPtr& obstacles)
{
    uint32_t time_now = arena_time_now();
    int sz = obstacles->poses.size();
    _obs_status.clear();
    for (int k = 0; k < sz; k++){
        IARCRobot r;
        r.time_ms = time_now;
        r.x = obstacles->poses[k].position.x;
        r.y = obstacles->poses[k].position.y;
        r.theta = quaternion_to_theta_z(obstacles->poses[k].orientation.w,obstacles->poses[k].orientation.z);
        r.velocity = PARAM::velocity_obs;
        _obs_status.push_back(r);
    }
    //quaternion_to_theta_z
    return 0;
}

int PlannerDecision::update_tgt(const geometry_msgs::PoseArray::ConstPtr& targets)
{
    uint32_t time_now = arena_time_now();
    int sz = targets->poses.size();

    _tgt_status.clear();
    for (int k = 0; k < sz; k++){
        IARCRobot r;
        r.time_ms = time_now;
        r.x = targets->poses[k].position.x;
        r.y = targets->poses[k].position.y;
        r.theta = quaternion_to_theta_z(targets->poses[k].orientation.w,targets->poses[k].orientation.z);
        r.velocity = PARAM::velocity_tgt;
        r.id = (enum robot_id) k;
        if( PARAM::target_mask[k] && in_arena(r)){
            _tgt_status.push_back(r);
            //make sure that every robot under consideration is alive..
        }
    }
    return 0;
}

int PlannerDecision::plan(iarc_arena_simulator::IARCTasksList &taskslist)
{
    _plan_seq ++;
    switch(_planner){
    case NONE:{
        LOG(ERROR) << "code can not be found!";
    }break;
    case MAXQ_OP:{
        return maxq_op(taskslist);
    }break;
    case REACTIVE:{
        LOG(ERROR) << "code can not be found!";
    }break;
    }
    return 0;
}

int PlannerDecision::maxq_op(iarc_arena_simulator::IARCTasksList &taskslist)
{
    if ( _fp_task_env != NULL){
        uint32_t time_now = arena_time_now();
        fprintf(_fp_task_env, "%d %d mav(%.2lf %.2lf %.2lf) n_tgt = %d n_obs =%d ", _plan_seq, time_now, _quad_status[0], _quad_status[1], _quad_status[2], _tgt_status.size(), _obs_status.size());
        for(int k = 0; k < _tgt_status.size(); k++){
            fprintf(_fp_task_env, "(%d %.2lf %.2lf %.2lf) ", _tgt_status[k].id, _tgt_status[k].x, _tgt_status[k].y, _tgt_status[k].theta*180.0/ M_PI);
        }
        for(int k = 0; k < _obs_status.size(); k++){
            fprintf(_fp_task_env, "(%.2lf %.2lf %.2lf) ", _obs_status[k].x, _obs_status[k].y, _obs_status[k].theta);
        }
    }
    uint32_t t1 = arena_time_now();
    _loop_start_time = t1;
    if(require_generate_list() ){
    	generate_list(taskslist.list);
    	_list_saved = taskslist.list;
    }
    //if (t1 > _latest_cmdtime + _latest_cmd_continue_time && ((t1 % PARAM::period_turn_ms) < PARAM::period_turn_ms - PARAM::time_turn_180_ms)){
    //    generate_list(taskslist.list);
    //}

    uint32_t t2 = arena_time_now();

    LOG(INFO) << "maxq-op [timecost_ms=]"<< t2 - t1;
    return 0;
}

IARCRobot PlannerDecision::get_robot_by_id(uint32_t robot_id)
{
	return get_robot_from_id(_tgt_status, robot_id);
}
iarc_arena_simulator::IARCTask PlannerDecision::get_current_task()
{
	uint32_t timenow = arena_time_now();
	for( int k = 0; k< _list_saved.size(); k++){
		if( timenow> _list_saved[k].time_start && timenow < _list_saved[k].time_end){
			return _list_saved[k];
		}
	}
	LOG(ERROR) << "current task not found!";
	IARCTask task;
	task.time_end = 0;
	return task;
}

bool PlannerDecision::require_generate_list()
{
	uint32_t timenow = ::arena_time_now();

	// not suitable time for decision
	if ( timenow > _latest_cmdtime  &&  timenow< _latest_cmdtime +_latest_cmd_continue_time + 500) return false;
	if( (timenow) % PARAM::period_turn_ms > PARAM::period_turn_ms - PARAM::time_turn_180_ms - 500
			|| (timenow ) % PARAM::period_turn_ms < 500) return false;

	// not suitable for re generate  same decision.....
	// check if the latest task condition is satisfied, if
	IARCTask task = get_current_task();
	if (task.time_end <= timenow) return true;

	switch(task.task_type){
	case type_hover:{
		return false;
	}break;
	case type_reach:{
		return false;
	}break;
	case type_follow:{
		return false;
	}break;
	case type_interact:{
		//this condition is not suitable, for their are other changes of other robots.
		IARCRobot robot = get_robot_by_id(task.robot_id);
		uint32_t time_action;

		if(robot.id == robot_arena) return true;

		if( task.robot_cmd == turn_180){
			time_action = task.time_end - PARAM::time_turn_180_ms;
		}
		else if( task.robot_cmd == turn_45){
			time_action = task.time_end - PARAM::time_turn_45_ms;
		}
		else{
			LOG(ERROR) << "task type error ! unexpected error!";
			return true;
		}

		IARCRobot r2;
		if (time_action < robot.time_ms){
			r2 = robot;
		}
		else{
			r2 = tree_predictor_zero_input(time_action, robot, 0.0);
		}

		LOG(INFO) << "[require generate] r2=" << r2.x << ","<<r2.y << " pose=" <<task.final_pose.position.x <<","<<task.final_pose.position.y
				<<" bound="<<task.final_bound.position.x <<","<<task.final_bound.position.y;

		if (fabs(r2.x - task.final_pose.position.x) < task.final_bound.position.x &&
				fabs(r2.y - task.final_pose.position.y)< task.final_bound.position.y){
			return false;
		}
		else{
			return true;
		}
	}break;
	}
	//IARCRobot robot = get_robot_by_id(task.robot_id);

	return false;
}
int PlannerDecision::generate_list(std::vector<iarc_arena_simulator::IARCTask> &taskslist)
{
    _tree.clear();
    _tree_predictor.clear();
    _tree_predictor.resize(_tgt_status.size());
    _tree_value = 0.0;
    _tree_best.clear();
    _tree_best_value = -1e7;
    _tree_predictor_value.clear();
    _tree_predictor_value.resize(_tgt_status.size());
    _tree_robot_predictor.clear();
    _tree_robot_predictor.resize(_tgt_status.size());

    LOG(INFO) << "time_now=" << arena_time_now() << " tgt size =" <<_tgt_status.size();

    for(int k = 0; k < _tgt_status.size(); k++)
    {
          _tree_robot_predictor[k] = _tgt_status[k];
          _tree_predictor_value[k] = 0.0;
          LOG(INFO) <<_tgt_status[k].id<<"  "<<_tgt_status[k].time_ms<<" "<< _tgt_status[k].x <<" "<< _tgt_status[k].y<<" "<<_tgt_status[k].theta /M_PI*180.0;
    }

    uint32_t time_now =arena_time_now();
    IARCTask root_task = make_task_type_hover(PARAM::str_arena_frame, ros::Time::now(),
            time_now-1, time_now);
    _tree.push_back(root_task);

    //tree_search(0, 1.0);
    LOG(ERROR)<<"Planning by simple";
    tree_search_simple(0, 1.0);

    taskslist.clear();
    if(_tree_best.size()>1)
    {
          int sz = _tree_best.size();
          for(int k = 1; k < sz; k++)
          {
                add_one_task(_tree_best[k], taskslist);
          }
    }
    else
    {
            for(int k = 0; k < _tgt_status.size(); k++)
            {
                if( !in_arena(_tree_robot_predictor[k]) ) continue;
                else
                {
                    string frame_id = PARAM::str_arena_frame;
                    ros::Time rostime = ros::Time::now();
                    IARCTask task = make_task_type_follow(frame_id, rostime, k,
                            _tree_best[0].time_end,  _tree_best[0].time_end+100);
                    add_one_task(task, taskslist);
                    break;
                }
            }
    }

    if(!taskslist.size()==0)
    {
    	for(int k = 0; k<taskslist.size(); k++){
    		LOG(INFO)<<taskslist[k].task_seq<<" id="<<taskslist[k].robot_id<<" cmd="<<::str_kind_turn[taskslist[k].robot_cmd]
    				<<" type="<<::str_task_type[taskslist[k].task_type]<<" timeend="<<taskslist[k].time_end
					<<" value="<<taskslist[k].task_value;
    	}
        LOG(INFO) << "task list generated. ok";
    }
    else
    {
        LOG(ERROR) << "task list is not generate!";
    }
    return 0;
}

int PlannerDecision::add_one_task(iarc_arena_simulator::IARCTask task, std::vector<iarc_arena_simulator::IARCTask> &taskslist)
{
    task.task_seq=_tasks_seq++;
    task.header.frame_id = PARAM::str_arena_frame;
    task.header.stamp = ros::Time::now();
    //task.final_pose.position.x =0.0
    //task.final_pose.position.y =0.0
    task.final_pose.position.z =1.0;
    task.final_bound.position.x = 1.0; //final bound ??
    task.final_bound.position.y = 1.0; //final bound ??
    task.final_bound.position.z = 2.0; //final bound ??
    //task.final_bound.orientation.w = 0.0; //???
    //task.final_bound.orientation.x = 0.0; //???
    //task.final_bound.orientation.y = 0.0; //???
    //task.final_bound.orientation.z = 0.0; //???

    taskslist.push_back(task);

    if(_fp_task!=NULL)
    {
         fprintf(_fp_task,"%d loop_time:%d now_time:%d seq:%d %d(%s) %d %d %d content: ",_plan_seq, _loop_start_time, arena_time_now(), task.task_seq, task.task_type, str_task_type[task.task_type], task.time_start, task.time_end-task.time_start, task.time_end);
         switch(task.task_type)
         {
            case type_hover:
            {
                  fprintf(_fp_task,"hover");
            }break;
            case type_follow:
            {
                fprintf(_fp_task,"follow_id:%d",task.robot_id);
             }break;
             case type_interact:
             {
                 fprintf(_fp_task, "interact_id:%d cmd_kind:%d(%s) pos(%.2lf %.2lf ) timeend=%d", task.robot_id, task.robot_cmd, ::str_kind_turn[task.robot_cmd], task.final_pose.position.x, task.final_pose.position.y, task.time_end);
              }break;
              case type_reach:
              {
                  fprintf(_fp_task,"reach_pos:(%.2lf %.2lf)",task.final_pose.position.x, task.final_pose.position.y);
               }break;
                default:
                {
                      fprintf(_fp_task,"unkonw task type. error");
                }
                }
                fprintf(_fp_task,"\n");
                fflush(_fp_task);
          }
    return 0;
}


bool PlannerDecision::in_esacaped(const IARCRobot &r)
{
    return !(in_arena(r) || in_sheephold(r));
}
bool PlannerDecision::in_sheephold(const IARCRobot &r)
{
    return r.y< - PARAM::arena_size/2.0 &&
            r.x>-PARAM::arena_size/2.0 && r.x < PARAM::arena_size/2.0;
}
bool PlannerDecision::in_arena(const IARCRobot & r)
{
    return fabs(r.x)<PARAM::arena_size/2.0 && fabs(r.y)<PARAM::arena_size/2.0;
}
bool PlannerDecision::will_in_sheephold(const IARCRobot & robot)
{
    double x0,y0;
    double x1,y1;
    double x2,y2;
    x0=robot.x; y0=robot.y;
    IARCRobot r=robot;
    uint32_t rtime=(r.time_ms/PARAM::period_turn_ms+1)*PARAM::period_turn_ms;
    r = tree_predictor_zero_input(rtime, r , 0.0);
    x1=r.x; y1=r.y;
    rtime=(r.time_ms/PARAM::period_turn_ms + 1) * PARAM::period_turn_ms;
    r = tree_predictor_zero_input(rtime,r,0.0);
    x2=r.x; y2=r.y;

    if(y1<-10.0 && fabs(y1-y0)>0.1 && (x0+(x1-x0)/(y1-y0)*(-10.0-y0))>-10.0 && (x0+(x1-x0)/(y1-y0)*(-10.0-y0))<10.0)
    {
          //double x=x0+(x1-x0)/(y1-y0)*(-10.0-y0);
          //第一点与绿边有交点．．直接返回值
          return true;
    }
    if(y2<-10.0 && x1>-10.0 && x1<10.0 && y1>-10.0 && y1<10.0
                && fabs(y2-y1)>0.1 && (x1+(x2-x1)/(y2-y1)*(-10.0-y1))>-10.0 && (x1+(x2-x1)/(y2-y1)*(-10.0-y1))<10.0 )
    {
          //第一点在赛场内，且第二点与绿边有交点．．直接返回值
          return true;
    }
    return false;
}

int PlannerDecision::tree_search_simple(int depth, double coef)
{
	   if(_tree_value >_tree_best_value)
	    {
	          _tree_best_value = _tree_value;
	          _tree_best = _tree;
	    }

	    if(depth>0)
	    {
	          return 0;
	    }

	    int pos = _tree.size()-1;
	    uint32_t loop_start_time=_tree[pos].time_end;

	    for(int k = 0; k < _tgt_status.size(); k++)
	    {
	          if( !in_arena(_tree_robot_predictor[k]) || will_in_sheephold(_tree_robot_predictor[k]) ) continue;

	          for(int t = loop_start_time/1000 + 1; t < loop_start_time/1000 + PARAM::period_turn_ms / 1000; t++)
	          {
	                if(PARAM::shouldExpand[t % (PARAM::period_turn_ms/1000)] == true)
	                {
	                      IARCRobot robot, robot_saved;
	                      IARCTask task;
	                      double res_saved = 0.0 , res = 0.0;

	                      //TURN_180
	                      task.task_type = type_interact;
	                      task.time_start = _tree[_tree.size()-1].time_end;
	                      task.time_end = t*1000+ PARAM::time_turn_180_ms;
	                      task.robot_id = _tgt_status[k].id;
	                      task.robot_cmd = turn_180;
	                      robot = _tree_robot_predictor[k];
	                      robot_saved = robot;
	                      robot = tree_predictor_zero_input(task.time_end - PARAM::time_turn_180_ms, robot, M_PI);
	                      robot.time_ms = task.time_end;

	                      _tree_robot_predictor[k]=robot;
	                    //  res = reward(robot.time_ms,  _tree_robot_predictor, _obs_status, robot, (robotcmd_kind)task.robot_cmd);
	                      res = reward2(robot.time_ms, robot, _obs_status, (robotcmd_kind)task.robot_cmd);
	                      task.task_value = res;
	                      _tree_value = _tree_value + coef * PARAM::tree_search_gama * res;


	                      task.final_pose.position.x = robot.x; //expect final robot position is expect final mav position
	                      task.final_pose.position.y = robot.y; //final - z position is not defined..

	                      if(legal_task(_tree[pos], task) > 0)
	                      {
	                            _tree.push_back(task);
	                            _tree_predictor[k].push_back(task);

	                        //    LOG(INFO) << "expand depth="<<depth<<" id="<<task.robot_id << " cmd=" << task.robot_cmd
	                        //            << " reward="<<res<< " exptime="<<task.time_end;

	                            tree_search_simple(depth+1, coef*PARAM::tree_search_gama);

	                            _tree.pop_back();
	                            _tree_predictor[k].pop_back();
	                      }
	                      _tree_robot_predictor[k] = robot_saved;
	                      _tree_value = _tree_value - coef * PARAM::tree_search_gama * res;

	                      //TURN_45
	                      task.task_type = type_interact;
	                      task.time_start = _tree[_tree.size()-1].time_end;
	                      task.time_end = t*1000+ PARAM::time_turn_45_ms;
	                      task.robot_id =  _tgt_status[k].id;
	                      task.robot_cmd = turn_45;
	                      robot = _tree_robot_predictor[k];
	                      robot_saved = robot;
	                      robot = tree_predictor_zero_input(task.time_end - PARAM::time_turn_45_ms, robot, M_PI/4);
	                      robot.time_ms = task.time_end;

	                      _tree_robot_predictor[k]=robot;
	//                      res = reward(robot.time_ms,  _tree_robot_predictor, _obs_status, robot, (robotcmd_kind)task.robot_cmd);
	                      res = reward2(robot.time_ms, robot, _obs_status,(robotcmd_kind)task.robot_cmd);
	                      task.task_value = res;
	                      _tree_value = _tree_value + coef * PARAM::tree_search_gama * res;

	                      task.final_pose.position.x = robot.x;
	                      task.final_pose.position.y = robot.y;

	                      if(legal_task(_tree[pos], task)>0)
	                      {
	                            _tree.push_back(task);
	                            _tree_predictor[k].push_back(task);

	                     //       LOG(INFO) << "expand depth="<<depth<<" id="<<task.robot_id << " cmd=" << task.robot_cmd
	                       //                                         << " reward="<<res<< " exptime="<<task.time_end;

	                            tree_search_simple(depth+1,coef * PARAM::tree_search_gama);

	                            _tree.pop_back();
	                            _tree_predictor[k].pop_back();
	                      }
	                      _tree_robot_predictor[k]=robot_saved;
	                      _tree_value = _tree_value - coef * PARAM::tree_search_gama * res;
	                }
	          }
	    }

	return 0;
}
int PlannerDecision::tree_search(int depth,double coef)
{
    if(_tree_value >_tree_best_value)
    {
          _tree_best_value = _tree_value;
          _tree_best = _tree;
    }

    if(depth>PARAM::tree_search_max_depth)
    {
          return 0;
    }

    int pos = _tree.size()-1;
    uint32_t loop_start_time=_tree[pos].time_end;

    for(int k = 0; k < _tgt_status.size(); k++)
    {
          if( !in_arena(_tree_robot_predictor[k]) || will_in_sheephold(_tree_robot_predictor[k]) ) continue;

          for(int t = loop_start_time/1000 + 1; t < loop_start_time/1000 + PARAM::period_turn_ms / 1000; t++)
          {
                if(PARAM::shouldExpand[t % (PARAM::period_turn_ms/1000)] == true)
                {
                      IARCRobot robot, robot_saved;
                      IARCTask task;
                      double res_saved = 0.0 , res = 0.0;

                      //TURN_180
                      task.task_type = type_interact;
                      task.time_start = _tree[_tree.size()-1].time_end;
                      task.time_end = t*1000+ PARAM::time_turn_180_ms;
                      task.robot_id = _tgt_status[k].id;
                      task.robot_cmd = turn_180;
                      robot = _tree_robot_predictor[k];
                      robot_saved = robot;
                      robot = tree_predictor_zero_input(task.time_end - PARAM::time_turn_180_ms, robot, M_PI);
                      robot.time_ms = task.time_end;

                      _tree_robot_predictor[k]=robot;
                    //  res = reward(robot.time_ms,  _tree_robot_predictor, _obs_status, robot, (robotcmd_kind)task.robot_cmd);
                      res = reward2(robot.time_ms, robot, _obs_status, (robotcmd_kind)task.robot_cmd);
                      task.task_value = res;
                      _tree_value = _tree_value + coef * PARAM::tree_search_gama * res;


                      task.final_pose.position.x = robot.x; //expect final robot position is expect final mav position
                      task.final_pose.position.y = robot.y; //final - z position is not defined..

                      if(legal_task(_tree[pos], task) > 0)
                      {
                            _tree.push_back(task);
                            _tree_predictor[k].push_back(task);

                        //    LOG(INFO) << "expand depth="<<depth<<" id="<<task.robot_id << " cmd=" << task.robot_cmd
                        //            << " reward="<<res<< " exptime="<<task.time_end;

                            tree_search(depth+1, coef*PARAM::tree_search_gama);

                            _tree.pop_back();
                            _tree_predictor[k].pop_back();
                      }
                      _tree_robot_predictor[k] = robot_saved;
                      _tree_value = _tree_value - coef * PARAM::tree_search_gama * res;

                      //TURN_45
                      task.task_type = type_interact;
                      task.time_start = _tree[_tree.size()-1].time_end;
                      task.time_end = t*1000+ PARAM::time_turn_45_ms;
                      task.robot_id =  _tgt_status[k].id;
                      task.robot_cmd = turn_45;
                      robot = _tree_robot_predictor[k];
                      robot_saved = robot;
                      robot = tree_predictor_zero_input(task.time_end - PARAM::time_turn_45_ms, robot, M_PI/4);
                      robot.time_ms = task.time_end;

                      _tree_robot_predictor[k]=robot;
//                      res = reward(robot.time_ms,  _tree_robot_predictor, _obs_status, robot, (robotcmd_kind)task.robot_cmd);
                      res = reward2(robot.time_ms, robot, _obs_status,(robotcmd_kind)task.robot_cmd);
                      task.task_value = res;
                      _tree_value = _tree_value + coef * PARAM::tree_search_gama * res;

                      task.final_pose.position.x = robot.x;
                      task.final_pose.position.y = robot.y;

                      if(legal_task(_tree[pos], task)>0)
                      {
                            _tree.push_back(task);
                            _tree_predictor[k].push_back(task);

                     //       LOG(INFO) << "expand depth="<<depth<<" id="<<task.robot_id << " cmd=" << task.robot_cmd
                       //                                         << " reward="<<res<< " exptime="<<task.time_end;

                            tree_search(depth+1,coef * PARAM::tree_search_gama);

                            _tree.pop_back();
                            _tree_predictor[k].pop_back();
                      }
                      _tree_robot_predictor[k]=robot_saved;
                      _tree_value = _tree_value - coef * PARAM::tree_search_gama * res;
                }
          }
    }
    return 0;
}

IARCRobot PlannerDecision::tree_predictor_zero_input(uint32_t rtime, IARCRobot robot, double final_turn)
{
	return make_prediction_robot(rtime, robot, final_turn);
}
double PlannerDecision::reward_tgt_one(const IARCRobot &robot)
{
      double x0,y0;
      double x1,y1;
      double x2,y2;
      x0=robot.x; y0=robot.y;
      IARCRobot r=robot;
      uint32_t rtime=(r.time_ms/PARAM::period_turn_ms+1)*PARAM::period_turn_ms;
      r = tree_predictor_zero_input(rtime, r , 0.0);
      x1=r.x; y1=r.y;
      rtime=(r.time_ms/PARAM::period_turn_ms + 1) * PARAM::period_turn_ms;
      r = tree_predictor_zero_input(rtime,r,0.0);
      x2=r.x; y2=r.y;

      if(y1<-10.0 && fabs(y1-y0)>0.1 && (x0+(x1-x0)/(y1-y0)*(-10.0-y0))>-10.0 && (x0+(x1-x0)/(y1-y0)*(-10.0-y0))<10.0)
      {
            //double x=x0+(x1-x0)/(y1-y0)*(-10.0-y0);
            //第一点与绿边有交点．．直接返回值
            return 1.0+(10.0-y1)/20.0;
      }
      if(y2<-10.0 && x1>-10.0 && x1<10.0 && y1>-10.0 && y1<10.0
                  && fabs(y2-y1)>0.1 && (x1+(x2-x1)/(y2-y1)*(-10.0-y1))>-10.0 && (x1+(x2-x1)/(y2-y1)*(-10.0-y1))<10.0 )
      {
            //第一点在赛场内，且第二点与绿边有交点．．直接返回值
            return 1.0+(10.0-y2)/20.0;
      }

      if(fabs(x0)<10.0 && fabs(y0)<10.0 && fabs(x1)<10.0 && fabs(y1)<10.0 && fabs(x2)<10.0 && fabs(y2)<10.0)
      {
            //平均距离(0,-10)的值作为衡量大小，最大为５００，最小为０
            //double l1=sqrt(SQR(x0)+SQR(y0+10));
            //double l2=sqrt(SQR(x1)+SQR(y1+10));
            //double l3=sqrt(SQR(x2)+SQR(y2+10));
            //double l0=sqrt(20*20+10*10);
            //double theta=atan2(y2-y1,x2-x1);
            //return ((500.0*(l0-(l1+l2+l3)/3.0)/l0)*PARAM::tree_search_gama_distance+(M_PI/2.0-fabs(fabs(theta)-M_PI/2.0))/M_PI*100.0*PARAM::tree_search_gama_theta)*PARAM::tree_search_gama_inner;//改变成为

            double disy = min(y0, min(y1, y2)) + PARAM::arena_size/2.0;
            double neary = 0.0;
            if (disy < 0.0 ) disy = 0.0;
            if (disy <= PARAM::radius_neary ){
                  neary = 1.0 -disy/PARAM::radius_neary;
            }
            else{
                   neary = 0.0;
            }

            double form_theta = fmod(robot.theta, 2.0 * M_PI);
            double neartheta = 0.0;
            if ( form_theta < - M_PI) form_theta = form_theta + 2.0 * M_PI;
            else if(form_theta > M_PI ) form_theta = form_theta - 2.0 * M_PI;
            neartheta = fabs(form_theta + M_PI / 2.0);
            if (neartheta > M_PI/2.0) neartheta = M_PI/2.0;
            neartheta = 1.0 - neartheta/M_PI*2.0;
            return PARAM::ramda_t_A*(PARAM::ramda_t_y * neary +
                          PARAM::ramda_t_theta * neartheta);
      }
      else
      {//出赛场，给出赛场一个评估函数
            double l1=sqrt(SQR(x0)+SQR(y0));
            double l2=sqrt(SQR(x1)+SQR(y1));
            double l3=sqrt(SQR(x2)+SQR(y2));
            double l0=10.0;
            double lmax=MAX(l1,MAX(l2,l3));
            //return (-1000.0*lmax/l0);
            return -1.0 * lmax/l0;
      }
      return 0.0;
}
double PlannerDecision::robot_dis(const IARCRobot &r1, const IARCRobot &r2)
{
    return sqrt((r1.x - r2.x) * (r1.x - r2.x)+(r1.y - r2.y)*(r1.y - r2.y));
}
double PlannerDecision::reward_Rm(uint32_t time_ms, const IARCRobot & mav_robot, robotcmd_kind cmd)
{
	//LOG(INFO) << "Rm=" <<0.0;
    return -1.0;
}
double PlannerDecision::reward_Ro(uint32_t time_ms, const std::vector<IARCRobot> &obstacles, const IARCRobot &mav_robot)
{
    double reward = 0.0;
    double dis_min = PARAM::arena_size;
    //scaled to 0~1.
    for(int k = 0; k < obstacles.size(); k++){
        IARCRobot robot = obstacles[k];
        //tree_predictor_zero_input(time_ms,robot, 0.0);
        if(time_ms - robot.time_ms >  PARAM::obstacle_time_forget_ms ) return 0.0;
        else{
        	robot.x = robot.x + robot.velocity * (double)(time_ms - robot.time_ms)/1000.0 *cos(robot.theta);
        	robot.y = robot.y + robot.velocity * (double)(time_ms - robot.time_ms)/1000.0 *sin(robot.theta);
        	robot.time_ms = time_ms;
        }
        double dis = robot_dis(robot, mav_robot);
        if (dis_min > dis){
            dis_min = dis;
        }
    }
    double danger = 0.0;
    if (dis_min < PARAM::radius_collision) danger = 1.0;
    else if (dis_min > PARAM::radius_safe) danger = 0.0;
    else{
        danger = 1.0 - (dis_min - PARAM::radius_collision) /
                (PARAM::radius_safe-PARAM::radius_collision);
    }
    reward = - danger * PARAM::ramda_o_a;
 //   LOG(INFO) << "Ro=" <<reward;
    return reward;
}
double PlannerDecision::reward_Rt2(uint32_t time_ms, const IARCRobot &targets)
{
	double reward = 0.0;
	reward = reward_tgt_one(targets);
	return reward;
}
double PlannerDecision::reward_Rt(uint32_t time_ms, const std::vector<IARCRobot> &targets)
{
    double reward = 0.0;
    for(int k = 0; k< targets.size(); k++){
        IARCRobot robot = targets[k];
        reward = reward + reward_tgt_one(robot);
        continue;

        tree_predictor_zero_input(time_ms, robot, 0.0);
        //reward = reward + reward_tgt_one(robot);
        if (in_sheephold(robot)){
            reward = reward + PARAM::ramda_t_S;
        }
        else if (!in_arena(robot)){
            reward = reward + PARAM::ramda_t_E;
        }
        else{
            double disy = (robot.y + PARAM::arena_size/2.0);
            double neary = 0.0;
            if (disy <= PARAM::radius_neary ){
                neary = 1.0 -disy/PARAM::radius_neary;
            }
            else{
                neary = 0.0;
            }

            double form_theta = fmod(robot.theta, 2.0 * M_PI);
            double neartheta = 0.0;
            if ( form_theta < - M_PI) form_theta = form_theta + 2.0 * M_PI;
            else if(form_theta > M_PI ) form_theta = form_theta - 2.0 * M_PI;
            neartheta = fabs(form_theta + M_PI / 2.0);
            if (neartheta > M_PI/2.0) neartheta = M_PI/2.0;
            neartheta = 1.0 - neartheta/M_PI*2.0;
            reward = reward + PARAM::ramda_t_A*(PARAM::ramda_t_y * neary +
                    PARAM::ramda_t_theta * neartheta);
        }
    }
   // LOG(INFO)<<"Rt="<<reward;
    return reward;
}
double PlannerDecision::reward2(uint32_t time_ms,
		const IARCRobot & focus_robot,
		const std::vector<IARCRobot> &obstacles, robotcmd_kind cmd)
{
	double reward;
	reward = PARAM::ramda_t * reward_Rt2(time_ms, focus_robot)
	+ PARAM::ramda_o * reward_Ro(time_ms, obstacles, focus_robot)
	+ PARAM::ramda_m * reward_Rm(time_ms, focus_robot, cmd);

	return reward;
}

double PlannerDecision::reward(uint32_t time_ms,
        const std::vector<IARCRobot> &targets, const std::vector<IARCRobot> &obstacles,
        const IARCRobot &mav_robot, robotcmd_kind cmd)
{
	double reward;
	LOG(INFO) << "targets[0]=("<< targets[0].x<<", "<< targets[0].y<<", "<<targets[0].theta/M_PI*180.0<<"deg)";
	reward = PARAM::ramda_t * reward_Rt(time_ms, targets)
    + PARAM::ramda_o * reward_Ro(time_ms, obstacles, mav_robot)
    + PARAM::ramda_m * reward_Rm(time_ms, mav_robot, cmd);
	LOG(INFO) <<"r=" <<reward;
    return reward;
}

int PlannerDecision::legal_task(const IARCTask &node1,const IARCTask &node2)
{
      if(fabs(node1.final_pose.position.x)>10.0 || fabs(node1.final_pose.position.y)>10.0
                  || fabs(node2.final_pose.position.x)>10.0 || fabs(node2.final_pose.position.y)>10.0) return 0;

      if(node1.task_type==type_interact && node2.task_type==type_interact)
      {
            if(((double)(node2.time_end-node1.time_end))*PARAM::velocity_mav_max
                        >::sqrt(SQR(node2.final_pose.position.y-node1.final_pose.position.y)+SQR(node2.final_pose.position.x-node1.final_pose.position.x)))
            {
                  return 1;
            }
            else
            {
                  return 0;
            }
      }
      if(node1.task_type==type_hover && node2.task_type==type_interact)
      {
            if(((double)(node2.time_end-node1.time_end))*PARAM::velocity_mav_max
                        >::sqrt(SQR(node2.final_pose.position.y-(double)_quad_status[1])+SQR(node2.final_pose.position.x-(double)_quad_status[0])))
            {
                  return 1;
            }
            else
            {
                  return 0;
            }
      }

      LOG(ERROR) <<"error! unexpected tasks in tree!";

      return 0;
}
