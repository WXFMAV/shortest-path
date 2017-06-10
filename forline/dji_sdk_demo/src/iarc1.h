#ifndef __IARC_H__
#define __IARC_H__

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include"Eigen/Dense"
#include<vector>
#include<iostream>
using namespace std;
using namespace Eigen;


#define N_OBS 4
#define N_TRG 3
static bool shouldExpand[20]={0,0,0,0,1,
                                                        0,1,0,0,1,
                                                        0,1,0,0,1,
                                                        0,0,0,0,0};
#define TREE_SEARCH_MAX_DEPTH 2
#define TREE_SEARCH_GAMA 0.5
#define TREE_SEARCH_GAMA_THETA 0.2
#define TREE_SEARCH_GAMA_DISTANCE 0.5
#define TREE_SEARCH_GAMA_INNER 1.0

#define TIME_TURN_180 2600
#define TIME_TURN_45 650
#define PERIOD_TURN 20000

#define KIND_TURN_NONE 0
#define KIND_TURN_180 1
#define KIND_TURN_45 2

#define DEFAULT_PARAM_AX 3.6
#define DEFAULT_PARAM_KX 3.0
#define DEFAULT_PARAM_VMX 2.0
#define DEFAULT_PARAM_AY 3.6
#define DEFAULT_PARAM_KY 3.0
#define DEFAULT_PARAM_VMY 2.0
#define DEFAULT_PARAM_VM 2.0

#define SQR(a) ((a)*(a))
#define MAX(a,b) ((a)>(b)?(a):(b))

typedef struct IARC_POSITION{
      double x,y,z;
}IARC_POSITION;
typedef struct IARC_Quaternion{
      double x,y,z,w;
} IARC_Quaternion;

typedef struct IARC_MAV{
      uint32_t time_ms;
      double x,y,z;
      double vx,vy,vz;
      IARC_Quaternion orientation;
      double param_ax,param_ay;
      double param_kx,param_ky;
      double param_vmx,param_vmy,param_vm;
}IARC_MAV;

typedef struct IARC_ROBOT{
      uint32_t time_ms;
      double x;
	double y;
	double velocity;
	IARC_Quaternion orientation;
}IARC_ROBOT;

typedef struct IARC_COMMAND{
      double mav_vx;
      double mav_vy;
      double mav_vz;
      double mav_vyawdegree;
      uint8_t  robot_turn_kind;
      uint8_t robot_turn_id;
}IARC_COMMAND;


static const char* str_task_type[4]={"hover ","reach ","follow","in.act"};

enum task_type{
      type_hover,
      type_reach,
      type_follow,
      type_interact
};
enum robotcmd_kind{
      turn_none=KIND_TURN_NONE,
      turn_180=KIND_TURN_180,
      turn_45=KIND_TURN_45
};
static const char* str_task_result[4]={"error ","going..","reached","failed!"};

enum task_result{
      task_error=0,
      task_going=1,
      task_reached=2,
      task_failed=3
};

typedef struct IARC_TASK{
      uint32_t seq;//task_unique ID sequence
      enum task_type type;//task_type
      uint32_t start_time;
      uint32_t  cost_time;
      double value;
      union{
            struct{
                  double reach_x;
                  double reach_y;
                  double reach_tor_x;
                  double reach_tor_y;
                  uint32_t reach_tor_time;
            };
            struct{
                  uint8_t follow_id;
                  double follow_tor_x;
                  double follow_tor_y;
            };
            struct{
                  uint8_t interact_id;
                  enum robotcmd_kind interact_cmdkind;
                  bool interact_command_sent;
                  double interact_exp_x;
                  double interact_exp_y;
                  double interact_exp_azimuth;
                  double interact_tor_x;
                  double interact_tor_y;
                  double interact_tor_azimuth;
            };
      } content;
}IARC_TASK;

typedef struct IARC_WAYPOINT{
      uint32_t    t;
      double x;
      double y;
}IARC_WAYPOINT;

class CIARC{
public:
      IARC_MAV mav;
	IARC_ROBOT targets[N_TRG];
	IARC_ROBOT obstacles[N_OBS];
	IARC_COMMAND command;
	uint32_t seq_targets;
	uint32_t seq_obstacles;
	FILE * fp,*fp_task,*fp_tasklist,*fp_pathlist,*fp_controller;
	vector<IARC_POSITION> pre_path;
	vector<IARC_TASK> task_list;
	int32_t task_pos,task_seq,task_seq_saved,task_total;
private:
	uint32_t seq_loop;
	uint32_t seq_targets_last;
	uint32_t seq_obstacles_last;
	//vector<IARC_TASK> task_list;
	//int32_t task_pos,task_seq,task_seq_saved,task_total;
	vector<IARC_WAYPOINT> path_list;
	int32_t wp_pos,wp_pos_saved;

	vector<IARC_TASK> tree;
	vector<vector<IARC_TASK>> tree_predictor;
	vector<IARC_TASK> tree_best;
	vector<IARC_ROBOT> tree_robot_predictor;
	double tree_best_value;
	double tree_value;
	vector<double> tree_predictor_value;
public:
	CIARC();
	~CIARC();
	int MainLoop(IARC_COMMAND &cmd);
	int Simulator(double dT);
	int SimulatorTask();
	bool inline InArena(const IARC_ROBOT &robot);
private:
	enum task_result TaskReached();
	uint32_t GetNowTime();
	int TaskPlan();
      int PathPlan();
      int MotionPlan();
      int AddOneTask(IARC_TASK task);
      double OptimalController(double a0,double k0,double tgo,double x1,double x2,double xt1,double xt2,double vm);
      bool TaskStatusTolerated();
      int TaskListGenerate();
      double Quaternion2ThetaZ(IARC_Quaternion q);
      IARC_Quaternion ThetaZ2Quaternion(double theta);
      int TreeSearch(int depth,double coef);
      int TreePredictor(int id,uint32_t time,IARC_ROBOT   &robot);
      int TreePredictorZeroInput(uint32_t time,IARC_ROBOT &robot,double final_turn);
      int TreeConnector(const IARC_TASK &node1,const IARC_TASK &node2);
      double TreePredictorValue(int id);
      double TreeRobotValue(const IARC_ROBOT &robot);
};

#endif
