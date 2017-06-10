#include "iarc5.h"
#include "Eigen/Dense"

using namespace Eigen;

class CIARC theIARC;

CIARC::CIARC()
{
      fp=fopen("CIARC.txt","w");
      if(fp==NULL){
            printf("file CIARC.txt open failed!\n");
      }
      fp_task=fopen("CIARC_Task.txt","w");
      if(fp_task==NULL)
      {
            printf("file CIARC_Task.txt open failed!\n");
      }
      fp_tasklist=fopen("CIARC_Tasklist.txt","w");
      if(fp_tasklist==NULL)
      {
            printf("file CIARC_Tasklist.txt open failed!\n");
      }
      fp_pathlist=fopen("CIARC_Pathlist.txt","w");
      if(fp_pathlist==NULL)
      {
            printf("file CIARC_Pathlist.txt open failed!\n");
      }
      fp_controller=fopen("CIARC_Controller.txt","w");
      if(fp_controller==NULL)
      {
            printf("file CIARC_Controller.txt open failed!\n");
      }
      seq_obstacles=0;
      seq_targets=0;
      seq_obstacles_last=0;
      seq_targets_last=0;
      seq_loop=0;

      task_list.clear();
      task_total=0;
      task_pos=0;

      IARC_TASK task;
      task.seq=0;
      task.type=type_hover;
      task.start_time=0;
      task.cost_time=1000;//hovering for 1000ms every first rtime!

      AddOneTask(task);

      task_seq=task_list[task_pos].seq;
      task_seq_saved=task_seq;

      wp_pos=0;
      wp_pos_saved=wp_pos;
      path_list.clear();

   //   SimulatorTask();
}

CIARC::~CIARC()
{
      if(fp!=NULL)
      {
            fclose(fp);
      }
      if(fp_task!=NULL)
      {
            fclose(fp_task);
      }
      if(fp_tasklist!=NULL)
      {
            fclose(fp_tasklist);
      }
      if(fp_controller!=NULL)
      {
            fclose(fp_controller);
      }
}

int CIARC::SimulatorTask()
{
      task_list.clear();
      IARC_TASK task;
      task.type=type_hover;
      task.start_time=0;
      task.cost_time=2000;//hovering for 1000ms every first rtime!
      AddOneTask(task);

      //hove for one second  then reached (5,0, 5000), then reached(0,5,10000),then follow(2,50000)

      task.type=type_reach;
      task.start_time=task.start_time+task.cost_time;
      task.cost_time=2000;
      task.content.reach_x=2.0;
      task.content.reach_y=0.0;
      task.content.reach_tor_x=0.2;
      task.content.reach_tor_y=0.2;
      task.content.reach_tor_time=200;
      AddOneTask(task);

      task.type=type_reach;
      task.start_time=task.start_time+task.cost_time;
      task.cost_time=2000;
      task.content.reach_x=2.0;
      task.content.reach_y=2.0;
      task.content.reach_tor_x=0.2;
      task.content.reach_tor_y=0.2;
      task.content.reach_tor_time=200;
      AddOneTask(task);

      task.type=type_follow;
      task.start_time=task.start_time+task.cost_time;
      task.cost_time=10000;
      task.content.follow_id=2;
      task.content.follow_tor_x=1.0;
      task.content.follow_tor_y=1.0;
      AddOneTask(task);

      task.type=type_interact;
      task.start_time=task.start_time+task.cost_time;
      task.cost_time=50;
      task.content.interact_id=2;
      task.content.interact_cmdkind=turn_45;
      task.content.interact_exp_x=0.0;
      task.content.interact_exp_y=0.0;
      task.content.interact_exp_azimuth=0.0;
      task.content.interact_tor_x=200.0;
      task.content.interact_tor_y=200.0;
      task.content.interact_tor_azimuth=20.0*M_PI;
      task.content.interact_command_sent=false;

      AddOneTask(task);

      task.type=type_follow;
      task.start_time=task.start_time+task.cost_time;
      task.cost_time=10000;
      task.content.follow_id=3;
      task.content.follow_tor_x=1.0;
      task.content.follow_tor_y=1.0;
      AddOneTask(task);

      task.type=type_interact;
      task.start_time=task.start_time+task.cost_time;
      task.cost_time=50;
      task.content.interact_id=3;
      task.content.interact_cmdkind=turn_45;
      task.content.interact_exp_x=0.0;
      task.content.interact_exp_y=0.0;
      task.content.interact_exp_azimuth=0.0;
      task.content.interact_tor_x=200.0;
      task.content.interact_tor_y=200.0;
      task.content.interact_tor_azimuth=20.0*M_PI;
      task.content.interact_command_sent=false;
      AddOneTask(task);

      task.type=type_follow;
      task.start_time=task.start_time+task.cost_time;
      task.cost_time=10000;
      task.content.follow_id=4;
      task.content.follow_tor_x=1.0;
      task.content.follow_tor_y=1.0;
      AddOneTask(task);

      task.type=type_interact;
      task.start_time=task.start_time+task.cost_time;
      task.cost_time=50;
      task.content.interact_id=4;
      task.content.interact_cmdkind=turn_45;
      task.content.interact_exp_x=0.0;
      task.content.interact_exp_y=0.0;
      task.content.interact_exp_azimuth=0.0;
      task.content.interact_tor_x=200.0;
      task.content.interact_tor_y=200.0;
      task.content.interact_tor_azimuth=20.0*M_PI;
      task.content.interact_command_sent=false;
      AddOneTask(task);

      path_list.clear();
      wp_pos=0;
      wp_pos_saved=0;

      return 0;
}
/*
 *function [vx]=Fopt_fdbk(a1,K,x10,x20,x1T,x2T,T)
u0=0;
A=[2 -1 4*a1^3 -4*a1^2
    -2 -1 0 4*a1^2
    2-2*a1*T -exp(a1*T) 4*a1^3 -4*a1^2*exp(-a1*T)
    -2 -exp(a1*T) 0 4*a1^2*exp(-a1*T)];
B=[4*a1^3*x10+4*a1*u0
    4*a1^2*x20-4*a1*u0
    4*a1^3*x1T+4*a1*u0-4*a1^2*u0*T
    4*a1^2*x2T-4*a1*u0];
Ainv=inv(A);
C=Ainv*B;
c1=C(1); c2=C(2); c3=C(3); c4=C(4);
ux=u0-c1/(2*a1)-c2/(2*a1);
vx=ux/K;

um=2.0;
if vx>um
    vx=um;
elseif vx<-um
    vx=-um;
end

if isnan(vx)
    vx=0;
end
 */
double CIARC::OptimalController(double a0,double k0,double tgo,double x1,double x2,double xt1,double xt2,double vm)
{
#define sqr3(a) ((a)*(a)*(a))
#define sqr2(a) ((a)*(a))
#define e(a) exp((a))
      //printf("OptimalController: in\n");
      double ans;
      MatrixXd A(4,4);
      MatrixXd B(4,1);
      MatrixXd C(1,4);
      MatrixXd res(1,1);
      A(0,0)=2.0; A(0,1)=-1.0; A(0,2)=4*sqr3(a0); A(0,3)=-4*sqr2(a0);
      A(1,0)=-2.0; A(1,1)=-1.0; A(1,2)=0.0; A(1,3)=4*sqr2(a0);
      A(2,0)=2.0-2*a0*tgo; A(2,1)=-e(a0*tgo); A(2,2)=4*sqr3(a0); A(2,3)=-4*sqr2(a0)*e(-a0*tgo);
      A(3,0)=-2.0; A(3,1)=-e(a0*tgo); A(3,2)=0.0; A(3,3)=4*sqr2(a0)*e(-a0*tgo);
      B(0,0)=4*sqr3(a0)*x1;
      B(1,0)=4*sqr2(a0)*x2;
      B(2,0)=4*sqr3(a0)*xt1;
      B(3,0)=4*sqr2(a0)*xt2;
      C(0,0)=-1.0/2.0/a0; C(0,1)=-1.0/2.0/a0; C(0,2)=0.0; C(0,3)=0.0;

     res=1/k0*C*A.inverse()*B;

     ans=res(0,0);
      ans=::isnan(ans)?0.0:ans>vm?vm:ans<-vm?-vm:ans;
      //printf("OptimalController: leave\n");

      return ans;

#undef sqr3
#undef sqr2
#undef e
}
uint32_t CIARC::GetNowTime()
{
      return mav.time_ms;
}

int CIARC::AddOneTask(IARC_TASK task)
{
      task.seq=task_total++;
      task_list.push_back(task);

      if(fp_task!=NULL)
      {
            fprintf(fp_task,"%d seq:%d %d(%s) %d %d %d content: ",GetNowTime(),task.seq,task.type,str_task_type[task.type],task.start_time,task.cost_time,task.start_time+task.cost_time);
            switch(task.type)
            {
            case type_hover:
            {
                  fprintf(fp_task,"hover");
            }break;
            case type_follow:
                        {
                              fprintf(fp_task,"follow_id:%d tor:%.2lf %.2lf",task.content.follow_id,task.content.follow_tor_x,task.content.follow_tor_y);
                        }break;
            case type_interact:
                        {
                              fprintf(fp_task,"interact_id:%d cmd_kind:%d exp(%.2lf %.2lf az:%.2lf(%.2lf))",task.content.interact_id,task.content.interact_cmdkind,
                                          task.content.interact_exp_x,task.content.interact_exp_y,task.content.interact_exp_azimuth,task.content.interact_exp_azimuth*180.0/M_PI);
                        }break;
            case type_reach:
                        {
                              fprintf(fp_task,"reach_pos:(%.2lf %.2lf) tor:(%.2lf %.2lf %d)",task.content.reach_x,task.content.reach_y,task.content.reach_tor_x,task.content.reach_tor_y,task.content.reach_tor_time);
                        }break;
            default:
            {
                  fprintf(fp_task,"unkonw task type. error");
            }
            }
            fprintf(fp_task,"\n");
            fflush(fp_task);
      }
      return 0;
}
enum task_result CIARC::TaskReached()
{
      enum task_result res;
      res=task_failed;
      if(task_pos>=0 && task_pos<task_list.size())
      {
            if(task_list[task_pos].seq!=task_seq)
            {//new task list arrived!  abort current task!
                  task_pos=-1;
                  task_seq=-1;
                  res=task_reached;
                  printf("error! impossible to get in here!\n");
            }
            else
            {//task still in list  .. judge if this task has reached
                  res=task_going;
                  switch(task_list[task_pos].type)
                  {
                  case type_hover:
                        res=GetNowTime()>=task_list[task_pos].cost_time+task_list[task_pos].start_time?task_reached:task_going;
                        break;
                  case type_reach:
                        if(abs(int(GetNowTime())-(int)task_list[task_pos].cost_time-(int)task_list[task_pos].start_time)<task_list[task_pos].content.reach_tor_time
                          && fabs(task_list[task_pos].content.reach_x-mav.x)<=task_list[task_pos].content.reach_tor_x
                          && fabs(task_list[task_pos].content.reach_y-mav.y)<=task_list[task_pos].content.reach_tor_y)
                        {
                              res=task_reached;
                        }
                        else
                        {
                              if(GetNowTime()>task_list[task_pos].cost_time+task_list[task_pos].start_time+task_list[task_pos].content.reach_tor_time)
                              {
                                    res=task_failed;
                              }
                              else
                              {
                                    res=task_going;
                              }
                        }
                        break;
                  case type_follow:
                        if(GetNowTime()>=task_list[task_pos].start_time+task_list[task_pos].cost_time)
                        {
                              if( fabs(mav.x-targets[task_list[task_pos].content.follow_id].x)<task_list[task_pos].content.follow_tor_x
                                          && fabs(mav.y-targets[task_list[task_pos].content.follow_id].y)<task_list[task_pos].content.follow_tor_y)
                              {
                                    res=task_reached;
                              }
                              else
                              {
                                    res=task_failed;
                              }
                        }
                        else
                        {
                              res=task_going;
                        }
                        break;
                  case type_interact:

                        if(GetNowTime()>task_list[task_pos].start_time+task_list[task_pos].cost_time)
                        {
                              res=task_reached;
                        }
                        else
                        {
                              res=task_going;
                        }
                        break;
                  default:
                        printf("unknow task type!\n");
                        res=task_failed;
                  }
            }
      }
      else
      {
            printf("Error!  pos not in task list!\n");
            res=task_failed;
      }

      return res;
}
IARC_Quaternion CIARC::ThetaZ2Quaternion(double theta)
{
      IARC_Quaternion q;
      q.x=0.0;
      q.y=0.0;
      q.z=cos((-theta+M_PI)/2);
      q.w=sin((-theta+M_PI)/2);
      return q;
}
double CIARC::Quaternion2ThetaZ(IARC_Quaternion q)
{
      double eq=0.0;
      eq=atan2(q.w,q.z);
      return M_PI-2*eq;
}

bool CIARC::TaskStatusTolerated()
{
      if(task_list.size()==0) return false;
      if(task_pos<0 || task_pos>=task_list.size()) return false;
      if(task_list[task_pos].seq!=task_seq) return false;
      bool tolerate=true;
      IARC_TASK *p=&task_list[task_pos];
      switch(task_list[task_pos].type)
      {
            case type_hover:
            {
                  if(GetNowTime()>p->start_time+p->cost_time)
                  {
                        tolerate=false;
                  }
                  else
                  {
                        tolerate=true;
                  }
            }break;
            case type_interact:
            {
                  if(fabs(p->content.interact_exp_x-targets[p->content.interact_id].x)<p->content.interact_tor_x
                                  &&fabs(p->content.interact_exp_y-targets[p->content.interact_id].y)<p->content.interact_tor_y
                                  && fabs(p->content.interact_exp_azimuth-Quaternion2ThetaZ(targets[p->content.interact_id].orientation))<p->content.interact_tor_azimuth)
                  {
                        tolerate=true;
                  }
                  else if(p->content.interact_command_sent && GetNowTime()<p->start_time+p->cost_time && GetNowTime()>p->start_time)
                  {
                        tolerate=true;
                  }
                  else
                  {
                  		tolerate=false;
                  }
            }break;
            case type_reach:
            {
                  tolerate=true;
            }break;
            case type_follow:
            {
                  tolerate=true;
            }break;
            default:
                  tolerate=false;
      }

      return tolerate;
}

int CIARC::TreeConnector(const IARC_TASK &node1,const IARC_TASK &node2)
{
      if(fabs(node1.content.interact_exp_x)>10.0 || fabs(node1.content.interact_exp_y)>10.0
                  || fabs(node2.content.interact_exp_x)>10.0 || fabs(node2.content.interact_exp_y)>10.0) return 0;

      if(node1.type==type_interact && node2.type==type_interact)
      {
            if(((double)(node2.start_time-node2.start_time-node2.cost_time))*DEFAULT_PARAM_VM
                        >::sqrt(SQR(node2.content.interact_exp_y-node1.content.interact_exp_y)+SQR(node2.content.interact_exp_x-node1.content.interact_exp_x)))
            {
                  return 1;
            }
            else
            {
                  return 0;
            }
      }
      if(node1.type==type_hover && node2.type==type_interact)
      {
            if(((double)(node2.start_time-node2.start_time-node2.cost_time))*DEFAULT_PARAM_VM
                        >::sqrt(SQR(node2.content.interact_exp_y-mav.y)+SQR(node2.content.interact_exp_x-mav.x)))
            {
                  return 1;
            }
            else
            {
                  return 0;
            }
      }

      printf("error!  unexpected nodes in tree!\n");

      return 0;
}
bool inline CIARC::InArena(const IARC_ROBOT &robot)
{
      return fabs(robot.x)<10.0 && fabs(robot.y)<10.0;
}
int CIARC::TreePredictorZeroInput(uint32_t rtime,IARC_ROBOT &robot,double final_turn=0.0)
{
      if(robot.time_ms<=rtime)
      {
            double theta=Quaternion2ThetaZ(robot.orientation);
            int cntloop_times=0;
            while(1)
            {
                  int32_t newtime=(robot.time_ms/PERIOD_TURN+1)*PERIOD_TURN;
   //               printf("new rtime:%d rtime%d robot_time_ms:%d dec0:%d\n",newtime,rtime,robot.time_ms,newtime-robot.time_ms-TIME_TURN_180);
                  if(newtime<=rtime)
                  {
                        if(newtime-robot.time_ms>TIME_TURN_180)
                        {
                              robot.x=robot.x+robot.velocity*cos(theta)*((double)(newtime-robot.time_ms-TIME_TURN_180)/1000.0);
                              robot.y=robot.y+robot.velocity*sin(theta)*((double)(newtime-robot.time_ms-TIME_TURN_180)/1000.0);
                              robot.time_ms=newtime;
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
//            printf("     theta:%.2lf    %d %.2lf %.2lf time_ms:%d rtime:%d",theta,cntloop_times,robot.x,robot.y,robot.time_ms,rtime);

            if(robot.time_ms<=rtime && rtime%PERIOD_TURN<=PERIOD_TURN-TIME_TURN_180)
                {
                     //   printf("dec:%d\n",(rtime-robot.time_ms));
                      double lftime=((double)(rtime-robot.time_ms))/1000.0;
                      robot.x=robot.x+robot.velocity*cos(theta)*lftime;
                      robot.y=robot.y+robot.velocity*sin(theta)*lftime;
                      robot.time_ms=rtime;
                }
                else if(robot.time_ms<=rtime && robot.time_ms%PERIOD_TURN<PERIOD_TURN-TIME_TURN_180)
                {
                     // printf("dec2:%d\n",(PERIOD_TURN-TIME_TURN_180-robot.time_ms%PERIOD_TURN));
                      double lftime=((double)(PERIOD_TURN-TIME_TURN_180-robot.time_ms%PERIOD_TURN))/1000.0;
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
            robot.orientation=ThetaZ2Quaternion(theta);
      }
      else
      {
            printf("error! can not predict history! %d %d\n",robot.time_ms,rtime);
      }
      return 0;
}

double CIARC::TreeRobotValue(const IARC_ROBOT &robot)
{
      double x0,y0;
      double x1,y1;
      double x2,y2;
      x0=robot.x; y0=robot.y;
      IARC_ROBOT r=robot;
      uint32_t rtime=(r.time_ms/PERIOD_TURN+1)*PERIOD_TURN;
      TreePredictorZeroInput(rtime,r,0.0);
      x1=r.x; y1=r.y;
      rtime=(r.time_ms/PERIOD_TURN+1)*PERIOD_TURN;
      TreePredictorZeroInput(rtime,r,0.0);
      x2=r.x; y2=r.y;

      if(y1<-10.0 && fabs(y1-y0)>0.1 && (x0+(x1-x0)/(y1-y0)*(-10.0-y0))>-10.0 && (x0+(x1-x0)/(y1-y0)*(-10.0-y0))<10.0)
      {
            //double x=x0+(x1-x0)/(y1-y0)*(-10.0-y0);
            //第一点与绿边有交点．．直接返回值
            return 1000.0;
      }
      if(y2<-10.0 && x1>-10.0 && x1<10.0 && y1>-10.0 && y1<10.0
                  && fabs(y2-y1)>0.1 && (x1+(x2-x1)/(y2-y1)*(-10.0-y1))>-10.0 && (x1+(x2-x1)/(y2-y1)*(-10.0-y1))<10.0 )
      {
            //第一点在赛场内，且第二点与绿边有交点．．直接返回值
            return 1000.0;
      }

      if(fabs(x0)<10.0 && fabs(y0)<10.0 && fabs(x1)<10.0 && fabs(y1)<10.0 && fabs(x2)<10.0 && fabs(y2)<10.0)
      {
            //平均距离(0,-10)的值作为衡量大小，最大为５００，最小为０
            double l1=sqrt(SQR(x0)+SQR(y0+10));
            double l2=sqrt(SQR(x1)+SQR(y1+10));
            double l3=sqrt(SQR(x2)+SQR(y2+10));
            double l0=sqrt(20*20+10*10);
            double theta=atan2(y2-y1,x2-x1);
            return -100;//((500.0*(l0-(l1+l2+l3)/3.0)/l0)*TREE_SEARCH_GAMA_DISTANCE+(M_PI/2.0-fabs(fabs(theta)-M_PI/2.0))/M_PI*100.0*TREE_SEARCH_GAMA_THETA)*TREE_SEARCH_GAMA_INNER;//改变成为
            //return 0.0;
      }
      else
      {//出赛场，给出赛场一个评估函数
            double l1=sqrt(SQR(x0)+SQR(y0));
            double l2=sqrt(SQR(x1)+SQR(y1));
            double l3=sqrt(SQR(x2)+SQR(y2));
            double l0=10.0;
            double lmax=MAX(l1,MAX(l2,l3));
            return -1000.0;
      }
      return 0.0;
}
int CIARC::TreeSearch(int depth,double coef)
{
      if(tree_value>tree_best_value)
      {
            tree_best_value=tree_value;
            tree_best=tree;
      }

      if(depth>TREE_SEARCH_MAX_DEPTH)
      {
            return 0;
      }

      int pos=tree.size()-1;
      uint32_t loop_start_time=tree[pos].start_time+tree[pos].cost_time;

      for(int k=0; k<N_TRG; k++)
      {
            if(!InArena(tree_robot_predictor[k])) continue;

            for(int t=loop_start_time/1000+1; t<loop_start_time/1000+PERIOD_TURN/1000; t++)
            {
                  if(shouldExpand[t%(PERIOD_TURN/1000)]==true)
                  {
                        IARC_ROBOT robot,robot_saved;
                        IARC_TASK task;

                        //TURN_180
                        task.type=type_interact;
                        task.start_time=t*1000;
                        task.cost_time=TIME_TURN_180;
                        task.content.interact_id=k;
                        task.content.interact_cmdkind=turn_180;
                        robot_saved=tree_robot_predictor[k];
                        robot=robot_saved;
                        TreePredictorZeroInput(task.start_time,robot,M_PI);
                        robot.time_ms=robot.time_ms+task.cost_time;
                        tree_value=tree_value-coef*TreeRobotValue(robot_saved)+coef*TREE_SEARCH_GAMA*TreeRobotValue(robot);
                        tree_robot_predictor[k]=robot;

                        task.content.interact_exp_x=robot.x;
                        task.content.interact_exp_y=robot.y;
                        task.content.interact_exp_azimuth=Quaternion2ThetaZ(robot.orientation)-M_PI;

                        if(TreeConnector(tree[pos],task)>0)
                        {
                              tree.push_back(task);
                              tree_predictor[k].push_back(task);

                              TreeSearch(depth+1,coef*TREE_SEARCH_GAMA);

                              tree.pop_back();
                              tree_predictor[k].pop_back();
                        }
                        tree_robot_predictor[k]=robot_saved;
                        tree_value=tree_value+coef*TreeRobotValue(robot_saved)-coef*TREE_SEARCH_GAMA*TreeRobotValue(robot);

                        //TURN_45
                        task.type=type_interact;
                        task.start_time=t*1000;
                        task.cost_time=TIME_TURN_45;
                        task.content.interact_id=k;
                        task.content.interact_cmdkind=turn_45;
                        robot_saved=tree_robot_predictor[k];
                        robot=robot_saved;
                        TreePredictorZeroInput(task.start_time,robot,M_PI/4);
                        robot.time_ms=robot.time_ms+task.cost_time;
                        tree_value=tree_value-coef*TreeRobotValue(robot_saved)+coef*TREE_SEARCH_GAMA*TreeRobotValue(robot);
                        tree_robot_predictor[k]=robot;
                        task.content.interact_exp_x=robot.x;
                        task.content.interact_exp_y=robot.y;
                        task.content.interact_exp_azimuth=Quaternion2ThetaZ(robot.orientation)-M_PI/4;
                        if(TreeConnector(tree[pos],task)>0)
                        {
                              tree.push_back(task);
                              tree_predictor[k].push_back(task);

                              TreeSearch(depth+1,coef*TREE_SEARCH_GAMA);

                              tree.pop_back();
                              tree_predictor[k].pop_back();
                        }
                        tree_robot_predictor[k]=robot_saved;
                        tree_value=tree_value+coef*TreeRobotValue(robot_saved)-coef*TREE_SEARCH_GAMA*TreeRobotValue(robot);
                  }
            }
      }
      return 0;
}
int CIARC::TaskListGenerate()
{
      //IARC_OPERATION op;
      //shouldExpand[0];
      tree.clear();
      tree_predictor.clear();
      tree_predictor.resize(N_TRG);
      tree_value=0.0;
      tree_best.clear();
      tree_best_value=-1e7;
      tree_predictor_value.clear();
      tree_predictor_value.resize(N_TRG);
      tree_robot_predictor.clear();
      tree_robot_predictor.resize(N_TRG);
      for(int k=0; k<N_TRG; k++)
      {
            tree_robot_predictor[k]=targets[k];
      }

      IARC_TASK root_task;
      root_task.type=type_hover;
      root_task.cost_time=1;
      root_task.start_time=GetNowTime();
      root_task.value=0.0;
      tree.push_back(root_task);

      TreeSearch(0,1.0);

      task_list.clear();
      if(tree_best.size()>1)
      {
            int sz=tree_best.size();             
            for(int k=1; k<sz; k++)
            {
                  IARC_TASK task;

                  task.type=type_follow;
                  task.start_time=tree_best[k-1].start_time+tree_best[k-1].cost_time;
                  task.cost_time=tree_best[k].start_time-task.start_time;
                  task.content.follow_id=tree_best[k].content.interact_id;
                  task.content.follow_tor_x=2.0;
                  task.content.follow_tor_y=2.0;

                  AddOneTask(task);

                  tree_best[k].content.interact_tor_x=2.0;
                  tree_best[k].content.interact_tor_y=2.0;
                  tree_best[k].content.interact_tor_azimuth=2.0;
                  tree_best[k].content.interact_command_sent=false;
                  tree_best[k].cost_time+=250; //uncertain turn delta time
                  AddOneTask(tree_best[k]);
            }
      }
      else
      {
      	      for(int k=0; k<N_TRG; k++)
      		  {
                  if(!InArena(tree_robot_predictor[k])) continue;
                  else
                  {
                      IARC_TASK task;
                      task.type = type_follow;
                      task.start_time = tree_best[0].start_time+tree_best[0].cost_time;  
                      task.cost_time = tree_best[0].cost_time;
                      task.content.follow_id = k;
                      task.content.follow_tor_x = 10.0;
                      task.content.follow_tor_y = 10.0;                                                               
                      AddOneTask(task);
                      break;                                            
                  }                  
              }
            
      }
      
      if(!task_list.empty())
      {
            task_pos=0;
            task_seq=task_list[task_pos].seq;
      }
      else
      {
            printf("error!   task list not generated!\n");
      }
      //SearchTree()
      //task_list.clear();
      //task_list.push_back();
      return 0;
}

int CIARC::TaskPlan()
{

      pre_path.clear();
      for(int t=0; t<20; t++)
      {
            IARC_ROBOT r;
            if(task_pos>=0 && task_pos<task_list.size())
            {
                  if(task_list[task_pos].type==type_follow)
                  {
                        r=targets[task_list[task_pos].content.follow_id];
                  }
                  else if(task_list[task_pos].type==type_interact)
                  {
                        r=targets[task_list[task_pos].content.interact_id];
                  }
                  else
                  {
                        r=targets[0];
                  }
            }

            TreePredictorZeroInput(t*1000+r.time_ms,r,0.0);

            IARC_POSITION pos;
            pos.x=r.x;
            pos.y=r.y;
            pos.z=0.0;
            pre_path.push_back(pos);

        //    printf("[%.2lf %.2lf] ",pos.x,pos.y);
      }
      //printf("\n");

      enum task_result res=TaskReached();
      bool tolerate=TaskStatusTolerated();

      if(tolerate==true && res==task_going)
      {
            if(task_pos<0 || task_pos>=task_list.size())
            {
                printf("error!  task is not in list!\n");
            }
      }
      else if(tolerate==true && ++task_pos<task_list.size())
     {
           task_seq=task_list[task_pos].seq;
     }
     else
     {
          if(GetNowTime()%PERIOD_TURN<PERIOD_TURN-TIME_TURN_180*1.5 || GetNowTime()%PERIOD_TURN>1000)                     
           {
                 TaskListGenerate();
           }
           else
           {
           		task_list.clear();
		      	IARC_TASK task;
      			task.type=type_hover;
      			task.start_time=GetNowTime();
			    task.cost_time=1000;//hovering for 1000ms every first rtime!
      			AddOneTask(task);	
           }
           
           //SimulatorTask();
           if(task_list.empty())
           {
               printf("error!task list is not generated!\n");
           }
     }

      if(fp_tasklist!=NULL)
      {
            fprintf(fp_tasklist,"%d seq:%d %d(%s) pos:%d last res: %d(%s)\n",GetNowTime(),task_seq,task_list[task_pos].type,str_task_type[task_list[task_pos].type],task_pos,res,str_task_result[res]);
      }

      return 0;
}

int CIARC::PathPlan()
{
      if(task_pos<0 || task_pos>=task_list.size())
      {
            printf("error!  task not in list!\n");
            return 0;
      }
      bool updated=false;
      IARC_TASK task;
      task=task_list[task_pos];

      IARC_WAYPOINT wp;
      wp.t=GetNowTime();
      wp.x=mav.x;
      wp.y=mav.y;

      //wp_pos=0;
      //wp_pos_saved=0;
      //path_list.clear();

      switch(task.type)
      {
      case type_hover:
      {
            if(task.seq!=task_seq_saved)
            {
                  wp_pos=0;
                  wp_pos_saved=0;
                  path_list.clear();

                  wp.x=mav.x;
                  wp.y=mav.y;
                  wp.t=task.start_time+task.cost_time;
                  path_list.push_back(wp);
                  task_seq_saved=task.seq;
                  updated=true;
            }
      }      break;
      case type_reach:
      {
            if(task.seq!=task_seq_saved)
            {
                  wp_pos=0;
                  wp_pos_saved=0;
                  path_list.clear();

                  uint32_t dT=1000;
                  uint32_t new_dT;
                  uint32_t det=task.cost_time;
                  int kpath=0;
                  kpath=det/dT+1;
                  new_dT=det/kpath;

                  wp.t=task.start_time+task.cost_time-(kpath-1)*(new_dT);
                  wp.x=mav.x+(task.content.reach_x-mav.x)/((double)kpath);
                  wp.y=mav.y+(task.content.reach_y-mav.y)/((double)kpath);

                  path_list.push_back(wp);

                  for(int k=1; k<kpath; k++)
                  {
                        wp.t=path_list[0].t+k*(new_dT);
                        wp.x=path_list[0].x+(double(k))*(task.content.reach_x-mav.x)/((double)kpath);
                        wp.y=path_list[0].y+(double(k))*(task.content.reach_y-mav.y)/((double)kpath);
                        path_list.push_back(wp);
                  }
                  task_seq_saved=task.seq;
                  updated=true;
            }
      }     break;

      case type_follow:
      {
            if(task.content.follow_id>=0 && task.content.follow_id<N_TRG)
            {
                  wp_pos=0;
                  wp_pos_saved=0;
                  path_list.clear();
                  wp.t=task.start_time+task.cost_time<GetNowTime()+2000?task.start_time+task.cost_time:GetNowTime()+2000;
                  wp.x=targets[task.content.follow_id].x;
                  wp.y=targets[task.content.follow_id].y;
                  path_list.push_back(wp);
                  task_seq_saved=task.seq;
                  updated=true;
            }
            else
            {
                 printf("error! follow robot ID error!\n");
            }
      } break;
      case type_interact:
      {
            if(task.content.interact_id>=0 && task.content.interact_id<N_TRG)
            {
                  wp_pos=0;
                  wp_pos_saved=0;
                  path_list.clear();
                  wp.t=task.start_time+task.cost_time;
                  wp.x=targets[task.content.interact_id].x;
                  wp.y=targets[task.content.interact_id].y;
                  path_list.push_back(wp);
                  task_seq_saved=task.seq;
                  updated=true;
            }
            else
            {
                  printf("error! interact robot ID error!\n");
            }
      }break;
      default:
            printf("unknow task! can not be handled!\n");
            updated=false;
      }

      if(fp_pathlist!=NULL && updated==true)
      {
            fprintf(fp_pathlist,"%d task seq:%d task type:%d(%s) wp_pos:%d pathlist size:%d\n",GetNowTime(),task.seq,(int)task.type,str_task_type[(int)task.type],wp_pos,(int)path_list.size());
      }
      return 0;
}

int CIARC::MotionPlan()
{
      command.mav_vx=0.0;
      command.mav_vy=0.0;
      command.mav_vz=0.0;
      command.mav_vyawdegree=0.0;
      command.robot_turn_id=0;
      command.robot_turn_kind=turn_none;

     if(task_pos>=0 && task_pos<task_list.size())
     {
           if(task_list[task_pos].type==type_interact && GetNowTime()>=task_list[task_pos].start_time && GetNowTime()<task_list[task_pos].start_time+task_list[task_pos].cost_time && task_list[task_pos].content.interact_command_sent==false)
           {
                 IARC_TASK *p=&task_list[task_pos];
                 static int lsttime=0;
                 if(GetNowTime()-lsttime>task_list[task_pos].cost_time
                             &&fabs(p->content.interact_exp_x-targets[p->content.interact_id].x)<p->content.interact_tor_x
                                 &&fabs(p->content.interact_exp_y-targets[p->content.interact_id].y)<p->content.interact_tor_y
                                 && fabs(p->content.interact_exp_azimuth-Quaternion2ThetaZ(targets[p->content.interact_id].orientation))<p->content.interact_tor_azimuth)
                 {
                       printf("here %d task_seq:%d  id:%d cmd_kind:%d\n",GetNowTime(),task_seq,p->content.interact_id,p->content.interact_cmdkind);
                       command.robot_turn_id=task_list[task_pos].content.interact_id;
                       command.robot_turn_kind=task_list[task_pos].content.interact_cmdkind;
                       lsttime=GetNowTime();
                       p->content.interact_command_sent=true;
                 }
                 else
                 {
                       command.robot_turn_id=0;
                       command.robot_turn_kind=turn_none;
                 }
           }
           else
           {
                 command.robot_turn_id=0;
                 command.robot_turn_kind=turn_none;
           }
     }

     int sz=path_list.size();
     uint32_t nowtime=GetNowTime();
     while(wp_pos>=0 && wp_pos<sz && path_list[wp_pos].t<nowtime)
     {
           wp_pos++;
     }

     printf("wp_pos:%d sz:%d\n",wp_pos,sz);
     if(wp_pos>=0 && wp_pos<sz)
     {
//           double OptimalController(double a0,double k0,double tgo,double x1,double x2,double xt1,double xt2);
           VectorXd  wp3(2),wp2(2),wp1(2),vel2(2);

           wp1(0)=mav.x; wp1(1)=mav.y;
           wp2(0)=path_list[wp_pos].x; wp2(1)=path_list[wp_pos].y;
           vel2(0)=0.0; vel2(1)=0.0;
           double tgo=double(path_list[wp_pos].t-GetNowTime())/1000.0;
           double average_vel=(wp2-wp1).norm()/tgo;
           if(wp_pos+1<sz)
           {
                 wp3(0)=path_list[wp_pos+1].x; wp3(1)=path_list[wp_pos+1].y;
                 vel2=(wp3-wp1).normalized()*average_vel;
           }
           else
           {
                 vel2=(wp2-wp1).normalized()*average_vel;
           }
           printf("wp:%d  (%.2lf %.2lf) (%.2lf %.2lf) tgo:%.2lf vel1:(%.2lf %.2lf) vel2:(%.2lf %.2lf)\n",wp_pos,mav.x,mav.y,wp2(0),wp2(1),tgo,mav.vx,mav.vy,vel2(0),vel2(1));
           command.mav_vx=OptimalController(mav.param_ax,mav.param_kx,tgo,mav.x,mav.vx,wp2(0),vel2(0),mav.param_vmx);
           command.mav_vy=OptimalController(mav.param_ay,mav.param_ky,tgo,mav.y,mav.vy,wp2(1),vel2(1),mav.param_vmy);
           printf("ux,uy:%.2lf %.2lf\n",command.mav_vx,command.mav_vy);

           command.mav_vz=0.0;
           command.mav_vyawdegree=0.0;
           fprintf(fp_controller,"%d,task_seq:%d wp_pos:%d waypoint(%.2lf,%.2lf,t:%d) mav(%.2lf %.2lf) u(%.2lf %.2lf) cmd(%d, %d)\n",
                       GetNowTime(),task_seq,wp_pos,path_list[wp_pos].x,path_list[wp_pos].y,path_list[wp_pos].t,
                       mav.x,mav.y,command.mav_vx,command.mav_vy, command.robot_turn_id, command.robot_turn_kind);
     }
     else
     {
           command.mav_vx=0.0;
           command.mav_vy=0.0;
           command.mav_vz=0.0;
           command.mav_vyawdegree=0.0;

           fprintf(fp_controller,"%d, task_seq:%d error wp(pos: %d size:%d) mav(%.2lf %.2lf) u(%.2lf %.2lf) cmd(%d,%d)\n",
                                   GetNowTime(),task_seq,wp_pos,(int)path_list.size(),
                                  mav.x,mav.y,command.mav_vx,command.mav_vy, command.robot_turn_id, command.robot_turn_kind);
     }

      return 0;
}



int CIARC::MainLoop(IARC_COMMAND &cmd)
{
      seq_loop++;

      //defines an empty command to ensure safety...
/*
      cmd.mav_vx=0.0;
      cmd.mav_vy=0.0;
      cmd.mav_vz=0.0;
      cmd.mav_vyawdegree=0.0;
      cmd.robot_turn_id=0;
      cmd.robot_turn_kind=KIND_TURN_NONE;
*/
      cmd.mav_vx=command.mav_vx;
      cmd.mav_vy=command.mav_vy;
      cmd.mav_vz=command.mav_vz;
      cmd.mav_vyawdegree=command.mav_vyawdegree;
      cmd.robot_turn_id=command.robot_turn_id;
      cmd.robot_turn_kind=command.robot_turn_kind;

//      if(seq_targets>seq_targets_last && seq_obstacles>seq_obstacles_last)
      {
            TaskPlan();
            PathPlan();
            seq_targets_last=seq_targets;
            seq_obstacles_last=seq_obstacles;
      }

      {
            MotionPlan();
            cmd.mav_vx=command.mav_vx;
            cmd.mav_vy=command.mav_vy;
            cmd.mav_vz=command.mav_vz;
            cmd.mav_vyawdegree=command.mav_vyawdegree;
            cmd.robot_turn_id=command.robot_turn_id;
            cmd.robot_turn_kind=command.robot_turn_kind;
      }

      if(fp!=NULL)
      {
            //air position flight position..
            fprintf(fp,"loop: %d %d mav(%.2lf,%.2lf,%.2lf) task_wp(%d %d) seq(%d %d) (x,y,oz,ow):",seq_loop,mav.time_ms,mav.x,mav.y,mav.z,task_seq,wp_pos,seq_targets,seq_obstacles);
           /*
            for(int i=0; i<N_TRG; i++)
            {
                  fprintf(fp,"(%d,%.2lf,%.2lf,%.2lf,%.2lf) ",i,targets[i].x,targets[i].y,targets[i].orientation.z,targets[i].orientation.w);
            }
            for(int i=0; i<N_OBS; i++)
            {
                  fprintf(fp,"(%d,%.2lf,%.2lf,%.2lf,%.2lf) ",i,obstacles[i].x,obstacles[i].y,obstacles[i].orientation.z,obstacles[i].orientation.w);
            }
            */
            fprintf(fp,"cmd:(%.2lf %.2lf %.2lf %.2lf) (%d %d)",cmd.mav_vx,cmd.mav_vy,cmd.mav_vz,cmd.mav_vyawdegree,cmd.robot_turn_id,cmd.robot_turn_kind);
            fprintf(fp,"\n");
      }
      return 0;
}

int CIARC::Simulator(double dT)
{
      /*
//%X=A*X+B*K*U
      %-----------------------Second Order System------------------------------
      T=dT;
      a1=3.6;
      K=3;
      %建立二阶系统模型，进一步进行统计分析，查看是否有可行的解，可行的值。
      %这里是惯性环节模型
      %建立一个同时再建立一个二阶系统模型
      A=[1 1/a1*(1-exp(-a1*T)) 0 0
      0 exp(-a1*T) 0 0
      0 0 1 1/a1*(1-exp(-a1*T))
      0 0 0 exp(-a1*T)];
      B=[1/a1^2*(a1*T+exp(-a1*T)-1) 0
          -1/a1*(exp(-a1*T)-1) 0
          0 1/a1^2*(a1*T+exp(-a1*T)-1)
          0 -1/a1*(exp(-a1*T)-1)];
      */

     MatrixXd G=MatrixXd::Zero(4,4);
    G(0,0)=1.0;    G(0,1)=1.0/mav.param_ax*(1.0-exp(-mav.param_ax*dT)); G(1,1)=exp(-mav.param_ax*dT);
    G(2,2)=1.0; G(2,3)=1.0/mav.param_ay*(1.0-exp(-mav.param_ay*dT)); G(3,3)=exp(-mav.param_ay*dT);

     MatrixXd H=MatrixXd::Zero(4,2);
     H(0,0)=1.0/mav.param_ax/mav.param_ax*(mav.param_ax*dT+exp(-mav.param_ax*dT)-1.0)*mav.param_kx;
     H(1,0)=-1.0/mav.param_ax*(exp(-mav.param_ax*dT)-1.0)*mav.param_kx;
     H(2,1)=1.0/mav.param_ay/mav.param_ay*(mav.param_ay*dT+exp(-mav.param_ay*dT)-1.0)*mav.param_ky;
     H(3,1)=-1.0/mav.param_ay*(exp(-mav.param_ay*dT)-1.0)*mav.param_ky;

     VectorXd X(4),U(2);
     X(0)=mav.x; X(1)=mav.vx;
     X(2)=mav.y; X(3)=mav.vy;
     U(0)=command.mav_vx;
     U(1)=command.mav_vy;

     X=G*X+H*U;

     mav.x=X(0); mav.vx=X(1);
     mav.y=X(2); mav.vy=X(3);

     printf("state:%.2lf %.2lf %.2lf %.2lf\n",X(0),X(1),X(2),X(3));
      return 0;
}
