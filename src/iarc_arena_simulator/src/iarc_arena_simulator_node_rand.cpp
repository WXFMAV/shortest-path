#include "iarc_arena_simulator_node.h"
using namespace iarc_arena_simulator;

static ros::Time time_start;
static uint32_t time_turn[N_TRG],kind_turn[N_TRG];
static IARCQuadStatus quad_status;
static FILE *fp=NULL;
static FILE *fp_cmdrcv=NULL;
static geometry_msgs::Pose obs[N_OBS];
static geometry_msgs::Pose trg[N_TRG];

uint32_t TimePassedMs(ros::Time start,ros::Time now);

bool near_mav(uint32_t robot_id)
{
	return true;
    if(robot_id< N_TRG)
    {
        double dis = 0.0;
        dis = sqrt((quad_status.x - trg[robot_id].position.x)*(quad_status.x - trg[robot_id].position.x)
                + (quad_status.y - trg[robot_id].position.y)*(quad_status.y - trg[robot_id].position.y)
                + (quad_status.z - trg[robot_id].position.z)*(quad_status.z - trg[robot_id].position.z));
        if (dis < 2.0) return true;  //command can be exed within 2 meters
    }
    return false;
}

void IARCQuadStatus_callback(const iarc_arena_simulator::IARCQuadStatus::ConstPtr & msg)
{
    quad_status = (*msg);
}

void IARCCommand_callback(const iarc_arena_simulator::IARCCommand::ConstPtr& msg)
{
    //printf("msg: id:%d kind:%d\n",msg->robot_id,msg->command_kind);
    ros::Time now=ros::Time::now();
    uint32_t time_now=TimePassedMs(time_start,now);
    uint32_t time_add=0;
    if(msg->command_kind==KIND_TURN_180) time_add=TIME_TURN_180;
    else time_add=TIME_TURN_45;

          if(msg->command_kind!=KIND_TURN_NONE && near_mav(msg->robot_id)==true)
          {
                time_turn[msg->robot_id]=time_now%PERIOD_TURN;
                kind_turn[msg->robot_id]=msg->command_kind;
                if(fp_cmdrcv!=NULL)
                {
                      fprintf(fp_cmdrcv,"robot_id:%d time_turn:%d kind_turn:%d[%s]\n",msg->robot_id,time_now,msg->command_kind,str_kind_turn[msg->command_kind]);
                      fflush(fp_cmdrcv);
                }
          }
    //}
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
bool OutofBound(geometry_msgs::Pose pos)
{
	if(pos.position.x<-10.0 || pos.position.x>10.0 || pos.position.y<-10.0 || pos.position.y>10.0)
	{
		return true;
	}
	return false;
}
int main(int argc,char **argv)
{
    ros::init(argc, argv, "iarc_arena");
    ros::NodeHandle nh;
    time_start=ros::Time::now();

    ros::Publisher obstacles_pub = 
    	nh.advertise<geometry_msgs::PoseArray>("/iarc_arena/IARCObstacles", 10);
    ros::Publisher targets_pub = 
    	nh.advertise<geometry_msgs::PoseArray>("/iarc_arena/IARCTargets",10);
    ros::Subscriber cmd_sub = 
        nh.subscribe<iarc_arena_simulator::IARCCommand>("/iarc_arena/IARCCommand",10,IARCCommand_callback);
    ros::Subscriber quad_sub =
        nh.subscribe<iarc_arena_simulator::IARCQuadStatus>("/iarc_arena/IARCQuadStatus",10, IARCQuadStatus_callback);
    fp=fopen(file_name_arena_info.c_str(),"w");
    if(fp==NULL)
    {
          printf("file arena_info.txt open failed!\n");
          return 0;
    }

    fp_cmdrcv=fopen(file_name_arena_cmd_received.c_str(),"w");
    if(fp_cmdrcv==NULL)
    {
          printf("file arena_cmd_receivd.txt open failed!\n");
          return 0;
    }


    srand((unsigned int)time_start.nsec);    

	float obs_theta[N_OBS],trg_theta[N_TRG];
	
					
   	geometry_msgs::Pose zeropose;
	zeropose.position.x=0.0;
	zeropose.position.y=0.0;
	zeropose.position.z=0.0;
	zeropose.orientation.x=0.0;
	zeropose.orientation.y=0.0;		
	zeropose.orientation.z=0.0;				
	zeropose.orientation.w=1.0;
    
    for(int k=0; k<N_TRG; k++)
    {
    	  double re,rx,ry;
    	  re=double(rand()%1000)/1000.0*2.0*PI;
    	  rx=double(rand()%1000)/1000.0*5.0;
    	  ry=double(rand()%1000)/1000.0*5.0;    	      	  
          trg_theta[k]=re;
          trg[k]=zeropose;
          trg[k].position.x=rx;
          trg[k].position.y=ry;
          time_turn[k]=PERIOD_TURN-TIME_TURN_180;
          kind_turn[k]=KIND_TURN_180;
    }

    trg[0].position.x = 6.0;
    trg[0].position.y = 1.0;
    trg_theta[0] = PI/4;
    trg[1].position.x = 6.0;
    trg[1].position.y = -1.0;
    trg_theta[1] = PI/4;
    trg[2].position.x = 5.0;
    trg[2].position.y = -3.0;
    trg_theta[2] = PI/4;


    for(int k=0; k<N_OBS; k++)
    {
          obs_theta[k]=2*PI/(N_OBS)*k;
          obs[k]=zeropose;
          obs[k].position.x=obs[k].position.x+float(rand()%1001-500)/500.0;
          obs[k].position.y=obs[k].position.y+float(rand()%1001-500)/500.0;
    }
            
    ros::Rate r(30);
    float dT=1/30.0;
    float v0=0.33;
    float r0=5.0;            
    int cnt=0;
    
    
    while(ros::ok())
    {   
        if (TimePassedMs(time_start,ros::Time::now())>600000)
           break;
        cnt++;
    	geometry_msgs::PoseArray obstacles;
    	geometry_msgs::PoseArray targets;
    	obstacles.header.frame_id = "/arena_frame";    	    	    	
    	obstacles.header.stamp = ros::Time::now();
    	targets.header.frame_id = "/arena_frame";    	    	    	
    	targets.header.stamp = ros::Time::now();

    	geometry_msgs::Pose ps;
		ps=zeropose;
										
    	for(int k=0; k<N_TRG; k++)
    	{
    		if(!OutofBound(trg[k]))
    		{
				float theta,de,theta_add;
				uint32_t time_now;												
				uint32_t time_add;

				ros::Time now=ros::Time::now();
				time_now=TimePassedMs(time_start,now)%PERIOD_TURN;				
				if(kind_turn[k]==KIND_TURN_180)
                        {
				      theta_add=THETA_TURN_180;
				      time_add=TIME_TURN_180;
                        }
				else if(kind_turn[k]==KIND_TURN_45)
                        {
				      theta_add=THETA_TURN_45;
				      time_add=TIME_TURN_45;
                        }
				else
				{
				      theta_add=0.0;
				      time_add=0;
				}

				time_add=time_add/((int)(dT*1000.0))*((int)(dT*1000.0));

				if(time_now>=time_turn[k]+time_add || time_now<(time_turn[k]))			
				{		
					de=0.0;
					v0=0.33;
				}
				else
				{
					de=theta_add/(((double)time_add)/1000.0)*((int)(dT*1000.0)/1000.0);
					v0=0.0;
					/*
					if(fp_cmdrcv!=NULL)
					{
					   //   fprintf(fp_cmdrcv,"time:%d robotid:%d turn time:%d kind:%d\n",time_now,k,time_turn[k],kind_turn[k]);
					}
					*/
				}		
				if(time_now+(int)(dT*1000.0)>=PERIOD_TURN-TIME_TURN_180 && time_now<PERIOD_TURN-TIME_TURN_180)
				{
					time_turn[k]=PERIOD_TURN-TIME_TURN_180;
					kind_turn[k]=KIND_TURN_180;
					//printf("20s 180 beging!\n");
				}
									
				//printf("%d %d %d %d %.6lf %d\n",time_now,k,kind_turn[k],time_turn[k],de,time_add);								
				trg_theta[k]=trg_theta[k]+de;						
				trg_theta[k]=trg_theta[k]+float(rand()%1001-500)/50000.0;						
				theta=trg_theta[k];
						
				trg[k].position.x=trg[k].position.x+dT*v0*cos(theta);
				trg[k].position.y=trg[k].position.y+dT*v0*sin(theta);			
										
				trg[k].orientation.x=0.0;
				trg[k].orientation.y=0.0;
				trg[k].orientation.z=cos((-theta+PI)/2);
				trg[k].orientation.w=sin((-theta+PI)/2);
			}
			else
			{
			}
			
			targets.poses.push_back(trg[k]);
    	}
    	
    	for(int k=0; k<N_OBS; k++)
    	{    		    	    	
    		float v0 = 0.33;
			if(!OutofBound(obs[k]))
			{
				float theta;
				obs_theta[k]=obs_theta[k]+float(rand()%1001-500)/500000.0;			
				obs_theta[k]=obs_theta[k]+dT*v0/r0;
				theta=obs_theta[k];
				obs[k].position.x += v0*cos(-theta+PI/2) *dT;
				obs[k].position.y +=v0*sin(-theta+PI/2) *dT;

				//obs[k].position.x=obs[k].position.x; //r0*cos(theta);
				//obs[k].position.y=obs[k].position.y; //r0*sin(theta);
				obs[k].orientation.x=0.0;
				obs[k].orientation.y=0.0;
				obs[k].orientation.z=cos((-theta+PI/2)/2);
				obs[k].orientation.w=sin((-theta+PI/2)/2);			
			}
			else
			{
				
			}
			obstacles.poses.push_back(obs[k]);
    	}
    	
    	obstacles_pub.publish(obstacles);
    	targets_pub.publish(targets);

    	if(fp!=NULL)
    	{
    	      uint32_t time_now=TimePassedMs(time_start,ros::Time::now());
    	      fprintf(fp,"%d %d (id,x,y,oz,ow):",cnt,time_now);
    	      for(int k=0; k<N_TRG; k++)
    	      {
    	            fprintf(fp,"(%d %.2lf %.2lf %.2lf %.2lf) ",k,trg[k].position.x,trg[k].position.y,trg[k].orientation.z,trg[k].orientation.w);
    	      }
    	      for(int k=0; k<N_OBS; k++)
              {
                    fprintf(fp,"(%d %.2lf %.2lf %.2lf %.2lf) ",k,obs[k].position.x,obs[k].position.y,obs[k].orientation.z,obs[k].orientation.w);
              }
    	      fprintf(fp,"\n");
    	}
    	r.sleep();    	    	
    	ros::spinOnce();
    }

    if(fp!=NULL) fclose(fp),fp=NULL;
    if(fp_cmdrcv!=NULL) fclose(fp_cmdrcv),fp_cmdrcv=NULL;
	return 0;
}
