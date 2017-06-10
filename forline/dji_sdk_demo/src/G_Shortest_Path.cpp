#include<iostream>
#include<queue>
#include<glog/logging.h>

#include "G_Shortest_Path.h"
using namespace std;


G_Shortest_Path::G_Shortest_Path()
{
	orgx=0.0;orgy=0.0;orgt=0.0;
	aimx=0.0;aimy=0.0;aimt=0.0;	
	step_t=0.0; step_x=0.0; step_y=0.0;
	range_xmin=0.0; range_ymin=0.0; range_xmax=0.0; range_ymax=0.0;
	danger_radius=0.0;
	options=0;
	memset(obs,0,sizeof(double)*TOTAL_OBS*5);
	int tD[9][2]={{0,0},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};//9���ڽ���ķ���
	memcpy(D,tD,sizeof(int)*9*2);
	F = NULL;
	Cost = NULL;
}
G_Shortest_Path::~G_Shortest_Path()
{
	Release();
}

int G_Shortest_Path::Release()
{
	if(F!=NULL)
	{
		for(int t=0; t<NT; t++)
		{
			for(int x=0; x<NX; x++)
			{
				delete []F[t][x];
				F[t][x]=NULL;
			}
			delete []F[t];
			F[t]=NULL;
		}
		delete []F;
		F=NULL;
	}

	if(Cost!=NULL)
	{
		for(int x=0; x<NX; x++)
		{
		    if(Cost[x]!=NULL){
			    delete []Cost[x];
		    }
			Cost[x]=NULL; 
		}
		delete []Cost;
		Cost=NULL;
	}

	return 0;
}
int G_Shortest_Path::Init(double p_orgx,double p_orgy,double p_orgt,double p_aimx,double p_aimy,double p_aimt,double p_obs[TOTAL_OBS][5],double p_danger_radius,
		double p_step_t,double p_step_x,double p_step_y,double p_range_xmin,double p_range_ymin,double p_range_xmax,double p_range_ymax,int p_options,int p_total_obs)
{	
	//---------------------------------Init Param
	this->orgx=p_orgx;	this->orgy=p_orgy;	this->orgt=p_orgt;
	this->aimx=p_aimx;	this->aimy=p_aimy;	this->aimt=p_aimt;
	this->step_t=p_step_t; this->step_x=p_step_x; this->step_y=p_step_y;
	this->range_xmin=p_range_xmin; this->range_ymin=p_range_ymin; this->range_xmax=p_range_xmax,this->range_ymax=p_range_ymax;
	this->options=p_options;
	this->total_obs=p_total_obs;
	this->danger_radius=p_danger_radius;
	for(int i=0; i<p_total_obs; i++)
	{
		for(int j=0; j<5; j++)
		{
			this->obs[i][j]=p_obs[i][j];
		}
	}
	//----------------------------------Build Matrix	
	NT=(aimt-orgt)/step_t;
	NX=(range_xmax-range_xmin)/step_x;
	NY=(range_ymax-range_ymin)/step_y;
	Bx=(orgx-range_xmin)/step_x;
	By=(orgy-range_ymin)/step_y;
	Ex=(aimx-range_xmin)/step_x;
	Ey=(aimy-range_ymin)/step_y;

	NX=NX>0?NX:1;
	NY=NY>0?NY:1;
	NT=NT>0?NT:1;

	LOG(INFO)<<"aim"<<aimt<<" "<<aimx<<" "<<aimy<<" org"<<orgt<<" "<<orgx<<" "<<orgy;
	LOG(INFO)<<"step"<<step_t<<" "<<step_x<<" "<<step_y;
	LOG(INFO)<< "NT=" <<NT<<"NX= "<<NX<<" "<<" NY"<<NY;
	LOG(INFO)<<"BX="<<Bx<<" By="<<By<<" Ex="<<Ex<<" Ey="<<Ey;

	if(NT*NX*NY>200*1000*1000)
	{
		LOG(ERROR) <<"NT*NX*NY is too big! stack overflow ="<<NT*NX*NY<<" bytes";
		NT=0;
		NX=0;
		NY=0;
	}
	F=NULL;
	Cost=NULL;
	if(this->options!=TYPE_ASTAR)
	{
		F=new int**[NT];

		for(int t=0; t<NT; t++)
		{
			F[t]=new int*[NX];
			for(int x=0; x<NX; x++)
			{
				F[t][x]=new int[NY];
				for(int y=0; y<NY; y++)
				{
					F[t][x][y]=MAX_VALUE_DP;//˳���ʼ�����������㡣
					//���ٻ����ģ����Զ㿪�ϰ��������
				}
			}
		}
	}
	else
	{		
		Cost=new int*[NX];
		for(int x=0; x<NX; x++)
		{
			Cost[x]=new int[NY];
			for(int y=0; y<NY; y++)
			{
				Cost[x][y]=MAX_VALUE_DP;
			}
		}
	}
	ramda=0.6;
	line_length2=(orgx-aimx)*(orgx-aimx)+(orgy-aimy)*(orgy-aimy);//��ɢ�����Ӵ��𣿷�ɢ����
	safe_distance2=25*danger_radius*danger_radius;//Σ�վ����5������Ϊ��ȫ����
	A=(orgy-aimy); B=(aimx-orgx); C=orgx*aimy-aimx*orgy;
	double r1,r2;
	r1=danger_radius;
	r2=r1*5;
	K1=1/(1/r1-1/r2);
	C1=-K1/r2;
	return 0;
}

//comment at 2016/3/13 21:21  -distance to line min, distance to obs max.
//int G_Shortest_Path::EstimateXY(double cx,double cy,double ct,double &dis1,double &dis2,int &dis)
//{
//	dis1=(A*cx+B*cy+C)*(A*cx+B*cy+C)/(A*A+B*B);
//	dis2=MAX_VALUE_DP;
//	for(int k=0; k<TOTAL_OBS; k++)
//	{
//		if(obs[k][4]>=0.1)//�ɼ��� ��ʾ�ǿɼ����ϰ������λ�õ�
//		{						
//			double tmp=(obs[k][0]+obs[k][2]*(ct-orgt)-cx)*(obs[k][0]+obs[k][2]*(ct-orgt)-cx)
//				+(obs[k][1]+obs[k][3]*(ct-orgt)-cy)*(obs[k][1]+obs[k][3]*(ct-orgt)-cy);
//			dis2=tmp<dis2?tmp:dis2;
//		}
//	}
//
//	if(dis2<danger_radius*danger_radius)
//	{
//		dis=100;//���ɴ�����
//		return FLAG_DANGER;
//	}
//	else
//	{
//		dis1=dis1<line_length2?dis1:line_length2;//��Ҫ���й�һ���� //���ߵľ��볬�����ٿ��Բ��ÿ����ˣ�С�ڶ���Ҳ���Բ��ÿ����ˣ���					
//		dis2=dis2-danger_radius*danger_radius;//��0
//		dis2=dis2<safe_distance2?dis2:safe_distance2;
//		dis1=dis1/line_length2;
//		dis2=1.0-dis2/safe_distance2;//����5���࣬��Σ�ա�
//		dis=(int)((dis1*ramda+dis2*(1-ramda))*100.0);	//�����һ��0-1֮�����.�ض���滮��������·�������� ���հٷֱ�������.
//	}
//	return FLAG_SAFE;
//}

//comment at 2016/3/13 22:34 mini distance to x2,y2 
//int G_Shortest_Path::EstimateXY(double cx,double cy,double ct,double &dis1,double &dis2,int &dis)
//{
//	dis1=(cx-aimx)*(cx-aimx)+(cy-aimy)*(cy-aimy);
//	dis2=MAX_VALUE_DP;
//	for(int k=0; k<TOTAL_OBS; k++)
//	{
//		if(obs[k][4]>=0.1)//�ɼ��� ��ʾ�ǿɼ����ϰ������λ�õ�
//		{						
//			double tmp=(obs[k][0]+obs[k][2]*(ct-orgt)-cx)*(obs[k][0]+obs[k][2]*(ct-orgt)-cx)
//				+(obs[k][1]+obs[k][3]*(ct-orgt)-cy)*(obs[k][1]+obs[k][3]*(ct-orgt)-cy);
//			dis2=tmp<dis2?tmp:dis2;
//		}
//	}
//
//	if(dis2<danger_radius*danger_radius)
//	{
//		dis=100;//���ɴ�����
//		return FLAG_DANGER;
//	}
//	else
//	{
//		dis1=dis1<line_length2?dis1:line_length2;//��Ҫ���й�һ���� //���ߵľ��볬�����ٿ��Բ��ÿ����ˣ�С�ڶ���Ҳ���Բ��ÿ����ˣ���					
//		dis2=dis2-danger_radius*danger_radius;//��0
//		dis2=dis2<safe_distance2?dis2:safe_distance2;
//		dis1=dis1/line_length2;
//		dis2=1.0-dis2/safe_distance2;//����5���࣬��Σ�ա�
//		dis=(int)((sqrt(dis1)*ramda+dis2*(1-ramda))*100.0);	//�����һ��0-1֮�����.�ض���滮��������·�������� ���հٷֱ�������.
//	}
//	return FLAG_SAFE;
//}
int G_Shortest_Path::EstimateXY(double cx,double cy,double ct,double &dis1,double &dis2,int &dis)
{
	dis1=sqrt((cx-aimx)*(cx-aimx)+(cy-aimy)*(cy-aimy));
	//dis1=(A*cx+B*cy+C)*(A*cx+B*cy+C)/(A*A+B*B);
	dis2=MAX_VALUE_DP;

	double line_length=sqrt(line_length2)*2.0;
	double totdis2=0.0;
	double totobs=0,avgdis2=0.0;
	for(int k=0; k<total_obs; k++)//��������һ�Ѿ�ֹ���ϰ����
	{
		if(obs[k][4]>=0.1)//�ɼ��� ��ʾ�ǿɼ����ϰ������λ�õ�
		{						
			double tmp=(obs[k][0]+obs[k][2]*(ct-orgt)-cx)*(obs[k][0]+obs[k][2]*(ct-orgt)-cx)
				+(obs[k][1]+obs[k][3]*(ct-orgt)-cy)*(obs[k][1]+obs[k][3]*(ct-orgt)-cy);
			tmp=sqrt(tmp);
			dis2=tmp<dis2?tmp:dis2;
			totdis2+=tmp;//���ϰ��������ܺ�
			totobs=totobs+1.0;
		}
	}	

	if(dis2<danger_radius)//��Ŀ���Խ��ʱ�����Ϸ�ΧӦ��Խ����У������ܸ����������㹻��ʱ��
	{
		dis=100;//���ɴ�����
		return FLAG_DANGER;
	}
	else
	{
		dis1=dis1<line_length?dis1:line_length;//��Ҫ���й�һ���� //���ߵľ��볬�����ٿ��Բ��ÿ����ˣ�С�ڶ���Ҳ���Բ��ÿ����ˣ���					
		//dis2=dis2-danger_radius*danger_radius;//��0
		//dis2=dis2<safe_distance2?dis2:safe_distance2;

		dis1=dis1/line_length;		
		//dis1=0.0; //dis1 ��Ӧ����Сֵ��������ξ������ֵ�أ��������Ŀ��ƽ�Ŀ���һ�㣿
		//dis1=(dis1>0.1?dis1:0.1);
		//dis2=danger_radius*danger_radius/dis2;//����5���࣬��Σ�ա�
		dis2=totdis2/totobs;
		dis2=(dis2>0.01?dis2:0.01);
		dis2=K1/dis2+C1;//ת����
		dis2=(dis2>1.0?1.0:dis2);
		dis2=(dis2<0.0?0.0:dis2);			
		dis=(int)((dis1*ramda+dis2*(1-ramda))*100.0);	//�����һ��0-1֮�����.�ض���滮��������·�������� ���հٷֱ�������.
		//dis=1.0;
	}
	return FLAG_SAFE;
}
int G_Shortest_Path::DPPath(int &n_step)
{	
	if(A<=step_y && B<=step_x)
	{
		n_step=0;
		return 0;
	}

	double tdis1,tdis2;
	int tdis;
	int flag=EstimateXY(aimx,aimy,aimt,tdis1,tdis2,tdis);
	F[NT-1][Ex][Ey]=(flag==FLAG_DANGER?MAX_VALUE_DP:tdis);

	printf("danger:%d\n",flag);
	//FILE *fp=fopen("tmp.txt","w");

	for(int t=NT-2; t>=0; t--)
	{
		for(int x=0; x<NX; x++)
		{
			for(int y=0; y<NY; y++)
			{
				double dis1,dis2;
				int dis;
				double cx,cy,ct;
				cx=((double)x)*step_x+range_xmin;
				cy=((double)y)*step_y+range_ymin;
				ct=((double)t)*step_t+orgt;

				
				if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
				{
					//fprintf(fp,"0");
					continue;
				}
				bool found=false;

				for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
				{
					int nx=x+D[k][0];
					int ny=y+D[k][1];
					if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;
					
					if(k==0)
					{
						dis=0;
					}
					else
					{
						dis=1;
					}

					if(F[t][x][y]>F[t+1][nx][ny]+dis)
					{
						F[t][x][y]=F[t+1][nx][ny]+dis;//��Χ8�㣬�������ָ��Ĳ��һ������
						found=true;
					}
				}
				//if(found) fprintf(fp,"1");
				//else fprintf(fp,"0");
			}
			//fprintf(fp,"\n");
		}
	//	fprintf(fp,"\n");
	}

	//fclose(fp);

	if(F[0][Bx][By]<MAX_VALUE_DP)
	{
		n_step=NT;
		printf("best value:%d\n",F[0][Bx][By]);
	}
	else
	{
		n_step=0;
		printf("no sulotions");//û��������Ľ�
	}
	return 0;
}

int G_Shortest_Path::GetPath(double **path)
{
	if(F[0][Bx][By]>=MAX_VALUE_DP)
	{
		for(int i=0;i<=NT; i++)
		{
			path[i][0]=orgx; path[i][1]=orgy; path[i][2]=orgt;
		}
		return 0;
	}

	int bestvalue=F[0][Bx][By];	
	int **bestpath=new int*[NT];

	for(int t=0; t<NT; t++)
		bestpath[t]=new int[3];

	bestpath[0][0]=Bx;
	bestpath[0][1]=By;
	bestpath[0][2]=0;

	bool wrongflag = false;
	int wrong_at = NT;
	//printf("Begin at:(%d,%d)\n",Bx,By);
	
	//FILE *fp=fopen("danger.txt","w");
	for(int t=1; t<NT; t++)
	{
		int x=bestpath[t-1][0];
		int y=bestpath[t-1][1];		

				
		double dis1,dis2;
		int dis;
		double cx,cy,ct;
		cx=((double)x)*step_x+range_xmin;
		cy=((double)y)*step_y+range_ymin;
		ct=((double)(t-1))*step_t+orgt;

				
		if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
		{
			printf("Path in danger area! at step:%d\n",t);
			wrongflag=true;
			break;
		}

		//fprintf(fp,"%d %d %d %d\n",t-1,(int)(dis1*100.0*ramda),(int)(dis2*100.0*(1.0-ramda)),dis);

		bool found=false;
		for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
		{
			int nx=x+D[k][0];
			int ny=y+D[k][1];
			if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;
			
			if(k==0)
			{
				dis=0;
			}
			else
			{
				dis=1;
			}

			if(F[t-1][x][y]==F[t][nx][ny]+dis)
			{
				F[t-1][x][y]=F[t][nx][ny]+dis;//��Χ8�㣬�������ָ��Ĳ��һ������
				bestpath[t][0]=nx;
				bestpath[t][1]=ny;
				bestpath[t][2]=t;
				found=true;
				break;
			}
		}
		if(!found)
		{
			printf("Error! path can not be rebuild! at step :%d\n",t);
			wrongflag=true;
			wrong_at = t;
			break;
		}
		else
		{
			//printf("step:%d, goto (%d,%d)\n",t,bestpath[t][0],bestpath[t][1]);
		}
	}
//	fclose(fp);
	//printf("End at:(%d,%d)\n",Ex,Ey);
	if(wrongflag)
	{
		printf("path error detected!\n");
	}

	{
		for(int i=0; i<wrong_at; i++)
		{
			path[i][0]=((double)bestpath[i][0])*step_x+range_xmin;
			path[i][1]=((double)bestpath[i][1])*step_y+range_ymin;
			path[i][2]=((double)bestpath[i][2])*step_t+orgt;
		}
		for(int i=wrong_at; i<NT; i++)
		{
		    path[i][0] = path[i-1][0];
		    path[i][1] = path[i-1][1];
		    path[i][2] = path[i-1][2];
		}
	}
	for(int i=0; i<NT; i++)
	{
		delete []bestpath[i];
	}
	delete []bestpath;

	return 0;
}


int G_Shortest_Path::Hope(int x,int y)
{
	return abs(x-Ex)+abs(y-Ey);
}
int G_Shortest_Path::AStar(int &nstep)
{
	globalmin=MAX_VALUE_ASTAR;
	double tdis1,tdis2;
	int tdis;
//	int flag=EstimateXY(aimx,aimy,aimt,tdis1,tdis2,tdis);//Ŀ����Ƿ����ϰ�
	//Cost[Ex][Ey]=(flag==FLAG_DANGER?MAX_VALUE_ASTAR:tdis);
	int flag=EstimateXY(orgx,orgy,orgt,tdis1,tdis2,tdis);//��ʼ���Ƿ�û���ϰ�	
	//Cost[Bx][By]=(flag==FLAG_DANGER?MAX_VALUE_ASTAR:tdis);
	Cost[Bx][By]=(flag==FLAG_DANGER?MAX_VALUE_ASTAR:0);
	bool **Openset;
	Openset=new bool*[NX];
	for(int x=0; x<NX; x++)
	{
		Openset[x]=new bool[NY];	
		for(int y=0; y<NY; y++)
		{
			Openset[x][y]=false;
		}
	}
	priority_queue<node>Q;
	node now;
	now.x=Bx; now.y=By; now.p=this;
	Q.push(now);
	Openset[Bx][By]=true;
	while(!Q.empty())//���ٴ���һ������չ�ڵ�
	{
		node now=Q.top();
		Q.pop();

		int x=now.x;
		int y=now.y;		
		Openset[x][y]=false;
		//��¼�Լ�����С����ֵ������
		for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
		{
			int nx=x+D[k][0];
			int ny=y+D[k][1];
			int nt=Cost[x][y];
			if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;

			double dis1,dis2;
			int dis;
			double cx,cy,ct;
			cx=((double)nx)*step_x+range_xmin;
			cy=((double)ny)*step_y+range_ymin;
			ct=((double)nt)*step_t+orgt;
			//��ʵλ�ã���ΪEstimateXY���������.
			if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
			{
				continue;
			}
			if(Cost[nx][ny]>Cost[x][y]+1)
			{
				Cost[nx][ny]=Cost[x][y]+1;
				//QueAdd(Q,nx,ny,pr,NQ);//���������BFS��Ȼ�������BFS�Ļ����ϼ���һ����������ԡ���
				//��Q������һ���ѣ�Ȼ��Ϳ��ԱȽϿ��ٵ��������ˡ� 
				node tmp;				
				tmp.x=nx; tmp.y=ny; tmp.p=this;
				if(Openset[tmp.x][tmp.y]==false)
				{
					Q.push(tmp);//�����ⲻ���ö���������Ϊ���䲻��һ���ѣ��任�������м����ֵ�ǻᷢ���仯�ġ���
					Openset[tmp.x][tmp.y]=true;
				}
				if(nx==Ex && ny==Ey)
				{
					globalmin=Cost[Ex][Ey];
					break;
				}
			}
		}
	}

	if(globalmin<MAX_VALUE_DP)
	{//found best solutions.	
		nstep=NT;
	}
	else
	{
		printf("No solutions found!\n");
		nstep=-1;
	}
	for(int x=0; x<NX; x++)
	{
		delete []Openset[x];
		Openset[x]=NULL;
	}
	delete []Openset;
	Openset=NULL;
	return 0;
}
int G_Shortest_Path::GetPath_AStar(double ** path)
{
	return 0;
}
int G_Shortest_Path::BFS(int &nstep)
{	
	globalmin=MAX_VALUE_DP;
	double tdis1,tdis2;
	int tdis;
	int flag=EstimateXY(aimx,aimy,aimt,tdis1,tdis2,tdis);
	F[NT-1][Ex][Ey]=(flag==FLAG_DANGER?MAX_VALUE_DP:tdis);
	flag=EstimateXY(orgx,orgy,orgt,tdis1,tdis2,tdis);
	F[0][Bx][By]=(flag==FLAG_DANGER?MAX_VALUE_DP:tdis);//�����ʼ�����������⣿Ӧ��ֻ��һ�����ܡ���

	int NQ=NX*NY;
	int ***Q;
	Q=new int**[2];

	for(int k=0; k<2; k++)
	{
		Q[k]=new int*[2];
		for(int xy=0; xy<2; xy++)
		{
			Q[k][xy]=new int[NX*NY];
		}
	}

	int pr[2][3];//begin end l
	int q=0,t=1;
	
	pr[0][0]=0; pr[0][1]=0; pr[0][2]=0; //begin end l
	pr[1][0]=0; pr[1][1]=0; pr[1][2]=NT-1;

	Q[q][0][pr[q][0]]=Bx; Q[q][1][pr[q][0]]=By;
	Q[t][0][pr[t][0]]=Ex; Q[t][1][pr[t][0]]=Ey;

	FILE *fp=fopen("rec.txt","w");
	printf("NT=%d\n",NT);
	while(pr[0][2]+1<=pr[1][2])
	{	
		if(pr[0][1]-pr[0][0]<pr[1][1]-pr[1][0])
		{
			q=0; t=1;
		}
		else
		{
			q=1; t=0;
		}

		if(pr[0][1]-pr[0][0]<0 || pr[1][1]-pr[1][0]<0)
		{
			printf("Can not expand anymore, no solutions\n");
			break;
		}

		//��չq��������е�Ԫ��
		int jn=pr[q][1]+1;
		for(int j=pr[q][0]; j<jn; j++)
		{
			int x=Q[q][0][j%NQ];
			int y=Q[q][1][j%NQ];
			int t=pr[q][2];

			for(int ix=0; ix<NX; ix++){
			    for(int iy=0; iy<NY; iy++){
			        if (F[t][ix][iy]<MAX_VALUE_DP){
			            fprintf(fp,"0 ");
			        }
			        else{
			            fprintf(fp,"x ");
			        }
			    }
			    fprintf(fp,"\n");
			}
			fprintf(fp,"\n");
			
			bool found=false;

			for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
			{
				int nx=x+D[k][0];
				int ny=y+D[k][1];
				int nt=t-(q*2-1);

				if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;

				double dis1,dis2;
				int dis;
				double cx,cy,ct;
				cx=((double)nx)*step_x+range_xmin;
				cy=((double)ny)*step_y+range_ymin;
				ct=((double)nt)*step_t+orgt;
				//��ʵλ�ã���ΪEstimateXY���������.
				if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
				{
					continue;
				}
				//q*2-1:-1,1... q=0, 
				if(k==0)
				{
					dis=0;
				}
				else
				{
					dis=1.0;
				}

				if(pr[0][2]+1==pr[1][2])
				{
					if(globalmin>F[nt][nx][ny]+F[t][x][y])
					{
						globalmin=F[nt][nx][ny]+F[t][x][y];
						if(nt>t)
						{
							globalmin_dt=t;globalmin_dx=x;globalmin_dy=y;
							globalmin_ut=nt;globalmin_ux=nx;globalmin_uy=ny;
						}
						else if(nt<t)
						{
							globalmin_dt=nt; globalmin_dx=nx;globalmin_dy=ny;
							globalmin_ut=t; globalmin_ux=x; globalmin_uy=y;
						}
						else
						{
							printf("impossible error occurs! Logic dump!\n");
							break;
						}
					}
				}
				else if(F[nt][nx][ny]>F[t][x][y]+dis)
				{						
					if(F[nt][nx][ny]>=MAX_VALUE_DP)
					{
						pr[q][1]++;
						Q[q][0][pr[q][1]%NQ]=nx;
						Q[q][1][pr[q][1]%NQ]=ny;						
					}
					F[nt][nx][ny]=F[t][x][y]+dis;//ֻ�������һ�Ρ�					
				}
			}

		}
		pr[q][0]=jn;//�����˫������������
		pr[q][2]+=-(q*2-1);
	}	
	fclose(fp);
	if(globalmin<MAX_VALUE_DP)
	{//found best solutions.	
		nstep=NT;
	}
	else
	{
	    printf( "No solutions found!\n");
		nstep=-1;
	}

	for(int k=0; k<2; k++)
	    {
	        //Q[k]=new int*[2];
	        for(int xy=0; xy<2; xy++)
	        {
	           // Q[k][xy]=new int[NX*NY];
	            delete []Q[k][xy];
	        }
	        delete []Q[k];
	    }
	delete []Q;
	Q=NULL;

	return 0;
}


int G_Shortest_Path::BFS2(int &nstep)
{
    globalmin=MAX_VALUE_DP;
    globalmin_d = MAX_VALUE_DP;
    globalmin_dt = -1;
    globalmin_ut = NT;

    double tdis1,tdis2;
    int tdis;
    int flag=EstimateXY(aimx,aimy,aimt,tdis1,tdis2,tdis);
    F[NT-1][Ex][Ey]=(flag==FLAG_DANGER?MAX_VALUE_DP:tdis);
    flag=EstimateXY(orgx,orgy,orgt,tdis1,tdis2,tdis);
    F[0][Bx][By]=(flag==FLAG_DANGER?MAX_VALUE_DP:tdis);//�����ʼ�����������⣿Ӧ��ֻ��һ�����ܡ���

    int NQ=NX*NY;
    int ***Q;
    Q=new int**[2];

    for(int k=0; k<2; k++)
    {
        Q[k]=new int*[2];
        for(int xy=0; xy<2; xy++)
        {
            Q[k][xy]=new int[NX*NY];
        }
    }

    int pr[2][3];//begin end l
    int q=0,t=1;

    pr[0][0]=0; pr[0][1]=0; pr[0][2]=0; //begin end l
    pr[1][0]=0; pr[1][1]=0; pr[1][2]=NT-1;

    Q[q][0][pr[q][0]]=Bx; Q[q][1][pr[q][0]]=By;
    Q[t][0][pr[t][0]]=Ex; Q[t][1][pr[t][0]]=Ey;

    //FILE *fp=fopen("rec.txt","w");
    //printf("NT=%d\n",NT);
    while(pr[0][2]+1<=pr[1][2])
    {
        if(pr[0][1]-pr[0][0]<pr[1][1]-pr[1][0])
        {
            q=0; t=1;
        }
        else
        {
            q=1; t=0;
        }

        if(pr[0][1]-pr[0][0]<0 || pr[1][1]-pr[1][0]<0)
        {
            LOG(ERROR)<<"Can not expand anymore, no solutions";
            break;
        }

        //��չq��������е�Ԫ��
        int jn=pr[q][1]+1;
        for(int j=pr[q][0]; j<jn; j++)
        {
            int x=Q[q][0][j%NQ];
            int y=Q[q][1][j%NQ];
            int t=pr[q][2];

            for(int ix=0; ix<NX; ix++){
                for(int iy=0; iy<NY; iy++){
                    if (F[t][ix][iy]<MAX_VALUE_DP){
                        //fprintf(fp,"0 ");
                    }
                    else{
                        //fprintf(fp,"x ");
                    }
                }
              //  fprintf(fp,"\n");
            }
            //fprintf(fp,"\n");

            bool found=false;

            for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
            {
                int nx=x+D[k][0];
                int ny=y+D[k][1];
                int nt=t-(q*2-1);

                if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;

                double dis1,dis2;
                int dis;
                double cx,cy,ct;
                cx=((double)nx)*step_x+range_xmin;
                cy=((double)ny)*step_y+range_ymin;
                ct=((double)nt)*step_t+orgt;
                //��ʵλ�ã���ΪEstimateXY���������.
                if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
                {
                    continue;
                }
                //q*2-1:-1,1... q=0,
                if(k==0)
                {
                    dis=0;
                }
                else
                {
                    dis=1.0;
                }

                if(pr[0][2]+1==pr[1][2])
                {
                    if(globalmin>F[nt][nx][ny]+F[t][x][y])
                    {
                        globalmin=F[nt][nx][ny]+F[t][x][y];
                        globalmin_d = MAX_VALUE_DP;
                        if(nt>t)
                        {
                            globalmin_dt=t;globalmin_dx=x;globalmin_dy=y;
                            globalmin_ut=nt;globalmin_ux=nx;globalmin_uy=ny;
                        }
                        else if(nt<t)
                        {
                            globalmin_dt=nt; globalmin_dx=nx;globalmin_dy=ny;
                            globalmin_ut=t; globalmin_ux=x; globalmin_uy=y;
                        }
                        else
                        {
                            LOG(ERROR)<<"impossible error occurs! Logic dump!";
                            break;
                        }
                    }
                }
                else if(F[nt][nx][ny]>F[t][x][y]+dis)
                {
                    if(F[nt][nx][ny]>=MAX_VALUE_DP)
                    {
                        pr[q][1]++;
                        Q[q][0][pr[q][1]%NQ]=nx;
                        Q[q][1][pr[q][1]%NQ]=ny;
                    }
                    F[nt][nx][ny]=F[t][x][y]+dis;//ֻ�������һ�Ρ�
                    if (nt > t && (F[nt][nx][ny] < globalmin_d || nt > globalmin_dt)){
                        globalmin_d = F[nt][nx][ny];
                        globalmin_dt = nt;
                        globalmin_dx = nx;
                        globalmin_dy = ny;
                    }
                }
            }

        }
        pr[q][0]=jn;//�����˫������������
        pr[q][2]+=-(q*2-1);
    }
    //fclose(fp);
    if(globalmin<MAX_VALUE_DP)
    {//found best solutions.
        nstep=NT;
    }
    else
    {
        LOG(ERROR)<< "No solutions found! replace with closest path";
        nstep=globalmin_dt+1;
    }

    for(int k=0; k<2; k++)
        {
            //Q[k]=new int*[2];
            for(int xy=0; xy<2; xy++)
            {
               // Q[k][xy]=new int[NX*NY];
                delete []Q[k][xy];
            }
            delete []Q[k];
        }
    delete []Q;
    Q=NULL;

    return 0;
}



int G_Shortest_Path::GetPath_BFS(double ** path)
{
	int **bestpath=new int*[NT];
	for(int t=0; t<NT; t++)
		bestpath[t]=new int[3];			
			
	bestpath[globalmin_dt][0]=globalmin_dx;
	bestpath[globalmin_dt][1]=globalmin_dy;
	bestpath[globalmin_dt][2]=globalmin_dt;			

	bestpath[globalmin_ut][0]=globalmin_ux;
	bestpath[globalmin_ut][1]=globalmin_uy;
	bestpath[globalmin_ut][2]=globalmin_ut;

	for(int t=globalmin_dt; t>0; t--)
	{
		int x=bestpath[t][0];
		int y=bestpath[t][1];

		double dis1,dis2;
		int dis;
		double cx,cy,ct;
		cx=((double)x)*step_x+range_xmin;
		cy=((double)y)*step_y+range_ymin;
		ct=((double)t)*step_t+orgt;
				
		if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
		{
			continue;
		}			

		bool found=false;		
		for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
		{
			int nx=x+D[k][0];
			int ny=y+D[k][1];
			int nt=t-1;
			if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;		

			if(k==0)
				{
					dis=0;
				}
				else
				{
					dis=1.0;
				}

			if(F[t][x][y]==F[nt][nx][ny]+dis)
			{											
				bestpath[nt][0]=nx;
				bestpath[nt][1]=ny;
				bestpath[nt][2]=nt;
				found=true;
				break;
			}				
		}
		if(!found)
		{
			printf("Search error! path can not match!\n");
			break;
		}
	}

	for(int t=globalmin_ut; t<NT-1; t++)
	{
		int x=bestpath[t][0];
		int y=bestpath[t][1];

		double dis1,dis2;
		int dis;
		double cx,cy,ct;
		cx=((double)x)*step_x+range_xmin;
		cy=((double)y)*step_y+range_ymin;
		ct=((double)t)*step_t+orgt;
				
		if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
		{
			continue;
		}				
			
		bool found=false;
		for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
		{
			int nx=x+D[k][0];
			int ny=y+D[k][1];
			int nt=t+1;

			if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;
			
			if(k==0)
			{
				dis=0;
			}
			else
			{
				dis=1.0;
			}

			if(F[t][x][y]==F[nt][nx][ny]+dis)
			{											
				bestpath[nt][0]=nx;
				bestpath[nt][1]=ny;
				bestpath[nt][2]=nt;
				found=true;
				break;
			}				
		}
		if(!found)
		{
			printf("Search error! path can not match!\n");
			break;
		}
	}

	if(bestpath[0][0]==Bx && bestpath[0][1]==By && bestpath[0][2]==0
		&&bestpath[NT-1][0]==Ex && bestpath[NT-1][1]==Ey && bestpath[NT-1][2]==NT-1)
	{
		//�����Outputһ��·����Ϣ����
		//FILE *fp=fopen("danger.txt","w");
		for(int i=0; i<NT; i++)
		{	
			path[i][0]=((double)bestpath[i][0])*step_x+range_xmin;
			path[i][1]=((double)bestpath[i][1])*step_y+range_ymin;
			path[i][2]=((double)bestpath[i][2])*step_t+orgt;

			int x=bestpath[i][0];
			int y=bestpath[i][1];

			double dis1,dis2;
			int dis;
			double cx,cy,ct;
			cx=((double)x)*step_x+range_xmin;
			cy=((double)y)*step_y+range_ymin;
			ct=((double)i)*step_t+orgt;
				
			EstimateXY(cx,cy,ct,dis1,dis2,dis);
			//fprintf(fp,"%d %d %d %d\n",i,(int)(dis1*100.0*ramda),(int)(dis2*100.0*(1.0-ramda)),dis);


			printf("step:%d, goto (%d,%d)\n",i,bestpath[i][0],bestpath[i][1]);
		}
		//fclose(fp);
	}
	else
	{
		printf("No Path Found! Error!\n");
	}

	for(int t=0; t<NT; t++)
	{
		delete []bestpath[t];
	}
	delete []bestpath;

	return 0;
}

int G_Shortest_Path::GetPath_BFS2(double ** path)
{
    int **bestpath=new int*[NT];
    for(int t=0; t<NT; t++){
        bestpath[t]=new int[3];
        bestpath[t][0]=0;
        bestpath[t][1]=0;
        bestpath[t][2]=0;
    }

    if (globalmin_dt >= 0 && globalmin_d < NT){
        bestpath[globalmin_dt][0]=globalmin_dx;
        bestpath[globalmin_dt][1]=globalmin_dy;
        bestpath[globalmin_dt][2]=globalmin_dt;
    }
    if(globalmin_ut < NT && globalmin_ut >=0){
        bestpath[globalmin_ut][0]=globalmin_ux;
        bestpath[globalmin_ut][1]=globalmin_uy;
        bestpath[globalmin_ut][2]=globalmin_ut;
    }
    for(int t=globalmin_dt; t>0; t--)
    {
        int x=bestpath[t][0];
        int y=bestpath[t][1];

        double dis1,dis2;
        int dis;
        double cx,cy,ct;
        cx=((double)x)*step_x+range_xmin;
        cy=((double)y)*step_y+range_ymin;
        ct=((double)t)*step_t+orgt;

        if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
        {
            continue;
        }

        bool found=false;
        for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
        {
            int nx=x+D[k][0];
            int ny=y+D[k][1];
            int nt=t-1;
            if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;

            if(k==0)
                {
                    dis=0;
                }
                else
                {
                    dis=1.0;
                }

            if(F[t][x][y]==F[nt][nx][ny]+dis)
            {
                bestpath[nt][0]=nx;
                bestpath[nt][1]=ny;
                bestpath[nt][2]=nt;
                found=true;
                break;
            }
        }
        if(!found)
        {
            printf("Search error! path can not match!\n");
            break;
        }
    }

    for(int t=globalmin_ut; t<NT-1; t++)
    {
        int x=bestpath[t][0];
        int y=bestpath[t][1];

        double dis1,dis2;
        int dis;
        double cx,cy,ct;
        cx=((double)x)*step_x+range_xmin;
        cy=((double)y)*step_y+range_ymin;
        ct=((double)t)*step_t+orgt;

        if(EstimateXY(cx,cy,ct,dis1,dis2,dis)==FLAG_DANGER)
        {
            continue;
        }

        bool found=false;
        for(int k=0; k<NE; k++)//for(int k=1; k<NE; k++)
        {
            int nx=x+D[k][0];
            int ny=y+D[k][1];
            int nt=t+1;

            if(nx<0 || nx>=NX || ny<0 || ny>=NY) continue;

            if(k==0)
            {
                dis=0;
            }
            else
            {
                dis=1.0;
            }

            if(F[t][x][y]==F[nt][nx][ny]+dis)
            {
                bestpath[nt][0]=nx;
                bestpath[nt][1]=ny;
                bestpath[nt][2]=nt;
                found=true;
                break;
            }
        }
        if(!found)
        {
            printf("Search error! path can not match!\n");
            break;
        }
    }


    if(bestpath[0][0]==Bx && bestpath[0][1]==By && bestpath[0][2]==0
        &&bestpath[NT-1][0]==Ex && bestpath[NT-1][1]==Ey && bestpath[NT-1][2]==NT-1)
    {
        //�����Outputһ��·����Ϣ����
        //FILE *fp=fopen("danger.txt","w");
        for(int i=0; i<NT; i++)
        {
            path[i][0]=((double)bestpath[i][0])*step_x+range_xmin;
            path[i][1]=((double)bestpath[i][1])*step_y+range_ymin;
            path[i][2]=((double)bestpath[i][2])*step_t+orgt;

            int x=bestpath[i][0];
            int y=bestpath[i][1];

            double dis1,dis2;
            int dis;
            double cx,cy,ct;
            cx=((double)x)*step_x+range_xmin;
            cy=((double)y)*step_y+range_ymin;
            ct=((double)i)*step_t+orgt;

            EstimateXY(cx,cy,ct,dis1,dis2,dis);
            //fprintf(fp,"%d %d %d %d\n",i,(int)(dis1*100.0*ramda),(int)(dis2*100.0*(1.0-ramda)),dis);


            printf("step:%d, goto (%d,%d)\n",i,bestpath[i][0],bestpath[i][1]);
        }
        //fclose(fp);
    }
    else
    {
        printf("No Path Found! Error! replace with closed path\n");
        for(int i=0; i<=globalmin_dt; i++)
        {
            path[i][0]=((double)bestpath[i][0])*step_x+range_xmin;
            path[i][1]=((double)bestpath[i][1])*step_y+range_ymin;
            path[i][2]=((double)bestpath[i][2])*step_t+orgt;
        }
    }

    for(int t=0; t<NT; t++)
    {
        delete []bestpath[t];
    }
    delete []bestpath;

    return 0;
}
