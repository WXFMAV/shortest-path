#pragma once
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>

#define TOTAL_OBS 10//����Ǹ��ϰ��
#define MAX_VALUE_DP 99999
#define MAX_VALUE_ASTAR 99999
#define FLAG_DANGER 1
#define FLAG_SAFE 0

#define TYPE_DP 0
#define TYPE_BFS 1
#define TYPE_ASTAR 2


#define NE 9


class G_Shortest_Path{
public:		
	double orgx,orgy,orgt;//��ʼ��
	double aimx,aimy,aimt;//Ŀ���
	double obs[TOTAL_OBS][5];//�ϰ����λ�ã��ٶȣ��Լ����ϰ����Ƿ�ɼ���//�����ϰ�������������޵ġ�	
	double danger_radius;//Σ�վ���
	double step_t,step_x,step_y;//�ռ�Ļ���
	double range_xmin,range_ymin,range_xmax,range_ymax;//����ռ�����ƴ�С
	int options;//ѡ��
	int total_obs;

	int ***F;
	int **Cost;

	int NT,NX,NY;//����
	int Bx,By,Ex,Ey;//ʼ�����յ����ɢ��ʾ
	double ramda,safe_distance2,line_length2,A,B,C,K1,C1;
	int D[NE][2];

private:
	int globalmin,globalmin_d;
	int globalmin_dt,globalmin_dx,globalmin_dy;
	int globalmin_ut,globalmin_ux,globalmin_uy;
public:
	G_Shortest_Path();
	~G_Shortest_Path();
	int Init(double p_orgx,double p_orgy,double p_orgt,double p_aimx,double p_aimy,double p_aimt,double p_obs[TOTAL_OBS][5],double p_danger_radius,
		double p_step_t,double p_step_x,double p_step_y,double p_range_xmin,double p_range_ymin,double p_range_xmax,double p_range_ymax,int p_options,int p_total_obs);
	int Release();
	int DPPath(int &step);//ֱ�ӿ��ǻ�ȡ����·��������ֵ����ͼҲ�����ú������С���
	int GetPath(double ** path);
	int BFS(int &nstep);	
	int BFS2(int &nstep);
	int AStar(int &nstep);
	int GetPath_BFS(double ** path);
	int GetPath_BFS2(double ** path);
	int GetPath_AStar(double ** path);	
	int Hope(int x,int y);
private:
	int EstimateXY(double cx,double cy,double ct,double &dis1,double &dis2,int &dis);
};

struct node{
	int x,y;
	G_Shortest_Path * p;
	bool operator<(const node &a) const{
		printf("%d %d\n",p->Cost[x][y]+p->Hope(x,y),p->Cost[a.x][a.y]+p->Hope(a.x,a.y));
		//return x+p->Hope(x,y)>a.x+p->Hope(a.x,a.y);
		return p->Cost[x][y]+p->Hope(x,y)>p->Cost[a.x][a.y]+p->Hope(a.x,a.y);
	}
};
