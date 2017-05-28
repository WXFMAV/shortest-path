#include "G_mxBFS.h"
#include "G_Shortest_Path.h"

 void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
 {
	double orgx,orgy,orgt;//��ʼ��
	double aimx,aimy,aimt;//Ŀ���
	double obs[TOTAL_OBS][5];//�ϰ����λ�ã��ٶȣ��Լ����ϰ����Ƿ�ɼ���//�����ϰ�������������޵ġ�
	double danger_radius;//Σ�վ���
	double step_t,step_x,step_y;//�ռ�Ļ���
	double range_xmin,range_ymin,range_xmax,range_ymax;//����ռ�����ƴ�С
	int options;//ѡ��
	int total_obs;

	DWORD t1=GetTickCount();

	if(nlhs!=3 || nrhs!=16)
	{
		printf("Mex Function Param not correct. Check your script!\n");
		return;
	}

	orgx=mxGetScalar(prhs[0]);
	orgy=mxGetScalar(prhs[1]);
	orgt=mxGetScalar(prhs[2]);
	aimx=mxGetScalar(prhs[3]);
	aimy=mxGetScalar(prhs[4]);
	aimt=mxGetScalar(prhs[5]);
	//--------------------------prhs[6]------------
	double *indata;
	int M,N;
	indata=mxGetPr(prhs[6]);
	M=mxGetM(prhs[6]);
	N=mxGetN(prhs[6]);
	if(M>TOTAL_OBS || N!=5)
	{
		printf("error! obs struct is not correct!\n");
		return ;
	}	
	total_obs=M;

	for(int i=0; i<M; i++)
		for(int j=0; j<N; j++)
		{
			obs[i][j]=indata[j*M+i];
		}
	//-------------------------------------------------
	danger_radius=mxGetScalar(prhs[7]);
	step_t=mxGetScalar(prhs[8]);
	step_x=mxGetScalar(prhs[9]);
	step_y=mxGetScalar(prhs[10]);
	range_xmin=mxGetScalar(prhs[11]);
	range_ymin=mxGetScalar(prhs[12]);
	range_xmax=mxGetScalar(prhs[13]);
	range_ymax=mxGetScalar(prhs[14]);
	options=mxGetScalar(prhs[15]);
	//--------------------------------------------16 params at all---------
	//printf("Parma Input Complete!\n");

	int nstep=0;
	double **path=NULL;

	CArena theArena;

	theArena.Init(orgx,orgy,orgt,aimx,aimy,aimt,obs,danger_radius,step_t,step_x,step_y,range_xmin,range_ymin,range_xmax,range_ymax,options,total_obs);
	theArena.BFS(nstep);

	//printf("Dynamic Programming Complete! nstep=%d\n",nstep);
	if(nstep>0)
	{
		path=new double*[nstep];
		for(int i=0; i<nstep; i++)
			path[i]=new double[3];
		
		theArena.GetPath_BFS(path);

		//printf("Path build!\n");
		
		plhs[0]=mxCreateDoubleMatrix(nstep,3,mxREAL);
		double *outdata=mxGetPr(plhs[0]);
		for(int i=0; i<nstep; i++)
		{
			for(int j=0; j<3; j++)
			{
				outdata[j*nstep+i]=path[i][j];				
			}
		}
		for(int i=0; i<nstep; i++)
			delete []path[i];
		delete []path;				
	}
	else
	{
		plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
		*mxGetPr(plhs[0])=0;
	}
	plhs[1]=mxCreateDoubleMatrix(1,1,mxREAL);
	*mxGetPr(plhs[1])=nstep;

	DWORD t2=GetTickCount();
	plhs[2]=mxCreateDoubleMatrix(1,1,mxREAL);
	*mxGetPr(plhs[2])=t2-t1;
	//printf("mexFunc complete!\n");
 }