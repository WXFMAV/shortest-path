#include"optimal_controller.h"

OptimalController::OptimalController()
{
    _filename="";    
}
OptimalController::~OptimalController()
{        
}
int OptimalController::init(std::string filename)
{
    _filename=filename;
    FILE *fp=fopen(_filename.c_str());
    if(fp==NULL)
    {
        printf("error! can not open file :%s",_filename.c_str());
        return 0;
    }
    else
    {
        int tn,tm;
        char str[100];     
        fscanf(fp,"%d %d %s\n",&tn,&tm,str);
        if(tn!=m)
        {
            printf("umax matrix in file %s is not matched!\n",_filename);
            return 0;
        }
        for(int i=0; i<m; i++)
        {
            fscanf(fp,"%lf",&umax(i,0));
        }
        
        fscanf(fp,"%d %d %s\n",&tn,&tm,str);
        if(tn!=n || tm!=m)
        {
            printf("B matrix in file %s is not matched!\n",_filename);
            return 0;
        }
        for(int i=0; i<n; i++)
            for(int j=0; j<m; j++)
                fscanf(fp,"%lf",&_B(i,j));
        fscanf(fp,"%d %d %s\n",&tn,&tm,str);
        
        if(tn!=m || tm!=m)
        {
            printf("R matrix in file %s is not matched!\n",_filename);
            return 0;
        }
        for(int i=0; i<m; i++)
            for(int j=0; j<m; j++)
                fscanf(fp,"%lf",&_R(i,j));
        
        fscanf(fp,"%d %d %s\n",&_ptm,&_ptn,str);
        _ptn--;
        _tspace.resize(_ptm);
        _pl.resize(_ptm);        
        
        for(int k=0; k<_ptm; k++)
        {            
            fscanf(fp,"%lf",&_ptspace[k]);
            for(int i=0; i<n; i++)
                for(int j=0; j<n; j++)
                {
                    double tmp;
                    fscanf(fp,"%lf",&tmp);
                    _pl[k](i,j)=tmp;                    
                }            
        }
        fclose(fp);                
    }        
    
    cout<<"B=\n"<<_B<<endl;
    cout<<"R=\n"<<_R<<endl;
    cout<<"P[0]=\n"<<_pl[0]<<endl;
}

int get_control(MatrixXd state_now, double tgo, MatrixXd &u)
{
    double dT=(_ptspace[_ptm-1]-_ptspace[0])/(double)(_ptm-1);
    cout<<"dT="<<dT<<endl;
    int kt=int(tgo/dT);
    Matrix *P=_pl[kt];    
    u=-R.inverse()*B.transpose()*(*P)*state_now;            
        
    for(int i=0; i<m; i++)
    {        
        if(fabs(u(i,0))>_umax(i,0))
        {
            u(i,0)=u(i,0)/fabs(u(i,0))*_umax(i,0);
        }            
    }
    return 0;
}
