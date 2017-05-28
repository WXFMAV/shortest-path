/*
 * GControllerLQR.cpp
 *
 *  Created on: 2017年4月21日
 *      Author: Hualin He
 */

#include "G_Controller_LQR.h"
#include "common.h"
#include <glog/logging.h>
using namespace Eigen;

#define m  PARAM::m
#define n  PARAM::n
#define const_tf PARAM::const_tf
#define filename  PARAM::file_name_lqr_plist

G_Controller_LQR::G_Controller_LQR() {
    // TODO Auto-generated constructor stub
    _status = ERROR;
    _tf = 0.0;
    _ptm = 0;
    _P_list.clear();
    _P_timespace.clear();
}

G_Controller_LQR::~G_Controller_LQR() {
    // TODO Auto-generated destructor stub
}

int G_Controller_LQR::init()
{
    _status = OK;
    //从一个文件中打开，读取Ｐ的列表信息..
    FILE *fp = NULL;
    fp = fopen(filename.c_str(),"r");
    if( fp == NULL ){
        LOG(ERROR) << "file open error: " << filename.c_str();
        return -1;
    }
    LOG(INFO) << "file "<<filename.c_str()<<" opened!";

    int tn,tm;
    char str[100];
    fscanf(fp,"%d %d %s\n",&tn,&tm,str);
    if (tn != m || tm != 1){
         LOG(ERROR)  << "dimension of umax matrix in file "<<filename.c_str()<< " is not matched!";
         return -1;
    }
    _u_limit.setZero(m,1);
    for (int i = 0; i < m; i++){
         fscanf(fp, "%lf", &_u_limit(i,0));
    }
    fscanf(fp, "%d %d %s\n", &tn, &tm, str);
    if(tn != n || tm != m){
               LOG(ERROR) << "dimension of B matrix in file "<<filename.c_str() <<" is not matched!";
                return -1;
    }
    _B.setZero(n,m);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            fscanf(fp, "%lf", &_B(i,j));
    fscanf(fp, "%d %d %s\n", &tn, &tm, str);

    if (tn != m || tm != m){
       LOG(ERROR) <<"dimension of R matrix in file "<<filename.c_str() <<" is not matched!";
       return -1;
    }
    _R.setZero(m, m);
    for (int i = 0; i < m; i++)
           for (int j = 0; j < m; j++)
                   fscanf(fp, "%lf", &_R(i,j));

    fscanf(fp, "%d %d %s\n", &_ptm, &tn, str);
    if ( tn != n * n + 1){
        LOG(ERROR) <<"dimension of P matrix in file "<< filename.c_str() <<" is not matched!";
        return -1;
    }
    //mapStudent.insert(map<int, string>::value_type (1, “student_one”));
    _P_timespace.clear();
    _P_list.clear();
    for (int k = 0; k < _ptm; k++){
        double tmp;
        fscanf(fp,"%lf",&tmp);
        _P_timespace.push_back(tmp);

        MatrixXd temp(n,n);
        for (int i = 0; i < n;  i++)
            for (int  j = 0; j < n; j++){
                fscanf(fp, "%lf", &temp(i,j));
            }
        _P_list.push_back(temp);
    }
    fclose(fp);
    LOG(INFO) << "[OK] P matrix list imported!";

    //_P = _P_list[locate_pos_tf(0.33)];//330ms is the common tf occurer time change to this..
    _tf = const_tf;

    return 0;
}
Eigen::MatrixXd G_Controller_LQR::get_P_at_tf(double tf)
{
    return _P_list[locate_pos_tf(tf)];
}
int G_Controller_LQR::locate_pos_tf(double tf)
{
    return lower_bound(_P_timespace.begin(), _P_timespace.end(), tf)
            - _P_timespace.begin();
}

int G_Controller_LQR::set_tf(double tf)
{
    _tf = tf;
    return 0;
}
Eigen::VectorXd G_Controller_LQR::get_control()
{
    MatrixXd &P = _P_list[locate_pos_tf(_tf)];
    _u = - _R.inverse() * _B.transpose() * P * (_state);
    limit_u();
    LOG(INFO) << "get control ok";
    LOG(INFO) <<"x="<< _state.transpose() ;
    LOG(INFO) << "u="<<_u.transpose();
    return _u;
}
