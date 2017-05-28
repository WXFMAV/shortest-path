/*
 * GControllerPID.cpp
 *
 *  Created on: 2017年4月21日
 *      Author: Hualin He
 */
#include <glog/logging.h>
#include "G_Controller_PID.h"
using namespace Eigen;

G_Controller_PID::G_Controller_PID() {
    // TODO Auto-generated constructor stub
    _status = OK;
}

G_Controller_PID::~G_Controller_PID() {
    // TODO Auto-generated destructor stub
}

int G_Controller_PID::init()
{
    _status = OK;
    _kp = MatrixXd::Zero(3,1) ; _kp = _kp.array()+1.5;
    _kd = MatrixXd::Zero(3,1);  _kd = _kd.array()+0.2;
    _err = MatrixXd::Zero(3,1);
    _err_last = MatrixXd::Zero(3,1);
        
    _u_limit.setZero(3,1);
    _u_limit[0] = 4.0;
    _u_limit[1] = 4.0;
    _u_limit[2] = 2.0;
    return 0;
}
Eigen::VectorXd G_Controller_PID::get_control()
{
//    LOG(INFO) << "_kp=" << _kp.transpose();
    LOG(INFO) << "e=" << (_ref-_state).transpose();
    _err_last = _err;
    _err = (_ref - _state).head(3);        
    _u = _kp.array() * _err.array() + _kd.array() * _err_last.array();    
    limit_u();
    LOG(INFO) << "_u=" <<_u.transpose();
    
    return _u;
}
