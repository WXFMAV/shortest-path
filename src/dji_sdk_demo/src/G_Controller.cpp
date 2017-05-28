/*
 * GController.cpp
 *
 *  Created on: 2017年4月21日
 *      Author: Hualin He
 */

#include "G_Controller.h"
#include <glog/logging.h>
using namespace Eigen;

G_Controller::G_Controller() {
    // TODO Auto-generated constructor stub
    _status = ERROR;
}

G_Controller::~G_Controller() {
    // TODO Auto-generated destructor stub
    _status = ERROR;
}


int G_Controller::init()
{
    _status = ERROR;
    return 0;
}

int G_Controller::set_feedback(Eigen::VectorXd state)
{
    _state = state;
    return 0;
}
int G_Controller::set_reference(Eigen::VectorXd ref)
{
    _ref = ref;
    return 0;
}
Eigen::VectorXd G_Controller::get_control()
{
    return _u;
}
int G_Controller::limit_u()
{
    if (_u_limit.size()==0 ||_u_limit.minCoeff() <= 0.0){
        LOG(ERROR) << "_u_limit not setted!";
        return -1;
    }
    else{
        int sz = _u_limit.size();
        for(int k = 0; k<sz; k++){
            if (_u[k] > _u_limit[k]){
                _u[k] = _u_limit[k];
            }
            else if (_u[k] < -_u_limit[k]){
                _u[k] = -_u_limit[k];
            }
        }
    }
    return 0;
}
