/*
 * GControllerPID.h
 *
 *  Created on: 2017年4月21日
 *      Author: exbot
 */

#ifndef DJI_SDK_DEMO_SRC_G_CONTROLLER_PID_H_
#define DJI_SDK_DEMO_SRC_G_CONTROLLER_PID_H_

#include "G_Controller.h"

class G_Controller_PID: public G_Controller {
public:
    G_Controller_PID();
    virtual ~G_Controller_PID();
    int init();
    virtual Eigen::VectorXd get_control();
public:
    Eigen::VectorXd _kp;
    Eigen::VectorXd _kd;
    Eigen::VectorXd _err;
    Eigen::VectorXd _err_last;
};

#endif /* DJI_SDK_DEMO_SRC_G_CONTROLLER_PID_H_ */
