/*
 * GController.h
 *
 *  Created on: 2017年4月21日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_G_CONTROLLER_H_
#define DJI_SDK_DEMO_SRC_G_CONTROLLER_H_
#include "Eigen/Dense"

class G_Controller {
public:
    G_Controller();
    virtual ~G_Controller();
    virtual int init();
    virtual int set_feedback(Eigen::VectorXd state);
    virtual int set_reference(Eigen::VectorXd ref);
    virtual Eigen::VectorXd get_control();
    int limit_u();
public:
    enum{ERROR,OK} _status;
    Eigen::VectorXd _state;
    Eigen::VectorXd _ref;
    Eigen::VectorXd _u;
    Eigen::VectorXd _u_limit;
};

#endif /* DJI_SDK_DEMO_SRC_G_CONTROLLER_H_ */
