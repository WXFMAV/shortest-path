/*
 * GControllerLQR.h
 *
 *  Created on: 2017年4月21日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_G_CONTROLLER_LQR_H_
#define DJI_SDK_DEMO_SRC_G_CONTROLLER_LQR_H_

#include "G_Controller.h"
#include "common.h"
#include <vector>
#include <map>
#include <algorithm>
#include <iterator>

class G_Controller_LQR: public G_Controller {
public:
    G_Controller_LQR();
    virtual ~G_Controller_LQR();
    virtual int init();
    virtual Eigen::VectorXd get_control();
    int set_tf(double tf);
private:
    int locate_pos_tf(double tf);
    Eigen::MatrixXd get_P_at_tf(double tf);
public:
    double _tf;
    int _ptm;
    Eigen::MatrixXd _A;
    Eigen::MatrixXd _B;
    Eigen::MatrixXd _C;
    Eigen::MatrixXd _P;
    Eigen::MatrixXd _K;
    Eigen::MatrixXd _F;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _R;
    std::vector<Eigen::MatrixXd> _P_list;
    std::vector<double> _P_timespace;
};

#endif /* DJI_SDK_DEMO_SRC_G_CONTROLLER_LQR_H_ */
