/*
 * GBazier.h
 *
 *  Created on: 2017年5月27日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_G_BAZIER_H_
#define DJI_SDK_DEMO_SRC_G_BAZIER_H_

#include "common.h"
#include <vector>

namespace G_Bazier {

class G_Bazier {
public:
	G_Bazier();
	virtual ~G_Bazier();
};
void createCurve3ok(const std::vector<IARC_POSITION> &originPoint, int beg, int end, std::vector<IARC_POSITION> &curvePoint, double du);
void createCurve(IARC_POSITION *originPoint,int originCount, std::vector<IARC_POSITION> &curvePoint);
void createCurve2(IARC_POSITION *originPoint, int beg, int end,std::vector<IARC_POSITION> &curvePoint, double du);
int BuildControlPoint(IARC_POSITION *originPoint,int originCount, std::vector<IARC_POSITION> &curvePoint);

} /* namespace G_Bazier */

#endif /* DJI_SDK_DEMO_SRC_G_BAZIER_H_ */
