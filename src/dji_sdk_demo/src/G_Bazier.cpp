/*
 * GBazier.cpp
 *
 *  Created on: 2017年5月27日
 *      Author: Hualin He
 */

#include "G_Bazier.h"
#include <math.h>
#include<vector>
#include<iostream>
using namespace std;

#define IARC_POSITION IARC_POSITION

namespace G_Bazier {

G_Bazier::G_Bazier() {
	// TODO Auto-generated constructor stub

}

G_Bazier::~G_Bazier() {
	// TODO Auto-generated destructor stub
}

//三次贝塞尔曲线
float bezier3funcX(float uu,IARC_POSITION *controlP){
   float part0 = controlP[0].x * uu * uu * uu;
   float part1 = 3 * controlP[1].x * uu * uu * (1 - uu);
   float part2 = 3 * controlP[2].x * uu * (1 - uu) * (1 - uu);
   float part3 = controlP[3].x * (1 - uu) * (1 - uu) * (1 - uu);
   return part0 + part1 + part2 + part3;
}
float bezier3funcY(float uu,IARC_POSITION *controlP){
   float part0 = controlP[0].y * uu * uu * uu;
   float part1 = 3 * controlP[1].y * uu * uu * (1 - uu);
   float part2 = 3 * controlP[2].y * uu * (1 - uu) * (1 - uu);
   float part3 = controlP[3].y * (1 - uu) * (1 - uu) * (1 - uu);
   return part0 + part1 + part2 + part3;
}


int BuildControlPoint(IARC_POSITION *originPoint,int originCount, vector<IARC_POSITION> &curvePoint){

	double x0, y0, x1, y1, x2, y2, x3, y3;
	double ctrl1_x, ctrl1_y, ctrl2_x, ctrl2_y;
	double smooth_coef;
	if(originCount < 4) return 0;

	x0 = originPoint[0].x; y0 = originPoint[0].y;
	x1 = originPoint[1].x; y1 = originPoint[1].y;
	x2 = originPoint[2].x; y2 = originPoint[2].y;
	x3 = originPoint[3].x; y3 = originPoint[3].y;
	smooth_coef = 1.0;
	// Assume we need to calculate the control
	// points between (x1, y1) and (x2, y2).
	// Then x0,y0 - the previous vertex,
	//      x3,y3 - the next one.

	double xc1 = (x0 + x1) / 2.0;
	double yc1 = (y0 + y1) / 2.0;
	double xc2 = (x1 + x2) / 2.0;
	double yc2 = (y1 + y2) / 2.0;
	double xc3 = (x2 + x3) / 2.0;
	double yc3 = (y2 + y3) / 2.0;

	double len1 = sqrt((x1- x0) * (x1- x0) + (y1-y0) * (y1-y0));
	double len2 = sqrt((x2- x1) * (x2- x1) + (y2-y1) * (y2-y1));
	double len3 = sqrt((x3-x2) * (x3- x2) + (y3-y2) * (y3-y2));

	double k1 = len1 / (len1 + len2);
	double k2 = len2 / (len2 + len3);

	double xm1 = xc1 + (xc2 - xc1) * k1;
	double ym1 = yc1 + (yc2 - yc1) * k1;

	double xm2 = xc2 + (xc3 - xc2) * k2;
	double ym2 = yc2 + (yc3 - yc2) * k2;

	// Resulting control points. Here smooth_value is mentioned
	// above coefficient K whose value should be in range [0...1].
	ctrl1_x = xm1 + (xc1 - xm1) * smooth_coef + x1 - xm1;
	ctrl1_y = ym1 + (yc1 - ym1) * smooth_coef + y1 - ym1;

	ctrl2_x = xm1 + (xc2 - xm1) * smooth_coef + x1 - xm1;
	ctrl2_y = ym1 + (yc2 - ym1) * smooth_coef + y1 - ym1;

	float u = 1;
	IARC_POSITION controlPoint[4];
	controlPoint[0].x = x0;  controlPoint[0].y = y0;
	controlPoint[1].x = ctrl1_x;  controlPoint[1].y = ctrl1_y;
	controlPoint[2].x = ctrl2_x;  controlPoint[2].y = ctrl2_y;
	controlPoint[3].x = x2;  controlPoint[3].y = y2;

	while(u >= 0){
		double px = bezier3funcX(u,controlPoint);
		double py = bezier3funcY(u,controlPoint);
		//u的步长决定曲线的疏密
		u -= 0.005;
		IARC_POSITION tempP;
		tempP.x = px;
		tempP.y = py;
		//存入曲线点
		curvePoint.push_back(tempP);
	}

return 0;

}

void createCurve(IARC_POSITION *originPoint,int originCount, vector<IARC_POSITION> &curvePoint){
    //控制点收缩系数 ，经调试0.6较好，IARC_POSITION是<a href="http://lib.csdn.net/base/opencv" class='replace_word' title="OpenCV知识库" target='_blank' style='color:#df3434; font-weight:bold;'>OpenCV</a>的，可自行定义结构体(x,y)
    float scale = 0.6;
    IARC_POSITION midpoints[originCount];
    //生成中点
    for(int i = 0 ;i < originCount ; i++){
        int nexti = (i + 1) % originCount;
        midpoints[i].x = (originPoint[i].x + originPoint[nexti].x)/2.0;
        midpoints[i].y = (originPoint[i].y + originPoint[nexti].y)/2.0;
    }

    //平移中点
    IARC_POSITION extrapoints[2 * originCount];
    for(int i = 0 ;i < originCount ; i++){
         int nexti = (i + 1) % originCount;
         int backi = (i + originCount - 1) % originCount;
         IARC_POSITION midinmid;
         midinmid.x = (midpoints[i].x + midpoints[backi].x)/2.0;
         midinmid.y = (midpoints[i].y + midpoints[backi].y)/2.0;
         double offsetx = originPoint[i].x - midinmid.x;
         double offsety = originPoint[i].y - midinmid.y;
         int extraindex = 2 * i;
         extrapoints[extraindex].x = midpoints[backi].x + offsetx;
         extrapoints[extraindex].y = midpoints[backi].y + offsety;
         //朝 originPoint[i]方向收缩
         double addx = (extrapoints[extraindex].x - originPoint[i].x) * scale;
         double addy = (extrapoints[extraindex].y - originPoint[i].y) * scale;
         extrapoints[extraindex].x = originPoint[i].x + addx;
         extrapoints[extraindex].y = originPoint[i].y + addy;

         int extranexti = (extraindex + 1)%(2 * originCount);
         extrapoints[extranexti].x = midpoints[i].x + offsetx;
         extrapoints[extranexti].y = midpoints[i].y + offsety;
         //朝 originPoint[i]方向收缩
         addx = (extrapoints[extranexti].x - originPoint[i].x) * scale;
         addy = (extrapoints[extranexti].y - originPoint[i].y) * scale;
         extrapoints[extranexti].x = originPoint[i].x + addx;
         extrapoints[extranexti].y = originPoint[i].y + addy;

         cout<< extrapoints[extranexti].x<<" "<<extrapoints[extranexti].y<<endl;

    }

    IARC_POSITION controlPoint[4];
    curvePoint.clear();
    //生成4控制点，产生贝塞尔曲线
    for(int i = 0 ;i < originCount ; i++){
           controlPoint[0] = originPoint[i];
           int extraindex = 2 * i;
           controlPoint[1] = extrapoints[extraindex + 1];
           int extranexti = (extraindex + 2) % (2 * originCount);
           controlPoint[2] = extrapoints[extranexti];
           int nexti = (i + 1) % originCount;
           controlPoint[3] = originPoint[nexti];
           float u = 1;
           cout<< controlPoint[1].x << " "<<controlPoint[1].y<< " "<<controlPoint[2].x<<" "<<controlPoint[2].y<<endl;

           while(u >= 0){
               double px = bezier3funcX(u,controlPoint);
               double py = bezier3funcY(u,controlPoint);
               //u的步长决定曲线的疏密
               u -= 0.005;
               IARC_POSITION tempP;
               tempP.x = px;
               tempP.y = py;
               //存入曲线点
               curvePoint.push_back(tempP);
           }
    }
}

void createCurve2(IARC_POSITION *originPoint, int beg, int end, std::vector<IARC_POSITION> &curvePoint, double du){
    //控制点收缩系数 ，经调试0.6较好，IARC_POSITION是<a href="http://lib.csdn.net/base/opencv" class='replace_word' title="OpenCV知识库" target='_blank' style='color:#df3434; font-weight:bold;'>OpenCV</a>的，可自行定义结构体(x,y)
    float scale = 0.6;
    int originCount = end - beg + 1;
    originPoint = originPoint + beg;
    IARC_POSITION midpoints[originCount];
    //生成中点
    for(int i = 0 ;i < originCount - 1 ; i++){
        int nexti = (i + 1) ;
        midpoints[i].x = (originPoint[i].x + originPoint[nexti].x)/2.0;
        midpoints[i].y = (originPoint[i].y + originPoint[nexti].y)/2.0;
    }

    //平移中点
    IARC_POSITION extrapoints[2 * (originCount - 1)];
    for(int i = 1 ;i < originCount-1 ; i++){
         int nexti = (i + 1) ;
         int backi = (i - 1);
         IARC_POSITION midinmid;
         midinmid.x = (midpoints[i].x + midpoints[backi].x)/2.0;
         midinmid.y = (midpoints[i].y + midpoints[backi].y)/2.0;
         double offsetx = originPoint[i].x - midinmid.x;
         double offsety = originPoint[i].y - midinmid.y;
         int extraindex = 2 * i;
         extrapoints[extraindex].x = midpoints[backi].x + offsetx;
         extrapoints[extraindex].y = midpoints[backi].y + offsety;
         //朝 originPoint[i]方向收缩
         double addx = (extrapoints[extraindex].x - originPoint[i].x) * scale;
         double addy = (extrapoints[extraindex].y - originPoint[i].y) * scale;
         extrapoints[extraindex].x = originPoint[i].x + addx;
         extrapoints[extraindex].y = originPoint[i].y + addy;

         int extranexti = (extraindex + 1);
         extrapoints[extranexti].x = midpoints[i].x + offsetx;
         extrapoints[extranexti].y = midpoints[i].y + offsety;
         //朝 originPoint[i]方向收缩
         addx = (extrapoints[extranexti].x - originPoint[i].x) * scale;
         addy = (extrapoints[extranexti].y - originPoint[i].y) * scale;
         extrapoints[extranexti].x = originPoint[i].x + addx;
         extrapoints[extranexti].y = originPoint[i].y + addy;

    }

    IARC_POSITION controlPoint[4];
    curvePoint.clear();
    //生成4控制点，产生贝塞尔曲线
    for(int i = 1 ;i < originCount - 2; i++){
           controlPoint[0] = originPoint[i];
           int extraindex = 2 * i;
           controlPoint[1] = extrapoints[extraindex + 1];
           int extranexti = (extraindex + 2);
           controlPoint[2] = extrapoints[extranexti];
           int nexti = (i + 1) ;
           controlPoint[3] = originPoint[nexti];
           float u = 1;

           while(u >= 0){
               double px = bezier3funcX(u,controlPoint);
               double py = bezier3funcY(u,controlPoint);
               //u的步长决定曲线的疏密
               u -= du;
               IARC_POSITION tempP;
               tempP.x = px;
               tempP.y = py;
               //存入曲线点
               curvePoint.push_back(tempP);
           }
    }
}

void createCurve3ok(const std::vector<IARC_POSITION> &originPoint, int beg, int end, std::vector<IARC_POSITION> &curvePoint, double du){
    //控制点收缩系数 ，经调试0.6较好，IARC_POSITION是<a href="http://lib.csdn.net/base/opencv" class='replace_word' title="OpenCV知识库" target='_blank' style='color:#df3434; font-weight:bold;'>OpenCV</a>的，可自行定义结构体(x,y)
    float scale = 0.6;
    int originCount = end - beg + 1;
    //originPoint = originPoint + beg;
    IARC_POSITION midpoints[originCount];
    //生成中点
    for(int i = 0 ;i < originCount - 1 ; i++){
        int nexti = (i + 1) ;
        midpoints[i].x = (originPoint[beg+i].x + originPoint[beg+nexti].x)/2.0;
        midpoints[i].y = (originPoint[beg+i].y + originPoint[beg+nexti].y)/2.0;
    }

    //平移中点
    IARC_POSITION extrapoints[2 * (originCount - 1)];
    for(int i = 1 ;i < originCount-1 ; i++){
         int nexti = (i + 1) ;
         int backi = (i - 1);
         IARC_POSITION midinmid;
         midinmid.x = (midpoints[i].x + midpoints[backi].x)/2.0;
         midinmid.y = (midpoints[i].y + midpoints[backi].y)/2.0;
         double offsetx = originPoint[beg+i].x - midinmid.x;
         double offsety = originPoint[beg+i].y - midinmid.y;
         int extraindex = 2 * i;
         extrapoints[extraindex].x = midpoints[backi].x + offsetx;
         extrapoints[extraindex].y = midpoints[backi].y + offsety;
         //朝 originPoint[beg+i]方向收缩
         double addx = (extrapoints[extraindex].x - originPoint[beg+i].x) * scale;
         double addy = (extrapoints[extraindex].y - originPoint[beg+i].y) * scale;
         extrapoints[extraindex].x = originPoint[beg+i].x + addx;
         extrapoints[extraindex].y = originPoint[beg+i].y + addy;

         int extranexti = (extraindex + 1);
         extrapoints[extranexti].x = midpoints[i].x + offsetx;
         extrapoints[extranexti].y = midpoints[i].y + offsety;
         //朝 originPoint[beg+i]方向收缩
         addx = (extrapoints[extranexti].x - originPoint[beg+i].x) * scale;
         addy = (extrapoints[extranexti].y - originPoint[beg+i].y) * scale;
         extrapoints[extranexti].x = originPoint[beg+i].x + addx;
         extrapoints[extranexti].y = originPoint[beg+i].y + addy;

    }

    IARC_POSITION controlPoint[4];
    curvePoint.clear();
    //生成4控制点，产生贝塞尔曲线
    for(int i = 1 ;i < originCount - 2; i++){
           controlPoint[0] = originPoint[beg+i];
           int extraindex = 2 * i;
           controlPoint[1] = extrapoints[extraindex + 1];
           int extranexti = (extraindex + 2);
           controlPoint[2] = extrapoints[extranexti];
           int nexti = (i + 1) ;
           controlPoint[3] = originPoint[beg+nexti];
           float u = 1;

           while(u >= 0){
               double px = bezier3funcX(u,controlPoint);
               double py = bezier3funcY(u,controlPoint);
               //u的步长决定曲线的疏密
               u -= du;
               IARC_POSITION tempP;
               tempP.x = px;
               tempP.y = py;
               //存入曲线点
               curvePoint.push_back(tempP);
           }
    }
}


} /* namespace G_Bazier */
