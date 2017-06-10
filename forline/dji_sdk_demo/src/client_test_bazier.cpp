#include<iostream>
#include <ros/ros.h>
#include "G_Bazier.h"

using namespace std;
using namespace G_Bazier;

int bazier_main(){

	IARC_POSITION pt[100];
	int n;
	vector<IARC_POSITION> curve;
	FILE* fp = fopen(PARAM::file_name_bazier_in.c_str(),"r");
	fscanf(fp, "%d", &n);
	for(int k = 0; k< n ; k++){
		fscanf(fp, "%lf %lf", &pt[k].x, &pt[k].y);
	}
	fclose(fp);

	FILE *fout = fopen(PARAM::file_name_bazier_out.c_str(),"w");

	/*
	createCurve2(pt, 0, n - 1, curve);
	for(int k = 0; k< curve.size(); k++){
		fprintf(fout, "%.2lf %.2lf\n",curve[k].x, curve[k].y);
	}
*/

	for( int kp = 3; kp < n; kp++){

		createCurve2(pt, kp - 3, kp, curve, 0.02);
		for(int k = 0; k< curve.size(); k++){
			fprintf(fout, "%.2lf %.2lf\n",curve[k].x, curve[k].y + (double)kp *(0.1));
		}
	}


	fclose(fout);

	return 0;
}

int bazier_main2(){

	vector<IARC_POSITION> pt;
	int n;
	vector<IARC_POSITION> curve;
	FILE* fp = fopen(PARAM::file_name_bazier_in.c_str(),"r");
	fscanf(fp, "%d", &n);
	for(int k = 0; k< n ; k++){
		IARC_POSITION tmp;
		fscanf(fp, "%lf %lf", &tmp.x, &tmp.y);
		pt.push_back(tmp);
	}
	fclose(fp);

	FILE *fout = fopen(PARAM::file_name_bazier_out.c_str(),"w");

	/*
	createCurve2(pt, 0, n - 1, curve);
	for(int k = 0; k< curve.size(); k++){
		fprintf(fout, "%.2lf %.2lf\n",curve[k].x, curve[k].y);
	}
*/

	for( int kp = 3; kp < n; kp++){

		createCurve3ok(pt, kp - 3, kp, curve, 0.02);
		for(int k = 0; k< curve.size(); k++){
			fprintf(fout, "%.2lf %.2lf\n",curve[k].x, curve[k].y);
		}
	}


	fclose(fout);

	return 0;
}
// 测试
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flight_control");
    ros::NodeHandle nh;

    arena_set_startnow();

    bazier_main2();

	return 0;
}
