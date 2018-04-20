#ifndef LATLONGTOUTM_H
#define LATLONGTOUTM_H

#include <cmath>
#include <ros/ros.h>
using namespace std;

struct UTM
{
	bool hemi; // 1: southern hemisphere; 0: northern hemisphere
	int zone;
	double x;
	double y;
};

UTM LatLon2Utm(double Lat, double Lon);

#endif  // LATLONGTOUTM_H
