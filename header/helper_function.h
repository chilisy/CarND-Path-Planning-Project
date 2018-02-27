//
//  helper_function.h
//  Path_Planning
//
//  Created by ChenLiheng on 27.02.18.
//

#ifndef helper_function_h
#define helper_function_h

#include <math.h>
#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

double distance(double x1, double y1, double x2, double y2);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


#endif /* helper_function_h */
