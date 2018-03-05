//
//  helper_function.h
//  Path_Planning
//
//  Created by ChenLiheng on 27.02.18.
//

#ifndef helper_function_h
#define helper_function_h

#include <cmath>
#include <vector>

using namespace std;

typedef enum{
    KL, // keep lane
    CLL, // change lane left
    CLR, // change lane right
    PCLL, // prepare change lane left
    PCLR // prepare change lane right
} FSM_State;

typedef struct {
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
} sensor_obj;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// The max s value before wrapping around the track back to 0
const double max_s_ = 6945.554;

const double mph2ms = 1/2.23694;
const double timestep = 0.02;

// max velocity
const double max_vel = 49.5;
const double max_acc = 9.5 * timestep;
const double max_jerk = 9.5 * timestep;

// target lane
const int target_lane = 1;
const double target_d = target_lane*4 + 2;

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

double dist(double x1, double y1, double x2, double y2);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


#endif /* helper_function_h */
