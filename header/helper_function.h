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


typedef enum{
    KL,
    CLL,
    CLR,
    PCLL,
    PCLR
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
const double timestep_ = 0.02;

// max velocity
const double max_vel_ = 49.5;
const double max_acc_ = 9.5 * timestep_;
const double max_jerk_ = 9.5 * timestep_;

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

double dist(double x1, double y1, double x2, double y2);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


#endif /* helper_function_h */
