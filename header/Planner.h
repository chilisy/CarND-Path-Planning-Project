//
//  Planner.h
//  Path_Planning
//
//  Created by ChenLiheng on 25.02.18.
//

#ifndef Planner_h
#define Planner_h

#include "helper_function.h"
#include "spline.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"
#include "Cost.h"

using namespace std;

#define COUNT_GEN_PTS 50
#define ID_DEFAULT 9999
#define CAR_LENGTH 8
#define SPACE_2_CAR_AHEAD 30

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

class TrajectoryPlanner{
    
    // variables:
    // telemetry
    double current_x_;
    double current_y_;
    double current_s_;
    double current_d_;
    double current_yaw_;
    double current_speed_;
    
    // map points
    vector<double> map_x_;
    vector<double> map_y_;
    vector<double> map_s_;
    vector<double> map_dx_;
    vector<double> map_dy_;
    
    // previous points
    vector<double> prev_x_;
    vector<double> prev_y_;
    
    // sensor objects
    vector<sensor_obj> objs_;
    int car_ahead_id_;
    int car_left_id_;
    int car_right_id_;
    
    // FSM
    FSM_State current_state_;
    vector<FSM_State> possible_states_;
    
    // cars
    
    
    // The max s value before wrapping around the track back to 0
    double max_s_ = 6945.554;
    
    // target velocity
    double vel_ = 0.0;
    
    // start in middle lane
    int lane_ = 1;
    
    double mph2ms = 1/2.23694;
    double timestep_ = 0.02;
    
    // max velocity
    double max_vel_ = 49.5;
    double max_acc_ = 9.5 * timestep_;
    double max_jerk_ = 9.5 * timestep_;
    
    // methods:
    void calculateVelocity();
    
    void driveLane(int lane);
    
    void getSuccessorStates();
    
    void chooseNextState();
    
    void transform2CarCoord(vector<double> &pts_car_x, vector<double> &pts_car_y, double ref_x, double ref_y, double ref_yaw, vector<double> pts_global_x, vector<double> pts_global_y);
    
    void transform2GlobalCoord(vector<double> &pts_global_x, vector<double> & pts_global_y, double ref_x, double ref_y, double ref_yaw, vector<double> pts_car_x, vector<double> pts_car_y);
    
public:
    
    // variables:
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    
    // constructor
    TrajectoryPlanner();
    
    virtual ~TrajectoryPlanner();
    
    // methods:
    void readMap(string mapfile);
    
    void getCurrentTelemetry(vector<double> car_telemetry);
    
    void getPreviousPath(vector<double> prev_x_path, vector<double> prev_y_path);
    
    void getSensorData(vector<vector<double>> sensor_data);
    
    void calculateTrajectory();
    
};

#endif /* Planner_h */
