//
//  Planner.h
//  Path_Planning
//
//  Created by ChenLiheng on 25.02.18.
//

#ifndef Planner_h
#define Planner_h

#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"

using namespace std;

class TrajectoryPlanner{
    
    // variables:
    double current_x_;
    double current_y_;
    double current_s_;
    double current_d_;
    double current_yaw_;
    double current_speed_;
    
    vector<double> map_x_;
    vector<double> map_y_;
    vector<double> map_s_;
    vector<double> map_dx_;
    vector<double> map_dy_;
    
    // The max s value before wrapping around the track back to 0
    double max_s_ = 6945.554;
    
    // methods:
    void stayConstantInFrenet();
    
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
    
    void calculateTrajectory();
    
};

#endif /* Planner_h */
