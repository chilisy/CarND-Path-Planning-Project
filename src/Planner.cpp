//
//  Planner.cpp
//  path_planning
//
//  Created by ChenLiheng on 25.02.18.
//

#include "helper_function.h"
#include "Planner.h"
#include "spline.h"

TrajectoryPlanner::TrajectoryPlanner() {
    
}

TrajectoryPlanner::~TrajectoryPlanner() {
    
}

void TrajectoryPlanner::getCurrentTelemetry(vector<double> car_telemetry){
    current_x_ = car_telemetry[0];
    current_y_ = car_telemetry[1];
    current_s_ = car_telemetry[2];
    current_d_ = car_telemetry[3];
    current_yaw_ = car_telemetry[4];
    current_speed_ = car_telemetry[5];
}

void TrajectoryPlanner::getMapPoints(vector<double> x, vector<double> y, vector<double> s, vector<double> dx, vector<double> dy){
    map_x_ = x;
    map_y_ = y;
    map_s_ = s;
    map_dx_ = dx;
    map_dy_ = dy;
}

void TrajectoryPlanner::calculateTrajectory() {
    
    stayConstantInFrenet();
}

void TrajectoryPlanner::stayConstantInFrenet() {
    vector<double> next;
    
    double dist_inc = 0.4;
    for (int i=0; i<50; i++){
        double next_s = current_s_+(i+1)*dist_inc;
        double next_d = 6;
        
        next = getXY(next_s, next_d, map_s_, map_x_, map_y_);
        next_x_vals.push_back(next[0]);
        next_y_vals.push_back(next[1]);
    }
}

