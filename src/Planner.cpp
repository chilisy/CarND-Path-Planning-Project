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

void TrajectoryPlanner::readMap(string mapfile) {
    ifstream in_map_(mapfile.c_str(), ifstream::in);
    
    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_x_.push_back(x);
        map_y_.push_back(y);
        map_s_.push_back(s);
        map_dx_.push_back(d_x);
        map_dy_.push_back(d_y);
    }
}

void TrajectoryPlanner::calculateTrajectory() {
    
    string type = "stayInLaneFrenet";
    
    if (type == "stayInLaneFrenet"){
        stayConstantInFrenet();
    }
}

void TrajectoryPlanner::stayConstantInFrenet() {
    vector<double> next;
    vector<double> x_out, y_out;
    double dist_inc = 0.4;
    for (int i=0; i<100; i++){
        double next_s = current_s_+(i+1)*dist_inc;
        double next_d = 6;
        
        next = getXY(next_s, next_d, map_s_, map_x_, map_y_);
        x_out.push_back(next[0]);
        y_out.push_back(next[1]);
    }
    next_x_vals = x_out;
    next_y_vals = y_out;
}

