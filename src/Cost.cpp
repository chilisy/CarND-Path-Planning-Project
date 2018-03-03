//
//  Cost.cpp
//  path_planning
//
//  Created by ChenLiheng on 03.03.18.
//

#include "Cost.h"

LaneCost::LaneCost() {
    
}

LaneCost::~LaneCost() {
    
}

void LaneCost::add_ego_telemetry(double x, double y, double s, double d, double yaw, double v) {
    current_x_ = x;
    current_y_ = y;
    current_s_ = s;
    current_d_ = d;
    current_yaw_ = yaw;
    current_speed_ = v;
}

void LaneCost::add_vehicle(sensor_obj obj) {
    vehicles_.push_back(obj);
}

