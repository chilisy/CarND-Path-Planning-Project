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
    ego_x_ = x;
    ego_y_ = y;
    ego_s_ = s;
    ego_d_ = d;
    ego_yaw_ = yaw;
    ego_speed_ = v;
}

void LaneCost::add_vehicle(sensor_obj obj) {
    vehicles_.push_back(obj);
}

