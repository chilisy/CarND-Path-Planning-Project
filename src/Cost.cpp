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

void LaneCost::set_lane(int lane) {
    lane_id = lane;
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

void LaneCost::calculateCost() {
    
    // calculate lane speed
    calculateLaneSpeed();
    
    calculateTargetCost();
    calculateIneffCost();
    
    lane_cost = target_lane_cost_ + inefficient_cost_;
    
    vehicles_.clear();
}

void LaneCost::calculateLaneSpeed() {
    lane_speed = max_vel*mph2ms;
    for (vector<sensor_obj>::iterator it=vehicles_.begin(); it!=vehicles_.end(); ++it){
        if (dist(ego_x_,ego_y_,it->x,it->y)<range_of_concern_ && ego_s_<it->s) {
            lane_speed = min(lane_speed, dist(0.0,0.0,it->vx,it->vy));
        }
    }
}

void LaneCost::calculateTargetCost() {
    
    //double cost = 1-exp(-abs(lane_id-target_lane)) + (lane_id)*0.001;
    double cost = 1-exp(-abs(lane_id-target_lane));
    target_lane_cost_ =  cost * weight_lane;
}

void LaneCost::calculateIneffCost() {
    
    double cost;
    if (lane_speed<=max_vel*mph2ms) {
        cost = 1-exp(-(max_vel*mph2ms-lane_speed)/max_vel/mph2ms);
    }else {
        cost = 1;
    }
    
    inefficient_cost_ = cost*weight_speed;
}

