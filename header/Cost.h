//
//  Cost.h
//  Path_Planning
//
//  Created by ChenLiheng on 03.03.18.
//

#ifndef Cost_h
#define Cost_h

#include "helper_function.h"
#include <math.h>
#include <vector>

using namespace std;

class LaneCost{
    // telemetry
    double ego_x_;
    double ego_y_;
    double ego_s_;
    double ego_d_;
    double ego_yaw_;
    double ego_speed_;
    
    double inefficient_cost_;
    double target_lane_cost_;
    
    vector<sensor_obj> vehicles_;
    double dist_2_ego_s;
    
    void calculateLaneSpeed();
    
    void calculateIneffCost();
    
    void calculateTargetCost();
    
public:
    double lane_cost;
    double lane_speed;
    
    LaneCost();
    
    virtual ~LaneCost();
    
    void add_ego_telemetry(double x, double y, double s, double d, double yaw, double v);
    
    void add_vehicle(sensor_obj obj);
    
    void calculateCost();
    
};

#endif /* Cost_h */
