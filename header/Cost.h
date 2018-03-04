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
    const double range_of_concern_ = 60.0;
    
    const double weight_speed = pow(10, 5);
    const double weight_lane = pow(10, 4);
    
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
    
    void calculateLaneSpeed();
    
    void calculateIneffCost();
    
    void calculateTargetCost();
    
public:
    double lane_cost;
    double lane_speed;
    int lane_id;
    
    LaneCost();
    
    virtual ~LaneCost();
    
    void add_ego_telemetry(double x, double y, double s, double d, double yaw, double v);
    
    void set_lane(int lane);
    
    void add_vehicle(sensor_obj obj);
    
    void calculateCost();
    
};

#endif /* Cost_h */
