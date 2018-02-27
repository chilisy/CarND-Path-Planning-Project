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

using namespace std;

class TrajectoryPlanner{
    
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
    
    void stayConstantInFrenet();
    
public:
    
    // variables
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    
    TrajectoryPlanner();
    
    virtual ~TrajectoryPlanner();
    
    void getCurrentTelemetry(vector<double> car_telemetry);
    
    void getMapPoints(vector<double> x, vector<double> y, vector<double> s, vector<double> dx, vector<double> dy);
    
    void calculateTrajectory();
    
};

#endif /* Planner_h */
