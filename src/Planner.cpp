//
//  Planner.cpp
//  path_planning
//
//  Created by ChenLiheng on 25.02.18.
//
#include "Planner.h"

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

void TrajectoryPlanner::getPreviousPath(vector<double> prev_x_path, vector<double> prev_y_path){
    prev_x_ = prev_x_path;
    prev_y_ = prev_y_path;
}

void TrajectoryPlanner::getSensorData(vector<vector<double> > sensor_data) {
    int size_sensor = sensor_data.size();
    vector<sensor_obj> objs;
    sensor_obj obj;
    for (int i=0; i<size_sensor; i++) {
        obj.id = sensor_data[i][0];
        obj.x = sensor_data[i][1];
        obj.y = sensor_data[i][2];
        obj.vx = sensor_data[i][3];
        obj.vy = sensor_data[i][4];
        obj.s = sensor_data[i][5];
        obj.d = sensor_data[i][6];
        
        objs.push_back(obj);
    }
    objs_ = objs;
    
    car_ahead_id_ = ID_DEFAULT;
    car_left_id_ = ID_DEFAULT;
    car_right_id_ = ID_DEFAULT;
    for (vector<sensor_obj>::iterator it = objs_.begin(); it!=objs_.end(); ++it) {
        // check car ahead
        // check lane
        if (it->d < (2+4*lane_+2) && it->d > (2+4*lane_-2)) {
            // check gap in s
            if (it->s > current_s_ && it->s-current_s_ < SPACE_2_CAR_AHEAD) {
                car_ahead_id_ = it->id;
            }
        }
        
        // check car left
        if (lane_ > 0) {
            // check lane
            if (it->d < (2+4*(lane_-1)+2) && it->d > (2+4*(lane_-1)-2)) {
                if (it->s > current_s_ - 2*CAR_LENGTH && it->s < current_s_ + 2*CAR_LENGTH) {
                    car_left_id_ = it->id;
                    cout << "car left detected: " << car_left_id_ << endl;
                }
            }
        }
        
        // check car right
        if (lane_ >= 0 && lane_ < 2) {
            // check lane
            if (it->d < (2+4*(lane_+1)+2) && it->d > (2+4*(lane_+1)-2)) {
                if (it->s > current_s_ - 2*CAR_LENGTH && it->s < current_s_ + 2*CAR_LENGTH) {
                    car_right_id_ = it->id;
                    cout << "car right detected: " << car_right_id_ << endl;
                }
            }
        }
        
    }
    
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
    
    chooseNextState();
    
    if (current_state_ == KL) {
        lane_ = lane_ + 0;
    }
    
    driveLane(lane_);
}

void TrajectoryPlanner::transform2CarCoord(vector<double> &pts_car_x, vector<double> &pts_car_y, double ref_x, double ref_y, double ref_yaw, vector<double> pts_global_x, vector<double> pts_global_y){
    
    for (int i = 0; i < pts_global_x.size(); i++) {
        pts_car_x[i] = (pts_global_x[i] - ref_x) * cos(ref_yaw) + (pts_global_y[i] - ref_y) * sin(ref_yaw);
        pts_car_y[i] = (pts_global_y[i] - ref_y) * cos(ref_yaw) - (pts_global_x[i] - ref_x) * sin(ref_yaw);
    }
}

void TrajectoryPlanner::transform2GlobalCoord(vector<double> &pts_global_x, vector<double> & pts_global_y, double ref_x, double ref_y, double ref_yaw, vector<double> pts_car_x, vector<double> pts_car_y){
    
    for (int i = 0; i < pts_car_x.size(); i++) {
        pts_global_x[i] = ref_x + pts_car_x[i]*cos(ref_yaw) - pts_car_y[i]*sin(ref_yaw);
        pts_global_y[i] = ref_y + pts_car_x[i]*sin(ref_yaw) + pts_car_y[i]*cos(ref_yaw);
    }
}

void TrajectoryPlanner::calculateVelocity() {
    
    double target_vel = max_vel_*mph2ms;
    if (car_ahead_id_ != ID_DEFAULT) {
        target_vel = (dist(0.0, 0.0, objs_[car_ahead_id_].vx, objs_[car_ahead_id_].vy));
    }
    
    if (vel_ - target_vel > 0.1) {
        vel_ -= max_acc_;
    } else if (target_vel - vel_ > 0.1) {
        vel_ += max_acc_;
    }
}

void TrajectoryPlanner::driveLane(int lane) {
    
    vector<double> ptsx, ptsy;
    
    double ref_x = current_x_;
    double ref_y = current_y_;
    double ref_yaw = deg2rad(current_yaw_);
    
    double prev_x = current_x_ - cos(current_yaw_);
    double prev_y = current_y_ - sin(current_yaw_);
    
    
    if (prev_x_.size() > 2) {
        ref_x = prev_x_[prev_x_.size()-1];
        ref_y = prev_y_[prev_x_.size()-1];
        
        prev_x = prev_x_[prev_x_.size()-2];
        prev_y = prev_y_[prev_x_.size()-2];
        ref_yaw = atan2(ref_y-prev_y, ref_x-prev_x);
    }
    
    ptsx.push_back(prev_x);
    ptsx.push_back(ref_x);
    
    ptsy.push_back(prev_y);
    ptsy.push_back(ref_y);
    
    
    vector<int> anchor_pts = {30, 60, 90};
    
    for (vector<int>::iterator it = anchor_pts.begin(); it!=anchor_pts.end(); ++it){
        vector<double> next = getXY(current_s_+*it, (2+4*lane), map_s_, map_x_, map_y_);
        ptsx.push_back(next[0]);
        ptsy.push_back(next[1]);
    }
    
    vector<double> ptsx_car_coord(ptsx.size());
    vector<double> ptsy_car_coord(ptsy.size());
    
    transform2CarCoord(ptsx_car_coord, ptsy_car_coord, ref_x, ref_y, ref_yaw, ptsx, ptsy);
    
    tk::spline spl;
    
    // set points to spline
    spl.set_points(ptsx_car_coord, ptsy_car_coord);
    
    vector<double> x_out(COUNT_GEN_PTS);
    vector<double> y_out(COUNT_GEN_PTS);
    
    // add previous points
    for (int i=0; i<prev_x_.size(); i++){
        x_out[i] = prev_x_[i];
        y_out[i] = prev_y_[i];
    }
    
    vector<double> cal_x, cal_y;
    
    double target_x = 30.0;
    double target_y = spl(target_x);
    double target_dist = dist(0.0, 0.0, target_x, target_y);
    
    //add new points
    double x_add_on = 0.0;
    
    for (int i=prev_x_.size(); i<COUNT_GEN_PTS; i++){
        calculateVelocity();
        double N = target_dist/timestep_/vel_;
        double x_point = x_add_on + target_x/N;
        cal_x.push_back(x_point);
        cal_y.push_back(spl(x_point));
        
        x_add_on = x_point;
    }
    
    vector<double> cal_x_global(cal_x.size());
    vector<double> cal_y_global(cal_y.size());
    transform2GlobalCoord(cal_x_global, cal_y_global, ref_x, ref_y, ref_yaw, cal_x, cal_y);
    
    int idx = 0;
    for (int i=prev_x_.size(); i<COUNT_GEN_PTS; i++){
        x_out[i] = cal_x_global[idx];
        y_out[i] = cal_y_global[idx];
        idx++;
    }
    
    next_x_vals = x_out;
    next_y_vals = y_out;
}

void TrajectoryPlanner::getSuccessorStates() {
    
    vector<FSM_State> possible_states;
    possible_states.push_back(KL);
    
    if(current_state_ == KL) {
        possible_states.push_back(PCLL);
        possible_states.push_back(PCLR);
    } else if (current_state_ == PCLL) {
        if (lane_-1 >= 0) {
            possible_states.push_back(PCLL);
            possible_states.push_back(CLL);
        }
    } else if (current_state_ == PCLR) {
        if (lane_+1 <=2) {
            possible_states.push_back(PCLR);
            possible_states.push_back(CLR);
        }
    }
    
    possible_states_ = possible_states;
}

void TrajectoryPlanner::chooseNextState() {
    
    getSuccessorStates();
    
    
}

