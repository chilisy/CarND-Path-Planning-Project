//
//  Planner.cpp
//  path_planning
//
//  Created by ChenLiheng on 25.02.18.
//
#include "Planner.h"

TrajectoryPlanner::TrajectoryPlanner() {
    lane_costs_.resize(COUNT_LANES);
}

TrajectoryPlanner::~TrajectoryPlanner() {
    
}

void TrajectoryPlanner::getCurrentTelemetry(vector<double> car_telemetry){
    current_x_ = car_telemetry[0];
    current_y_ = car_telemetry[1];
    s_t_2_ = s_t_1_;
    s_t_1_ = current_s_;
    current_s_ = car_telemetry[2];
    current_d_ = car_telemetry[3];
    current_yaw_ = car_telemetry[4];
    current_speed_ = car_telemetry[5];
    
    int iLane = 0;
    for (vector<LaneCost>::iterator it = lane_costs_.begin(); it!=lane_costs_.end(); ++it){
        it->add_ego_telemetry(current_x_, current_y_, current_s_, current_d_, current_yaw_, current_speed_);
        it->set_lane(iLane);
        iLane++;
    }
    cout << "Get Telemetry" << endl;
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
            // add obj
            lane_costs_[lane_].add_vehicle(*it);
            // check gap in s
            if (it->s > current_s_ && it->s-current_s_ < SPACE_2_CAR_AHEAD) {
                car_ahead_id_ = it->id;
            }
        }
        
        // check car left
        if (lane_ > 0 && lane_ <= 2) {
            // check lane
            if (it->d < (2+4*(lane_-1)+2) && it->d > (2+4*(lane_-1)-2)) {
                // add obj
                lane_costs_[lane_-1].add_vehicle(*it);
                
                if (it->s > current_s_ - CAR_LENGTH && it->s < current_s_ + 2*CAR_LENGTH) {
                    car_left_id_ = it->id;
                    cout << "car left detected: " << car_left_id_ << endl;
                }
            }
        }
        
        // check car right
        if (lane_ >= 0 && lane_ < 2) {
            // check lane
            if (it->d < (2+4*(lane_+1)+2) && it->d > (2+4*(lane_+1)-2)) {
                // add obj
                lane_costs_[lane_+1].add_vehicle(*it);
                
                if (it->s > current_s_ - CAR_LENGTH && it->s < current_s_ + 2*CAR_LENGTH) {
                    car_right_id_ = it->id;
                    cout << "car right detected: " << car_right_id_ << endl;
                }
            }
        }
    }
    cout << "Get sensor data" << endl;
}

void TrajectoryPlanner::readMap() {
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    
    ifstream in_map_(map_file_.c_str(), ifstream::in);
    
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
    if (map_x_.size() == 0) {
        cout << "Error: no map found!" << endl;
    }
}

void TrajectoryPlanner::calculateTrajectory() {
    
    chooseNextState();
    
    if (current_state_ == CLL && state_t_1 == PCLL) {
        --lane_;
    } else if (current_state_ == CLR && state_t_1 == PCLR) {
        ++lane_;
    }
    
    cout << "Lane " << lane_ << endl;
    
    driveLane();
    
    cout << "Trajectory calculated! " << endl;
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
    
    //double target_vel = lane_costs_[lane_].lane_speed;
    double target_vel = max_vel*mph2ms;
    if (car_ahead_id_ != ID_DEFAULT) {
     target_vel = (dist(0.0, 0.0, objs_[car_ahead_id_].vx, objs_[car_ahead_id_].vy));
    }
    
    double acc = max_acc;
    if (current_state_ != KL) {
        acc = max_acc/4;
    }
    
    if (vel_ - target_vel > 0.1) {
        vel_ -= acc;
    } else if (target_vel - vel_ > 0.1) {
        vel_ += acc;
    }
    
    cout << "velocity calculated" << endl;
}

void TrajectoryPlanner::driveLane() {
    
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
    
    cout << "Starting points created" << endl;
    
    vector<int> anchor_pts = {25, 60, 90};
    
    cout << "Anchor points defined" << endl;
    
    for (vector<int>::iterator it = anchor_pts.begin(); it!=anchor_pts.end(); ++it){
        cout << "ref_x: " << ref_x << ", ref_y: " << ref_y << ", ref_yaw: " << ref_yaw << endl;
        vector<double> ref_s_d = getFrenet(ref_x, ref_y, ref_yaw, map_x_, map_y_);
        cout << "Ref coordinate in Frenet calculated" << endl;
        vector<double> next_xy = getXY(ref_s_d[0]+*it, 2+4*lane_, map_s_, map_x_, map_y_);
        cout << "Anchor points calculated" << endl;
        ptsx.push_back(next_xy[0]);
        ptsy.push_back(next_xy[1]);
    }
    
    vector<double> ptsx_car_coord(ptsx.size());
    vector<double> ptsy_car_coord(ptsy.size());
    
    transform2CarCoord(ptsx_car_coord, ptsy_car_coord, ref_x, ref_y, ref_yaw, ptsx, ptsy);
    
    tk::spline spl;
    
    // set points to spline
    spl.set_points(ptsx_car_coord, ptsy_car_coord);
    
    cout << "Spline created" << endl;
    
    vector<double> x_out(COUNT_GEN_PTS);
    vector<double> y_out(COUNT_GEN_PTS);
    
    // add previous points
    for (int i=0; i<prev_x_.size(); i++){
        x_out[i] = prev_x_[i];
        y_out[i] = prev_y_[i];
    }
    
    cout << "Previous points added to trajectory" << endl;
    
    vector<double> cal_x, cal_y;
    
    double target_x = 30.0;
    double target_y = spl(target_x);
    double target_dist = dist(0.0, 0.0, target_x, target_y);
    
    //add new points
    double x_add_on = 0.0;
    
    for (int i=prev_x_.size(); i<COUNT_GEN_PTS; i++){
        calculateVelocity();
        double N = target_dist/timestep/vel_;
        double x_point = x_add_on + target_x/N;
        cal_x.push_back(x_point);
        cal_y.push_back(spl(x_point));
        
        x_add_on = x_point;
    }
    
    cout << "New points calculated for trajectory" << endl;
    
    vector<double> cal_x_global(cal_x.size());
    vector<double> cal_y_global(cal_y.size());
    transform2GlobalCoord(cal_x_global, cal_y_global, ref_x, ref_y, ref_yaw, cal_x, cal_y);
    
    cout << "New points transformed to global coordinate system" << endl;
    
    int idx = 0;
    for (int i=prev_x_.size(); i<COUNT_GEN_PTS; i++){
        x_out[i] = cal_x_global[idx];
        y_out[i] = cal_y_global[idx];
        idx++;
    }
    
    cout << "New points added to trajectory" << endl;
    
    next_x_vals = x_out;
    next_y_vals = y_out;
    
    cout << "trajectory calculation complete" << endl;
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
    
    // get possible successor stats
    getSuccessorStates();
    
    state_t_1 = current_state_;
    
    // calculate all lane costs
    for (vector<LaneCost>::iterator it = lane_costs_.begin(); it!=lane_costs_.end(); ++it) {
        it->calculateCost();
    }
    
    double minCost = 99999999;
    int target_lane_id = lane_;
    for (vector<LaneCost>::iterator it = lane_costs_.begin(); it!=lane_costs_.end(); ++it) {
        if (it->lane_cost<minCost){
            minCost = it->lane_cost;
            target_lane_id = it->lane_id;
        }
    }
    
    // do not change 2 lanes at once
    if (target_lane_id-lane_ == 2){
        target_lane_id = lane_;
    }
    
    // state chart
    // KL
    if (current_state_ == KL) {
        if (target_lane_id == lane_ ) {
            current_state_ = KL;
        } else if (target_lane_id < lane_) {
            current_state_ = PCLL;
        } else if (target_lane_id > lane_) {
            current_state_ = PCLR;
        }
    }
    // PCLL
    else if (current_state_ == PCLL) {
        if (target_lane_id == lane_) {
            current_state_  = KL;
        } else if (target_lane_id < lane_) {
            if (find(possible_states_.begin(), possible_states_.end(), CLL)!=possible_states_.end() && car_left_id_ == ID_DEFAULT){
                current_state_ = CLL;
            }
        }
    }
    // PCLR
    else if (current_state_ == PCLR) {
        if (target_lane_id == lane_) {
            current_state_  = KL;
        } else if (target_lane_id > lane_) {
            if (find(possible_states_.begin(), possible_states_.end(), CLR)!=possible_states_.end() && car_right_id_ == ID_DEFAULT){
                current_state_ = CLR;
            }
        }
    }
    // CLL
    else if (current_state_ == CLL) {
        if (fabs(current_d_-(lane_*4)-2) < 0.4) {
            current_state_ = KL;
        }
    }
    // CLR
    else if (current_state_ == CLR) {
        if (fabs(current_d_-(lane_*4)-2) < 0.4) {
            current_state_ = KL;
        }
    }
    cout << "Current state: " << current_state_ << endl;
}

vector<double> TrajectoryPlanner::JMT(vector<double> start, vector <double> end, double T)
{
    /*
     Calculate the Jerk Minimizing Trajectory that connects the initial state
     to the final state in time T.
     
     INPUTS
     
     start - the vehicles start location given as a length three array
     corresponding to initial values of [s, s_dot, s_double_dot]
     
     end   - the desired end state for vehicle. Like "start" this is a
     length three array.
     
     T     - The duration, in seconds, over which this maneuver should occur.
     
     OUTPUT
     an array of length 6, each value corresponding to a coefficent in the polynomial
     s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
     
     EXAMPLE
     
     > JMT( [0, 10, 0], [10, 10, 0], 1)
     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
     */
    vector<double> alpha(6);
    alpha[0] = start[0];
    alpha[1] = start[1];
    alpha[2] = start[2] * 0.5;
    
    Eigen::Matrix3d M;
    M << pow(T, 3), pow(T, 4), pow(T, 5),
    pow(T, 2)*3, pow(T, 3)*4, pow(T, 4)*5,
    T*6, pow(T, 2)*12, pow(T, 3)*20;
    
    Eigen::Vector3d b;
    b << end[0]-(start[0]+start[1]*T+0.5*start[2]*pow(T,2)),
    end[1]-(start[1]+start[2]*T),
    end[2]-start[2];
    
    Eigen::Vector3d a = M.colPivHouseholderQr().solve(b);;
    
    alpha[3] = a[0];
    alpha[4] = a[1];
    alpha[5] = a[2];
    
    return alpha;
}

double TrajectoryPlanner::polyval(double t, vector<double> alpha) {
    double out = 0.0;
    int i = 0;
    for(vector<double>::iterator it = alpha.begin(); it!=alpha.end(); ++it) {
        out += *it*pow(t, i);
        i++;
    }
    return out;
}

