//
//  model.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "model.hpp"

double calc_distance_square(double x1, double y1, double x2, double y2) {
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

bool distance_is_smaller(formation_point_t &p, model_state_t &s, double r) {
    return calc_distance_square(p.x, p.y, s.x, s.y) <= (r * r);
}

robot_model::robot_model() {
    this->search_sensor_path_id = 0;
    this->formation_goal_id = 0;
    this->need_new_sensor_path = true;
    this->last_vel = 0;
    this->is_waiting = false;
}

void
robot_model::update_control_goal() {
    if (control_goal_needs_update()) {
        // find the first point on sensor path whose distance is larger than control_goal_r
        int control_goal_r_square = control_goal_r * control_goal_r;
        for (int i = search_sensor_path_id; i < sensor_path.size(); i++) {
            formation_point_t &sensor_point = sensor_path[i];
            if (calc_distance_square(sensor_point.x, sensor_point.y,
                                 cur_state.x, cur_state.y) >= control_goal_r_square) {
                search_sensor_path_id = i;
                return;
            }
        }
        search_sensor_path_id = sensor_path.size() - 1;
    }
}

bool
robot_model::formation_goal_needs_update() {
    bool check_distance = distance_is_smaller(formation_path[formation_goal_id], cur_state, sensor_map_r);
    if (check_distance) {
        if (formation_goal_id < common_formation_path_id_list[current_common_formation_path_idx]) {
            return true;
        } else if (formation_goal_id == common_formation_path_id_list[current_common_formation_path_idx]) {
            if (can_update) {
                num_robot_enter_formation_goal--;
                is_waiting = false;
                if (num_robot_enter_formation_goal == 0) {
                    can_update = false;
                    current_common_formation_path_idx++;
                }
                return true;
            } else {
                if (!is_waiting) {
                    num_robot_enter_formation_goal++;
                    is_waiting = true;
                }
                if (num_robot_enter_formation_goal == n_total_robots) {
                    can_update = true;
                }
                return false;
            }
        } else {
            return false;
        }
    }
    return false;
}

bool
robot_model::sensor_goal_needs_update() {
    return distance_is_smaller(cur_sensor_goal, cur_state, control_goal_r);
}

bool
robot_model::control_goal_needs_update() {
//    formation_point_t &cur_control_goal = sensor_path[search_sensor_path_id];
//    double d_x = pow((cur_control_goal.x - this->cur_state.x), 2);
//    double d_y = pow((cur_control_goal.y - this->cur_state.y), 2);
//    return (d_x + d_y) <= pow(this->control_goal_update_r, 2);
    return distance_is_smaller(sensor_path[search_sensor_path_id], cur_state,
                                 control_goal_r);
}

vector<formation_point_t>
robot_model::observe_barrier(vector<formation_point_t> barrier) {
    return barrier;
}

void
robot_model::update_formation_goal() {
    if (formation_goal_needs_update() && (formation_goal_id != formation_path.size() - 1)) {
        formation_goal_id++;
    }
}


// under such circumstances will trigger this function:
// 1. other robot is within the distance of 2r
// 2. when the robot reaches within the sensor boundary of goal
// 3. when the robot travels a certain distance
// here we will only use line to find sensor map goal
// return point will be passed to Dstar algorithm to generate a new path
// need int position
void
robot_model::update_sensor_goal(vector<formation_point_t> barrier) {
    if (sensor_goal_needs_update()) {
        if (distance_is_smaller(formation_path[formation_goal_id], cur_state, control_goal_r)) {
            cur_sensor_goal = formation_path[formation_goal_id];
            if (need_new_sensor_path) {
                create_new_sensor_path(barrier);
            } else {
                return;
            }
            need_new_sensor_path = false;
        } else {
            formation_point_t &cur_formation_goal = formation_path[formation_goal_id];
            double d_x = cur_formation_goal.x - cur_state.x;
            double d_y = cur_formation_goal.y - cur_state.y;
            double d_x2 = d_x * d_x;
            double d_y2 = d_y * d_y;
            // generate the new sensor goal
            double c = sqrt(d_x2 + d_y2);
            d_x = round(sensor_map_r / c * d_x + cur_state.x);
            d_y = round(sensor_map_r / c * d_y + cur_state.y);
            cur_sensor_goal.x = static_cast<int>(round(d_x));
            cur_sensor_goal.y = static_cast<int>(round(d_y));
            for (auto& barrier_point: barrier) {
                if (cur_sensor_goal.x == barrier_point.x &&
                    cur_sensor_goal.y == barrier_point.y) {
                    cur_sensor_goal.x = cur_formation_goal.x;
                    cur_sensor_goal.y = cur_formation_goal.y;
                    break;
                }
            }
            create_new_sensor_path(barrier);
        }
    }
}


// use global formation_map OR formation_barrier to create a new path in sensor map using Dstar
// need int position
void
robot_model::create_new_sensor_path(vector<formation_point_t> barrier) {
    vector<formation_point_t> observed_barrier = this->observe_barrier(barrier);
    Dstar dstar;
    dstar.init(round(cur_state.x), round(cur_state.y), cur_sensor_goal.x, cur_sensor_goal.y);
    for (auto &barrier_block: observed_barrier) {
        dstar.updateCell(barrier_block.x, barrier_block.y, barrier_block.cost);
    }
    dstar.replan();
    list<state> dstar_path = dstar.getPath();
    sensor_path.clear();
    search_sensor_path_id = 0;
    for (auto &path_point: dstar_path) {
        sensor_path.push_back(formation_point_t(path_point.x, path_point.y, 0));
    }
}
