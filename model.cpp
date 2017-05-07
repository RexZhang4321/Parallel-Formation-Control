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

robot_model::robot_model() {
    this->search_sensor_path_id = 0;
}

void
robot_model::gen_control_goal() {
    // find the first point on sensor path whose distance is larger than control_goal_r
    int control_goal_r_square = control_goal_r * control_goal_r;
    for (int i = search_sensor_path_id; i < sensor_path.size(); i++) {
        formation_point_t &sensor_point = sensor_path[i];
        if (calc_distance_square(sensor_point.x, sensor_point.y,
                                 cur_state.x, cur_state.y) >= control_goal_r_square) {
            search_sensor_path_id = i;
//            cur_control_goal.x = sensor_point.x;
//            cur_control_goal.y = sensor_point.y;
            return;
        }
    }
    search_sensor_path_id = sensor_path.size() - 1;
}

bool
robot_model::control_goal_needs_update() {
    formation_point_t &cur_control_goal = sensor_path[search_sensor_path_id];
    double d_x = pow((cur_control_goal.x - this->cur_state.x), 2);
    double d_y = pow((cur_control_goal.y - this->cur_state.y), 2);
    return (d_x + d_y) <= pow(this->control_goal_update_r, 2);
}

vector<formation_point_t>
robot_model::observe_barrier(vector<formation_point_t> barrier) {
    return barrier;
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
