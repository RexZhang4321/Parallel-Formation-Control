//
//  formation_map.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "formation_map.hpp"

formation_map::formation_map() : boundary_x(999), boundary_y(999), n_robot(4)  {
    // default formation shape
    formation_shape.push_back(formation_point_t(50, 50, 1));
    formation_shape.push_back(formation_point_t(50, -50, 1));
    formation_shape.push_back(formation_point_t(-50, -50, 1));
    formation_shape.push_back(formation_point_t(-50, 50, 1));
    
    // default formation path
    for (int i = 0; (i * 100 + 64) < this->boundary_y; i++) {
        formation_path.push_back(formation_point_t(499, i * 100 + 64, 1));
    }
    
    // default formation barrier
    for (int x = 399; x <= 599; x++) {
        formation_barrier.push_back(formation_point_t(x, 299, -1));
    }
    
    // default formation central point
    this->central_point = formation_path[0];
}

formation_map::formation_map(int bx, int by, int n_robot) {
    this->boundary_x = bx;
    this->boundary_y = by;
    this->n_robot = n_robot;
}

formation_map::~formation_map() {}

void
formation_map::generate_formation_path() {
    // TODO
}

vector<formation_point_t>
formation_map::get_formation_path() {
    return this->formation_path;
}

void
formation_map::generate_formation_barrier() {
    // TODO
    // need to consider cost near the barrier
}

vector<formation_point_t>
formation_map::get_formation_barrier() {
    return this->formation_barrier;
}

vector<formation_point_t>
formation_map::create_formation_shape(int num_model, formation_point_t &central_point) {
    vector<formation_point_t> t;
    return t;
}

// under such circumstances will trigger this function:
// 1. other robot is within the distance of 2r
// 2. when the robot reaches within the sensor boundary of goal
// 3. when the robot travels a certain distance
// here we will only use line to find sensor map goal
// return point will be passed to Dstar algorithm to generate a new path
// need int position
formation_point_t
formation_map::gen_sensor_map_goal(model_state_t robot_location, double r) {
    formation_point_t &cur_formation_goal = formation_path[cur_formation_goal_id];
    double d_x = cur_formation_goal.x - robot_location.x;
    double d_y = cur_formation_goal.y - robot_location.y;
    
    // if distance of current formation_goal is smaller than r, update current formation goal
    double d_x2 = pow(d_x, 2);
    double d_y2 = pow(d_y, 2);
    if ((d_x2 + d_y2) <= r * r) {
        if (cur_formation_goal_id == formation_path.size() - 1) {
            // we have reached the destination
            ;
        } else {
            cur_formation_goal_id++;
        }
    }
    
    // generate the new sensor goal
    double c = sqrt(d_x2 + d_y2);
    formation_point_t t;
    d_x = r / c * d_x + robot_location.x;
    d_y = r / c * d_y + robot_location.y;
    t.x = static_cast<int>(d_x);
    t.y = static_cast<int>(d_y);
    return t;
}

// use global formation_map OR formation_barrier to create a new path in sensor map using Dstar
// need int position
vector<formation_point_t>
formation_map::create_new_sensor_path(formation_point_t sensor_goal, model_state_t robot_location) {
    vector<formation_point_t> t;
    return t;
}
