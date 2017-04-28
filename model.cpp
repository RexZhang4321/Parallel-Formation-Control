//
//  model.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "model.hpp"

formation_point_t
robot_model::gen_control_goal() {
    // TODO
    formation_point_t t;
    return t;
}

bool
robot_model::control_goal_needs_update() {
    double d_x = pow((this->cur_control_goal.x - this->cur_state.x), 2);
    double d_y = pow((this->cur_control_goal.y - this->cur_state.y), 2);
    return (d_x + d_y) <= pow(this->control_goal_update_r, 2);
}
