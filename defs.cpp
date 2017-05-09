//
//  defs.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "defs.hpp"

int current_common_formation_path_idx;
int num_robot_enter_formation_goal;
bool can_update;

model_state_t::model_state_t() : x(0), y(0), theta(0) {}

control_input_t::control_input_t() : v(0), gamma(0) {}

formation_point_t::formation_point_t() : x(0), y(0), cost(0) {}

formation_point_t::formation_point_t(int _x, int _y, double _cost) :
x(_x), y(_y), cost(_cost) {}
