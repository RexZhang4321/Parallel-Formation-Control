//
//  control_method.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "control_method.hpp"
#include <math.h>

void model_controller::do_control(__IN model_state_t* cur, __IN formation_point_t* target, __OUT control_input_t* input) {
    pi_controller(cur, target, input);
}

void model_controller::pi_controller(__IN model_state_t* cur, __IN formation_point_t* target, __OUT control_input_t* input) {
    // TODO. no accel limit

	// Below is the P parameters for speed v and moving angle theta of control input.
	float K_v = 1;
	float K_h = 1;

    // Below is the speed v of control input of robot.
	float original_v = K_h * sqrt(pow(target->x - cur->x, 2) + pow(target->y - cur->y, 2));
	input->v = vel_limit(original_v);

	// Below is to calculate the moving angle of control input of robot.
	// This angle has a limit.
	float theta_star = atan((target->y - cur->y) / (target->x - cur->x));
	float original_gamma = K_h * (theta_star - cur->theta);
	input->gamma = steering_angle_limit(original_gamma);
	}
}
