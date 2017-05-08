//
//  control_method.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "control_method.hpp"

double normalize_angle(double u) {
    u = fmod(u, 2 * PI);
    if (u > PI) {
        u = u - 2 * PI;
    } else if (u <= -PI) {
        u = u + 2 * PI;
    }
    return u;
}

void model_controller::do_control(__IN model_state_t* cur, __IN formation_point_t* target, __OUT control_input_t* input) {
    pi_controller(cur, target, input);
}

void model_controller::pi_controller(__IN model_state_t* cur, __IN formation_point_t* target, __OUT control_input_t* input) {
    // TODO. no accel limit

	// Below is the P parameters for speed v and moving angle theta of control input.
	float K_v = 1;
	float K_h = 1;

    // Below is the speed v of control input of robot.
	float original_v = K_v * sqrt(pow(target->x - cur->x, 2) + pow(target->y - cur->y, 2));
	input->v = vel_limit(original_v);

	// Below is to calculate the moving angle of control input of robot.
	// This angle has a limit.
//    printf("cur: x: %lf, y: %lf;\t target: x: %d, y: %d\n", cur->x, cur->y, target->x, target->y);
	float theta_star = atan2((target->y - cur->y), (target->x - cur->x));
	float original_gamma = K_h * normalize_angle(normalize_angle(theta_star) - normalize_angle(cur->theta));
	input->gamma = original_gamma;
//    printf("%lf\n", original_gamma);
}
