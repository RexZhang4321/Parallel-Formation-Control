//
//  control_method.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "control_method.hpp"

void model_controller::do_control(__IN model_state_t* cur, __IN model_state_t* future, __OUT control_input_t* input) {
    pi_controller(cur, future, input);
}

void model_controller::pi_controller(__IN model_state_t* cur, __IN model_state_t* future, __OUT control_input_t* input) {
    // TODO
}
