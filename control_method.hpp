//
//  control_method.hpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//
//  This file, along with the control_method.cpp, defines the control method we
//  used in this project. Which is a basic P controller, where the control variable
//  are only determined by the a proportional factor multiplied to the difference
//  between the target status and current status.

#ifndef control_method_hpp
#define control_method_hpp

#include <stdio.h>
#include "defs.hpp"
#include <math.h>
#include "model_components.hpp"

class model_controller {
public:
    void do_control(__IN model_state_t* cur, __IN formation_point_t* target, __OUT control_input_t* input);
    
    void pi_controller(__IN model_state_t* cur, __IN formation_point_t* target, __OUT control_input_t* input);
};


#endif /* control_method_hpp */
