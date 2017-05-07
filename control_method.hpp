//
//  control_method.hpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

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
