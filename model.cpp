//
//  model.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "model.hpp"


void robot_model::model_move(__IN control_input_t* input, __IN __OUT model_state_t* stat) {
    bicycle_move(input, stat);
}
