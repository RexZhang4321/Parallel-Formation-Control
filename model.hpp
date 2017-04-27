//
//  model.hpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#ifndef model_hpp
#define model_hpp

#include <stdio.h>
#include <vector>
#include "bicycle_model.hpp"

using namespace std;

class robot_model {
public:
    void model_move(__IN control_input_t* input, __IN __OUT model_state_t* stat);
    
    // when the robot approaches the last control goal within certain distance r'
    formation_point_t gen_control_goal(vector<formation_point_t> sensor_path, formation_point_t cur_location);
};


#endif /* model_hpp */
