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
#include "defs.hpp"
#include "control_method.hpp"
#include <math.h>

using namespace std;

class robot_model {
public:
    virtual void model_move() = 0;
    
    // when the robot approaches the last control goal within certain distance r'
    formation_point_t gen_control_goal();
    
    model_state_t cur_state;
    
    formation_point_t cur_formation_goal;
    
    formation_point_t cur_sensor_goal;
    
    formation_point_t cur_control_goal;
    
    bool control_goal_needs_update();
    
    vector<formation_point_t> sensor_path;

    // robot observation r
    double sensor_map_r = 20;
    
    // used for getting control goal
    double control_goal_r = 5;
    
    // used for triggering update
    double control_goal_update_r = 2;
    
    control_input_t input;
    
    model_controller m_ctl;
};


#endif /* model_hpp */
