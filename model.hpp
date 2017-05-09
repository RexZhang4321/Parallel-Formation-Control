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
#include "Dstar.h"
#include <math.h>

using namespace std;

class robot_model {
public:
    robot_model();
    
    int robot_id;
    int n_total_robots;
    
    bool is_waiting;
    
    double last_vel;
    
    virtual void model_move() = 0;
    
    model_state_t cur_state;
    
    vector<formation_point_t> formation_path;
    vector<int> common_formation_path_id_list;
    
    int formation_goal_id;
    
    formation_point_t cur_sensor_goal;
    
    int search_sensor_path_id; // control goal
    
    bool formation_goal_needs_update();
    
    bool sensor_goal_needs_update();
    
    bool control_goal_needs_update();
    
    vector<formation_point_t> sensor_path;

    // robot observation r
    double sensor_map_r = 20;
    
    // used for getting control goal
    double control_goal_r = 5;
    
    // used for triggering update
    double control_goal_update_r = 2;
    
    void update_formation_goal();
    
    void update_sensor_goal(vector<formation_point_t> barrier);
    
    // when the robot approaches the last control goal within certain distance r'
    void update_control_goal();
    
    void create_new_sensor_path(vector<formation_point_t> barrier);
    
    vector<formation_point_t> observe_barrier(vector<formation_point_t> barrier);
    
    control_input_t input;
    
    model_controller m_ctl;
    
    bool need_new_sensor_path;
};


#endif /* model_hpp */
