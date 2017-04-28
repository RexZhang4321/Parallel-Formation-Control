//
//  formation_map.hpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#ifndef formation_map_hpp
#define formation_map_hpp

#include <stdio.h>
#include <vector>
#include "Dstar.h"
#include <math.h>
#include "defs.hpp"

using namespace std;

class formation_map {
public:
    formation_map();
    formation_map(int bx, int by, int n_robot);
    ~formation_map();

    void generate_formation_path();
    vector<formation_point_t> get_formation_path();

    void generate_formation_barrier();
    vector<formation_point_t> get_formation_barrier();

    vector<formation_point_t> create_formation_shape(int num_model, formation_point_t &central_point);
    
    formation_point_t gen_sensor_map_goal(model_state_t robot_location, double r);
    
    vector<formation_point_t> create_new_sensor_path(formation_point_t sensor_goal, model_state_t robot_location);

    // boudary
    // assume the map starts from (0, 0) to (boundary_x, boundary)
    int boundary_x;
    int boundary_y;
    
    int n_robot;
    
    int cur_formation_goal_id;
    
    formation_point_t central_point;
    
    // formation path
    vector<formation_point_t> formation_path;
    
    // barrier
    vector<formation_point_t> formation_barrier;
    
    // formation shape
    vector<formation_point_t> formation_shape;
};

#endif /* formation_map_hpp */
