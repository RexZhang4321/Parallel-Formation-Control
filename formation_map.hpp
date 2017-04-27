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
#include "defs.hpp"

using namespace std;

class formation_map {
public:
    formation_map();
    ~formation_map();

    void generate_formation_path();
    vector<formation_point_t> get_formation_path();

    void generate_formation_barrier();
    vector<formation_point_t> get_formation_barrer();

    vector<formation_point_t> create_formation_shape(int num_model, formation_point_t &central_point);
    
    // under such circumstances will trigger this function:
    // 1. other robot is within the distance of 2r
    // 2. when the robot reaches within the sensor boundary of goal
    // 3. when the robot travels a certain distance
    // return point will be passed to Dstar algorithm to generate a new path
    formation_point_t gen_sensor_map_goal(formation_point_t robot_location, double r);
    
    // use global formation_map OR formation_barrier to create a new path in sensor map using Dstar
    vector<formation_point_t> create_new_sensor_path(formation_point_t sensor_goal, formation_point_t robot_location);
private:
    // boudary
    // assume the map starts from (0, 0) to (boundary_x, boundary)
    int boundary_x;
    int boundary_y;
    
    // formation path
    vector<formation_point_t> formation_path;
    
    // barrier
    vector<formation_point_t> formation_barrier;
    
    // formation shape
    vector<formation_point_t> formation_shape;
};

#endif /* formation_map_hpp */
