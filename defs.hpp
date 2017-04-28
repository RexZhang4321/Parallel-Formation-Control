//
//  defs.hpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#ifndef defs_hpp
#define defs_hpp

#include <stdio.h>

#define N_INTEGRATE 1000
#define INTERVAL 0.01
#define MAP_RATIO 100
#define __IN 
#define __OUT

typedef struct model_state_t {
    double x;
    double y;
    double theta;
    model_state_t();
} model_state_t;

typedef struct control_input_t {
    double v;
    double gamma;
    control_input_t();
} control_input_t;

typedef struct formation_point_t {
    int x;
    int y;
    double cost;
    formation_point_t();
    formation_point_t(int, int, double);
    bool operator==(const formation_point_t &fp) {
        return this->x == fp.x && this->y == fp.y;
    }
} formation_point_t;

#endif /* utils_hpp */
