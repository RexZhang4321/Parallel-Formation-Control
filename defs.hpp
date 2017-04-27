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

typedef struct {
    double x;
    double y;
    double theta;
} model_state_t;

typedef struct {
    double v;
    double gamma;
} control_input_t;

typedef struct {
    int x;
    int y;
    double cost;
} formation_point_t;

#endif /* utils_hpp */
