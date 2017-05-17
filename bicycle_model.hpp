//
//  bicycle_model.hpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//
//  This is the head file of the bicycle_model.cpp. This file inherits the base
//  class of model, where the function model_move() a pure virtual function (or interface
//  in other languages) that must be implemented, and this bicycle model basically 
//  implements the robot model we use. We call this bicycle model because this is 
//  a basic dynamic model where there are two wheels and one wheel can control the
//  direction.
//

#ifndef _BICYCLE_MODEL_H
#define _BICYCLE_MODEL_H

#include "model_components.hpp"
#include "defs.hpp"
#include "model.hpp"

class bicycle : public robot_model {
public:
    void model_move();
    
private:
    void bicycle_move();
};


#endif
