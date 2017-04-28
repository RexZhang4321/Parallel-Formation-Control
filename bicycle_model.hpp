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
