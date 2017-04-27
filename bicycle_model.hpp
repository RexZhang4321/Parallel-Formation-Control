#ifndef _BICYCLE_MODEL_H
#define _BICYCLE_MODEL_H

#include "model_components.hpp"
#include "defs.hpp"

void bicycle_move(__IN control_input_t* input, __IN __OUT model_state_t* stat);

#endif
