#ifndef _MODEL_COMPONENTS_H
#define _MODEL_COMPONENTS_H

#include <math.h>
#include "defs.hpp"

#define MAX_VELOCITY 20.0
#define MIN_VELOCITY -20.0
#define MAX_ACCELERATION 100.0
#define MIN_ACCELERATION -100.0
#define MAX_ANGLE PI/3
#define MIN_ANGLE -PI/3

double vel_limit(double v);

double acc_limit(double v, double last_vel, double dt);

double steering_angle_limit(double angle);

double degree_to_radian(double degree);

double radian_to_degree(double radian);

double integrator(double old_val, double new_val, double dt, int enable);

#endif
