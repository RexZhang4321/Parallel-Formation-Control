#ifndef _MODEL_COMPONENTS_H
#define _MODEL_COMPONENTS_H

#include <math.h>

#define PI 3.14159265

#define MAX_VELOCITY 20.0
#define MIN_VELOCITY 0.0
#define MAX_ACCELERATION 100.0
#define MIN_ACCELERATION -100.0
#define MAX_ANGLE 45.0
#define MIN_ANGLE -45.0

double vel_limit(double v);

double acc_limit(double v, double dt);

double steering_angle_limit(double angle);

double degree_to_radian(double degree);

double radian_to_degree(double radian);

double integrator(double old_val, double new_val, double dt, int enable);

#endif
