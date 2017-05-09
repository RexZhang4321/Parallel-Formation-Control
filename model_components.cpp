#include "model_components.hpp"

double vel_limit(double v) {
    if (v > MAX_VELOCITY) return MAX_VELOCITY;
    if (v < MIN_VELOCITY) return MIN_VELOCITY;
    return v;
}

double acc_limit(double v, double last_velocity, double dt) {
    double acc = (v - last_velocity) / dt;
    if (acc > MAX_ACCELERATION) 
        return MAX_ACCELERATION * dt + last_velocity;
    if (acc < MIN_ACCELERATION) 
        return MIN_ACCELERATION * dt + last_velocity;
    return v;
}

double steering_angle_limit(double angle) {
    if (angle > MAX_ANGLE) return MAX_ANGLE;
    if (angle < MIN_ANGLE) return MIN_ANGLE;
    return angle;
}

double degree_to_radian(double degree) {
    return PI * degree / 180.0;
}

double radian_to_degree(double radian) {
    return 180 * radian / PI;
}

double integrator(double old_val, double new_val, double dt, int enable) {
    return 0.0;
}
