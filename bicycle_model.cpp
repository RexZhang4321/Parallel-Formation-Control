#include "bicycle_model.hpp"

void bicycle::bicycle_move() {
    double v = this->input.v;
    double gamma = this->input.gamma;
    double dt = INTERVAL;
    double m_v = acc_limit(vel_limit(v), dt);
    double new_angle = steering_angle_limit(gamma);
    int i;
    double x = this->cur_state.x, y = this->cur_state.y, theta = this->cur_state.theta;
    double delta_theta = new_angle / N_INTEGRATE;
    double delta_time = INTERVAL / N_INTEGRATE;
    for (i = 0; i < N_INTEGRATE; i++) {
        // integrate cos
        x += m_v * cos(theta) * delta_time;
        // integrate sin
        y += m_v * sin(theta) * delta_time;
        //printf("%lf\n", m_v * sin(stat->theta + i * delta_theta) * delta_time);
        // integrate angle
        theta += dt * m_v * delta_theta;
    }
    this->cur_state.x = x;
    this->cur_state.y = y;
    this->cur_state.theta = theta;
}

void bicycle::model_move() {
    this->m_ctl.do_control(&this->cur_state, &this->cur_control_goal, &this->input);
    this->bicycle_move();
}
