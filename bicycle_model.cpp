#include "bicycle_model.hpp"

void bicycle_move(__IN control_input_t* input, __IN __OUT model_state_t* stat) {
    double v = input->v;
    double gamma = input->gamma;
    double dt = INTERVAL;
    double m_v = acc_limit(vel_limit(v), dt);
    double new_angle = steering_angle_limit(gamma);
    int i;
    double x = stat->x, y = stat->y, theta = stat->theta;
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
    stat->x = x;
    stat->y = y;
    stat->theta = theta;
}
