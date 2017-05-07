#include <stdio.h>
#include <stdlib.h>
#include "bicycle_model.hpp"
#include "formation_map.hpp"
#include "control_method.hpp"
#include "defs.hpp"
#ifdef __USE_MPI__
#include "mpi.h"
#endif

int main(int argc, char **argv) {
    #ifdef __USE_MPI__
    // MPI init stuffs
    int my_id;
    int num_procs;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &my_id);
    #endif

    // initialize
    formation_map* fmp = new formation_map();

    fmp->cur_formation_goal_id = 1;

    vector<bicycle> robots(4);
    for (int i = 0; i < robots.size(); i++) {
        // set initial robot position
        robots[i].cur_state.x = fmp->formation_shape[i].x + fmp->central_point.x;
        robots[i].cur_state.y = fmp->formation_shape[i].y + fmp->central_point.y;
        robots[i].cur_state.theta = PI / 2;
        // robot formation goal offset
        formation_point_t &tmp_point = fmp->formation_path[fmp->cur_formation_goal_id];
        robots[i].cur_formation_goal.x = tmp_point.x + fmp->formation_shape[i].x;
        robots[i].cur_formation_goal.y = tmp_point.y + fmp->formation_shape[i].y;
        // set initial sensor goal
        robots[i].cur_sensor_goal = fmp->gen_sensor_map_goal(robots[i].cur_state, i, robots[i].sensor_map_r);
        robots[i].create_new_sensor_path(fmp->get_formation_barrier());
        // set initial control goal
        robots[i].gen_control_goal();
    }

    int simulation_time = 520; // seconds
    int sim_steps = simulation_time / INTERVAL;
    for (int i = 0; i < sim_steps; i++) {
        for (int rid = 0; rid < robots.size(); rid++) {
            /* when the robot reaches within certain distance to the sensor goal */
            if (rid == 0) {
                //printf("%d\n", robots[rid].search_sensor_path_id);
            }
            if (robots[rid].sensor_path[robots[rid].search_sensor_path_id] == robots[rid].cur_sensor_goal) {
                // update sensor goal
                robots[rid].cur_sensor_goal = fmp->gen_sensor_map_goal(robots[rid].cur_state, rid, robots[rid].sensor_map_r);
                robots[rid].create_new_sensor_path(fmp->get_formation_barrier());
            }
            /* when the robot approaches the last control goal within certain distance control_goal_update_r */
            if (robots[rid].control_goal_needs_update()) {
                // update control goal
                robots[rid].gen_control_goal();
            }
            robots[rid].model_move();
            // print every 0.1 seconds OR 10 steps
            if (i % 100 == 0 && rid == 0) {
                printf("Robot %d: x: %lf, y: %lf, theta: %lf\n", rid,
                       robots[rid].cur_state.x - 549, robots[rid].cur_state.y, robots[rid].cur_state.theta);
                printf("formation map goal: %d, %d\n", fmp->formation_path[fmp->cur_formation_goal_id].x, fmp->formation_path[fmp->cur_formation_goal_id].y);
                printf("sensor map goal: %d, %d\n", robots[rid].sensor_path[robots[rid].search_sensor_path_id].x, robots[rid].sensor_path[robots[rid].search_sensor_path_id].y);
            }
        }
    }
    #ifdef __USE_MPI__
    MPI_Finalize();
    #endif
    return 0;
}
