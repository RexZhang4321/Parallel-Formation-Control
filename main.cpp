#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    int n_model = 2;
    current_common_formation_path_idx = 1;
    num_robot_enter_formation_goal.resize(n_model);
    can_update = false;
    formation_map* fmp = new formation_map(n_model);

    fmp->cur_formation_goal_id = 1;

    vector<bicycle> robots(n_model);
    vector<FILE*> fps(n_model);
    vector<bool> common_formation_path(fmp->formation_path.size(), true);
    char robot_filename[100];
    for (int i = 0; i < robots.size(); i++) {
        sprintf(robot_filename, "./pic/robot%d", i);
        fps[i] = fopen(robot_filename, "w");
        robots[i].robot_id = i;
        int formation_path_idx = 0;
        // robot formation goal offset
        bool skip_flg = false;
        for (auto &formation_path_point: fmp->formation_path) {
            skip_flg = false;
            for (auto &barrier_point: fmp->formation_barrier) {
                if (formation_path_point.x + fmp->formation_shape[i].x == barrier_point.x &&
                    formation_path_point.y + fmp->formation_shape[i].y == barrier_point.y) {
                    skip_flg = true;
                    common_formation_path[formation_path_idx] =
                        common_formation_path[formation_path_idx] & false;
                    break;
                }
            }
            formation_path_idx++;
            if (skip_flg) continue;
            robots[i].formation_path.push_back(
                    formation_point_t(formation_path_point.x + fmp->formation_shape[i].x,
                                      formation_path_point.y + fmp->formation_shape[i].y,
                                      0));
        }
        // set initial robot position
        robots[i].cur_state.x = robots[i].formation_path[0].x;
        robots[i].cur_state.y = robots[i].formation_path[0].y;
        robots[i].cur_state.theta = PI / 2;
        // at least two formation points
        robots[i].formation_goal_id = 1;
        // set initial sensor goal
        robots[i].cur_sensor_goal.x = robots[i].cur_state.x;
        robots[i].cur_sensor_goal.y = robots[i].cur_state.y;
        robots[i].update_sensor_goal(fmp->get_formation_barrier());
        // set initial control goal
        robots[i].update_control_goal();
    }
    
    vector<int> common_formation_path_id_list;
    for (int i = 0; i < common_formation_path.size(); i++) {
        if (common_formation_path[i]) {
            common_formation_path_id_list.push_back(i);
            printf("%d\t", i);
        }
    }
    printf("\n");
    for (auto &robot: robots) {
        robot.common_formation_path_id_list = common_formation_path_id_list;
        robot.n_total_robots = n_model;
    }

    // output barrier information
    const char barrier_filename[100] = "./pic/barrier";
    FILE *fp = fopen(barrier_filename, "w");
    for (auto barrier_point: fmp->get_formation_barrier()) {
        fprintf(fp, "%d\t%d\t%lf\n", barrier_point.x, barrier_point.y, barrier_point.cost);
    }
    fclose(fp);

    int i = 0;
    while (current_common_formation_path_idx != common_formation_path_id_list.size() - 1) {
        for (int rid = 0; rid < robots.size(); rid++) {
            /* when the robot reaches within certain distance to the sensor goal */
            robots[rid].update_formation_goal();
            robots[rid].update_sensor_goal(fmp->get_formation_barrier());
            /* when the robot approaches the last control goal within certain distance control_goal_update_r */
            robots[rid].update_control_goal();
            robots[rid].model_move();
            // print every 0.1 seconds OR 10 steps
            if (i % 100 == 0 && rid == 0) {
                printf("%d, Robot %d: x: %lf, y: %lf, theta: %lf\n", i, rid,
                       robots[rid].cur_state.x, robots[rid].cur_state.y, robots[rid].cur_state.theta);
                printf("formation map goal: %d, %d\n", robots[rid].formation_path[robots[rid].formation_goal_id].x, robots[rid].formation_path[robots[rid].formation_goal_id].y);
                printf("control goal: %d, %d\n", robots[rid].sensor_path[robots[rid].search_sensor_path_id].x, robots[rid].sensor_path[robots[rid].search_sensor_path_id].y);
                printf("common formation goal: %d\n", common_formation_path_id_list[current_common_formation_path_idx]);
            }
            if (i % 10 ==0) {
                fprintf(fps[rid], "%lf\t%lf\n", robots[rid].cur_state.x, robots[rid].cur_state.y);
            }
        }
        i++;
    }
    for (auto fp: fps) {
        fclose(fp);
    }
    #ifdef __USE_MPI__
    MPI_Finalize();
    #endif
    return 0;
}
