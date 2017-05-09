#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <queue>
#include <vector>
#include <array>
#include "bicycle_model.hpp"
#include "formation_map.hpp"
#include "control_method.hpp"
#include "defs.hpp"
#include "mpi.h"

using namespace std;

void pack_robot_info(int robot_id, model_state_t cur_state, int current_common_formation_path_id, double* dest);

int main(int argc, char **argv) {
    // MPI init stuffs
    int my_id;
    int num_procs;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &my_id);

    // initialize
    int n_model = argc > 1 ? atoi(argv[1]) : 3;
    current_common_formation_path_idx = 1;
    num_robot_enter_formation_goal.resize(n_model);
    can_update = false;
    formation_map* fmp = new formation_map(n_model);

    fmp->cur_formation_goal_id = 1;

    vector<bicycle> robots(n_model);
    vector<FILE*> fps(n_model);
    vector<bool> common_formation_path(fmp->formation_path.size(), true);
    for (int i = 0; i < robots.size(); i++) {
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
        }
    }
    for (auto &robot: robots) {
        robot.common_formation_path_id_list = common_formation_path_id_list;
        robot.n_total_robots = n_model;
    }


    printf("%d\n", my_id);
    MPI_Barrier(MPI_COMM_WORLD);
    // master init
    // worker working
    if (my_id == 0) {
        char barrier_fn[100] = "./pic/barrier";
        FILE *fp = fopen(barrier_fn, "w");
        for (auto &barrier_point: fmp->get_formation_barrier()) {
            fprintf(fp, "%d\t%d\t%lf\n", barrier_point.x, barrier_point.y, barrier_point.cost);
        }
        fclose(fp);
        queue<int> worker_queue;
        queue<int> finished_worker_queue;
        vector<MPI_Request> reqs(num_procs);
        vector<array<double, 4>> recv_data(num_procs);
        int n_robot_this_round = 0;
        double data_send[5];
        int current_common_formation_path_idx = 1;
        for (int i = 1; i < num_procs; i++) {
            worker_queue.push(i);
        }
        while (current_common_formation_path_idx != common_formation_path_id_list.size() - 1) {
            if (n_robot_this_round == n_model) {
                current_common_formation_path_idx++;
                n_robot_this_round = 0;
            }
            // dispatch workers
            for (int i = n_robot_this_round; i < n_model && !worker_queue.empty(); i++) {
                int worker_id = worker_queue.front();
                worker_queue.pop();
                // prepare the data to be sent
                pack_robot_info(n_robot_this_round, robots[n_robot_this_round].cur_state, common_formation_path_id_list[current_common_formation_path_idx], data_send);
                MPI_Send(data_send, 5, MPI_DOUBLE, worker_id, 0, MPI_COMM_WORLD);
                // async receieve data from worker
                MPI_Irecv(&recv_data[worker_id], 4, MPI_DOUBLE, worker_id, 0, MPI_COMM_WORLD, &reqs[worker_id]);
                finished_worker_queue.push(worker_id);
                n_robot_this_round++;
            }
            // collect workers' results
            for (int i = 0; i < n_model && !finished_worker_queue.empty(); i++) {
                int worker_id = finished_worker_queue.front();
                finished_worker_queue.pop();
                MPI_Wait(&reqs[worker_id], MPI_STATUS_IGNORE);
                int rid = static_cast<int>(round(recv_data[worker_id][0]));
                robots[rid].cur_state.x = recv_data[worker_id][1];
                robots[rid].cur_state.y = recv_data[worker_id][2];
                robots[rid].cur_state.theta = recv_data[worker_id][3];
                worker_queue.push(worker_id);
            }
        }
        data_send[0] = -1;
        for (int i = 1; i < num_procs; i++) {
            MPI_Send(&data_send, 5, MPI_DOUBLE, i, 0, MPI_COMM_WORLD);
        }
    } else {
        while (true) {
            double recv_data[5];
            char robot_filename[100];
            MPI_Recv(recv_data, 5, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            int rid = static_cast<int>(round(recv_data[0]));
            // the end of the program
            if (rid == -1) {
                break;
            }
            sprintf(robot_filename, "./pic/robot%d", rid);
            FILE *fp = fopen(robot_filename, "a");
            // set start status from master
            robots[rid].cur_state.x = recv_data[1];
            robots[rid].cur_state.y = recv_data[2];
            robots[rid].cur_state.theta = recv_data[3];
            robots[rid].formation_goal_id = static_cast<int>(round(recv_data[4]));
            // initialize necessary start information
            robots[rid].cur_sensor_goal.x = robots[rid].cur_state.x;
            robots[rid].cur_sensor_goal.y = robots[rid].cur_state.y;
            robots[rid].update_sensor_goal(fmp->get_formation_barrier());
            robots[rid].update_control_goal();
            int i = 0;
            while (!robots[rid].formation_goal_needs_update()) {
                robots[rid].update_sensor_goal(fmp->get_formation_barrier());
                robots[rid].update_control_goal();
                robots[rid].model_move();
                if (i % 10 == 0) {
                    fprintf(fp, "%lf\t%lf\n", robots[rid].cur_state.x, robots[rid].cur_state.y);
                }
                i++;
            }
            double data_send[4];
            data_send[0] = rid;
            data_send[1] = robots[rid].cur_state.x;
            data_send[2] = robots[rid].cur_state.y;
            data_send[3] = robots[rid].cur_state.theta;
            fclose(fp);
            MPI_Send(data_send, 4, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD);
        }
    }

    MPI_Finalize();
    return 0;
}

void pack_robot_info(int robot_id, model_state_t cur_state, int current_common_formation_path_id, double* dest) {
    dest[0] = robot_id;
    dest[1] = cur_state.x;
    dest[2] = cur_state.y;
    dest[3] = cur_state.theta;
    dest[4] = current_common_formation_path_id;
}
