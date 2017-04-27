#include <stdio.h>
#include <stdlib.h>
#include "model.hpp"
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

    model_state_t* stat = new model_state_t;
    stat->x = 0.0;
    stat->y = 0.0;
    stat->theta = 0.0;
    
    control_input_t* input = new control_input_t;
    input->v = 1.0;
    
    robot_model* robot1 = new robot_model();

    int i;
    double alpha[] = {0, 0, 0, 0.5, 0, -0.5, 0, 0, 0, 0, 0};
    for (i = 0; i < 1000; i++) {
        if (i % 10 == 0)
            printf("%d, x: %.3lf, y: %.3lf, theta: %.3lf\n", i / 10 + 1, stat->x, stat->y, stat->theta);
        input->gamma = alpha[i / 100];
        robot1->model_move(input, stat);
    }
    #ifdef __USE_MPI__
    MPI_Finalize();
    #endif
    return 0;
}
