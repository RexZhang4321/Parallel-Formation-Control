//
//  formation_map.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "formation_map.hpp"

formation_map::formation_map(int n_model) : boundary_x(999), boundary_y(999), n_robot(n_model)  {
    // default formation shape
    create_formation_shape(n_model);
//    formation_shape.push_back(formation_point_t(50, 50, 1));
//    formation_shape.push_back(formation_point_t(50, -50, 1));
//    formation_shape.push_back(formation_point_t(-50, -50, 1));
//    formation_shape.push_back(formation_point_t(-50, 50, 1));
    
    // default formation path
    generate_formation_path();
    /*
    for (int i = 0; (i * 100 + 64) < this->boundary_y; i++) {
        formation_path.push_back(formation_point_t(499, i * 100 + 64, 1));
    }
    */
    
    // default formation barrier
    generate_formation_barrier();
    /*
    for (int x = 399; x <= 599; x++) {
        formation_barrier.push_back(formation_point_t(x, 299, -1));
    }
     */
    
    // default formation central point
    this->central_point = formation_path[0];
}

formation_map::formation_map(int bx, int by, int n_robot) {
    this->boundary_x = bx;
    this->boundary_y = by;
    this->n_robot = n_robot;
}

formation_map::~formation_map() {}

void
formation_map::generate_formation_path() {
	// x = 499
	int N = 10; // number of points on path
	for (int i = 0; i <= N; i++) {
        formation_path.push_back(formation_point_t(499, i * 100 + 64, 0));
	}
//    for (int i = 1; i <= 2; i++) {
//        formation_path.push_back(formation_point_t(499 + i * 100, 264, 0));
//    }
//    for (int i = 1; i <= 2; i++) {
//        formation_path.push_back(formation_point_t(499 + 200, 264 + i * 100, 0));
//    }
}

vector<formation_point_t>
formation_map::get_formation_path() {
    return this->formation_path;
}

void
formation_map::generate_formation_barrier() {
    // need to consider cost near the barrier
	/* #####
	   #ooo#
	   #####  */
	int N = 150; // length of central barrier
    int n_layer = 1;
	// Below is the central barrier points with cost = -1 (no pass). y = 499
    for (int nl = 1; nl <= n_layer; nl++) {
	for (int i = 0; i < N; i++) {
        formation_barrier.push_back(formation_point_t(499 - N / 2 + i, 499 * nl + nl - 1, -1));
	}
    }

	// The points near barrier have a cost of N_cost;
	int N_cost = 1000;
    for (int c = 1; c < 5; c++) {
	for (int i = 0; i < N * n_layer; i++) {
		// up side without most left and right points
        formation_barrier.push_back(formation_point_t(formation_barrier[i].x, formation_barrier[i].y + c, N_cost));
		// down side without most left and right points
        formation_barrier.push_back(formation_point_t(formation_barrier[i].x, formation_barrier[i].y - c, N_cost));
	}
    }
    for (int nl = 0; nl < n_layer; nl++) {
    for (int c = 1; c < 3; c++) {
	for (int i = 0; i < 3; i++) {
		// left side
        formation_barrier.push_back(formation_point_t(formation_barrier[N * nl].x - c, formation_barrier[N * nl].y + i - 1, N_cost));
		// right side
        formation_barrier.push_back(formation_point_t(formation_barrier[N * (nl + 1) - 1].x + c, formation_barrier[N * (nl + 1) - 1].y + i -1, N_cost));
	}
    }
    }
}

vector<formation_point_t>
formation_map::get_formation_barrier() {
    return this->formation_barrier;
}

void
formation_map::create_formation_shape(int num_model) {
    int R = 50;
    double avg_angle_div_2 = PI / num_model;
    double angle;
    int if_odd = !(num_model % 2);
    for (int i = 0; i < num_model; i++) {
        angle = PI / 2 + (2 * i + if_odd) * avg_angle_div_2;
        formation_shape.push_back(formation_point_t(round(R * cos(angle)),
                                                    round(R * sin(angle)),
                                                    0));
    }
}

void
formation_map::create_formation_shape_sqr(int num_model) {
    int width = ceil(sqrt(num_model));
    int scale = 100 / width;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < width; j++) {
            formation_shape.push_back(formation_point_t(scale*(-width / 2 + j), scale * (width / 2 - i), 0));
        }
    }
}
