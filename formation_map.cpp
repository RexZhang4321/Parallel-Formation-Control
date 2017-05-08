//
//  formation_map.cpp
//  parallelProj
//
//  Created by Jingxuan Zhang on 4/26/17.
//  Copyright © 2017 Jingxuan Zhang. All rights reserved.
//

#include "formation_map.hpp"

formation_map::formation_map() : boundary_x(999), boundary_y(999), n_robot(4)  {
    // default formation shape
    formation_shape.push_back(formation_point_t(50, 50, 1));
    formation_shape.push_back(formation_point_t(50, -50, 1));
    formation_shape.push_back(formation_point_t(-50, -50, 1));
    formation_shape.push_back(formation_point_t(-50, 50, 1));
    
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
	for (int i = 0; i <= 2; i++) {
        formation_path.push_back(formation_point_t(499, i * 100 + 64, 0));
	}
    for (int i = 1; i <= 2; i++) {
        formation_path.push_back(formation_point_t(499 + i * 100, 264, 0));
    }
//    for (int i = 0; i <= 2; i++) {
//        formation_path.push_back(formation_point_t(499 + 2 * 100, i * 100 + 264, 0));
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
	// Below is the central barrier points with cost = -1 (no pass). y = 499
	for (int i = 0; i < N; i++) {
        formation_barrier.push_back(formation_point_t(499 - N / 2 + i, 499, -1));
	}

	// The points near barrier have a cost of N_cost;
	int N_cost = 1000;
	for (int i = 0; i < N; i++) {
		// up side without most left and right points
        formation_barrier.push_back(formation_point_t(formation_barrier[i].x, formation_barrier[i].y + 1, N_cost));
		// down side without most left and right points
        formation_barrier.push_back(formation_point_t(formation_barrier[i].x, formation_barrier[i].y - 1, N_cost));
	}
	for (int i = 0; i < 3; i++) {
		// left side
        formation_barrier.push_back(formation_point_t(formation_barrier[0].x - 1, formation_barrier[0].y + i - 1, N_cost));
		// right side
        formation_barrier.push_back(formation_point_t(formation_barrier[N - 1].x + 1, formation_barrier[N - 1].y + i -1, N_cost));
	}
}

vector<formation_point_t>
formation_map::get_formation_barrier() {
    return this->formation_barrier;
}

vector<formation_point_t>
formation_map::create_formation_shape(int num_model, formation_point_t &central_point) {
    vector<formation_point_t> t;
    return t;
}
