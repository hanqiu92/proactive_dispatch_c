//
//  common.hpp
//  simulator_c
//
//  Created by Han Qiu on 12/7/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#ifndef common_hpp
#define common_hpp

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>
#include <set>
#include <array>
#include <random>

static const int MAX_CAPACITY = 3;
static const float U_D_COST = 0.2;
static const float U_T_COST = 0.2;
static const float U_D_FARE = 0.77;
static const float U_T_FARE = 0.2;
static const float BASE_COST = 1.0;
static const float BASE_FARE = 2.0;

static const int EMP_SEARCH_DIST = 5;
static const int AVI_SEARCH_DIST = 2;
static const int MAX_REROUTE_DIST_ALLOWED = 2;
static const float MAX_REROUTE_TIME_ALLOWED = 5.0;
static const int DEMAND_LIFE_PERIOD = 5;

struct Option{
    int type; // 0 is private, 1 is pool, -1 is null
    int veh_id;
    int dist;
    float time;
    int pickup_dist;
    float pickup_time;
    float fare;
    float cost;
    float adj_fare;
    float adj_cost;
    int insert;
};

struct RoutingOutput{
    float time;
    std::vector<int> route;
    int dist;
};

struct Des{
    int id;
    int type; // 0 private on, 1 private off, 2 pool on, 3 pool off
};

struct VehicleState{
    int loc;
    int status; // 0 empty, 1 private, 2 pool
    int curr_pax;
    std::vector<Des> des_cell;
};

#endif /* common_hpp */
