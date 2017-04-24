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
static const float U_D_COST = 0.1;
static const float U_T_COST = 0.0;
static const float U_D_FARE = 0.5;
static const float U_T_FARE = 0.02;
static const float BASE_COST = 1.0;
static const float BASE_FARE = 2.0;

static const int EMP_SEARCH_DIST = 5;
static const int AVI_SEARCH_DIST = 2;
static const int MAX_REROUTE_DIST_ALLOWED = 2;
static const float MAX_REROUTE_TIME_ALLOWED = 5.0;
static const int DEMAND_LIFE_PERIOD = 5;

enum class Mode {
    stay,taxi,pool,exit
};

struct Option{
    Mode type;
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

#endif /* common_hpp */
