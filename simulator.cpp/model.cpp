//
//  model.cpp
//  simulator_c
//
//  Created by Han Qiu on 12/7/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#include "model.hpp"

std::random_device rd;
std::mt19937 mt(rd());
std::uniform_real_distribution<float> urand(0.0,1.0);

int model(std::vector<Option> assort_list, int size){
    const float a = 5.0;
    const float d = 0.5;
    const float ASC_private = a - 0.5;
    const float ASC_pool = a - 1.0;
    const float ASC_ori = a;
    const float b_time = 0.03;
    const float b_fare_ext = 2.0;
    const float b_fare_ori = 5.0;
    const float exit_preference = 0.0;

    // use simple MNL
    std::vector<float> prob(size,0.0);
    float maximum = 0.0;

    for (int i = 0; i < size; i++){
        Option temp = assort_list[i];

        switch (temp.type) {
            case Mode::pool:
                prob[i] = (ASC_pool - 1.2 * b_time * (temp.time + temp.pickup_time) - temp.fare -
                           temp.adj_fare * (temp.adj_fare < 0) -
                           b_fare_ext * temp.adj_fare * (temp.adj_fare > 0)) * d;
                break;
            case Mode::taxi:
                prob[i] = (ASC_private - b_time * (temp.time + temp.pickup_time) - temp.fare -
                           temp.adj_fare * (temp.adj_fare < 0) -
                           b_fare_ext * temp.adj_fare * (temp.adj_fare > 0)) * d;
                break;
            case Mode::stay:
                prob[i] = (ASC_ori - b_time * temp.time - b_fare_ori * temp.cost) * d;
                break;
            default:
                prob[i] = 0;
                break;
        }

        if (prob[i] > maximum){
            maximum = prob[i];
        }
    }

    float sum = 0.0;
    for (int i = 0; i < size; i++){
        prob[i] = exp(prob[i] - maximum);
        sum += prob[i];
    }
    sum += exp(exit_preference - maximum);
    for (int i = 0; i < size; i++){
        prob[i] = prob[i] / sum;
    }

    float realization = urand(mt);
    for (int i = 0; i < size; i++){
        if (prob[i] < realization) realization -= prob[i];
        else return i;
    }
    return -1;
}
