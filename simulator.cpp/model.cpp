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
    const float a = 10.0;
    const float b = 0.002;
    const float c = 0.2;
    const float d = 0.5;
    const float e = 2.0;
    const float f = 5.0;
    
    const float ASC_private = a - 1.0;
    const float ASC_pool = a - 2.0;
    const float ASC_ori = a;
    const float b_dist_private = b;
    const float b_dist_pool = b * 2;
    const float b_time_private = c;
    const float b_time_pool = c * 1.5;
    const float b_time_ori = c;
    const float b_fare_ext_private = e;
    const float b_fare_ext_pool = e;
    const float b_fare_ori = f;
    const float exit_preference = 0.0;
    
    // use simple MNL
    std::vector<float> prob(size,0.0);
    float maximum = 0.0;
    
    for (int i = 0; i < size; i++){
        Option temp = assort_list[i];
        
        switch (temp.type) {
            case Mode::pool:
                prob[i] = (ASC_pool - b_dist_pool * temp.dist -
                           b_time_pool * temp.time - temp.fare - temp.adj_fare * (temp.adj_fare < 0) -
                           b_fare_ext_pool * temp.adj_fare * temp.adj_fare * (temp.adj_fare > 0)) * d;
                break;
            case Mode::taxi:
                prob[i] = (ASC_private - b_dist_private * temp.dist -
                           b_time_private * temp.time - temp.fare - temp.adj_fare * (temp.adj_fare < 0) -
                           b_fare_ext_private * temp.adj_fare * temp.adj_fare * (temp.adj_fare > 0)) * d;
                break;
            case Mode::stay:
                prob[i] = (ASC_ori - b_time_ori * temp.time - b_fare_ori * temp.cost) * d;
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

