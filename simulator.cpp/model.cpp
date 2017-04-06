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
    if (size == 0){
        return -1;
    }else{
        const float a = 10.0;
        const float b = 0.5;
        const float c = 5.0;
        const float d = 0.5;
        
        const float ASC_private = a + 1.0;
        const float ASC_pool = a;
        const float b_dist_private = 0.0;
        const float b_dist_pool = b;
        const float b_time_private = b;
        const float b_time_pool = 0.5 * b;
        const float b_fare_private = c;
        const float b_fare_pool = 2.0 * c;
        const float exit_preference = 0.0;
        
        // use simple MNL
        std::vector<float> prob(size,0.0);
        float maximum = 0.0;
        
        for (int i = 0; i < size; i++){
            Option temp = assort_list[i];
            float fare_ext = temp.adj_fare;
            
            if (temp.type == 1){
                prob[i] = (ASC_pool - b_dist_pool * (temp.dist - 5) -
                           b_time_pool * (temp.time - temp.dist) -
                           b_fare_pool * fare_ext) * d;
            }else{
                prob[i] = (ASC_private - b_dist_private * (temp.dist - 5) -
                           b_time_private * (temp.time - temp.dist) -
                           b_fare_private * fare_ext) * d;
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
}

