//
//  main.cpp
//  simulator_c
//
//  Created by Han Qiu on 12/7/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#include "train.hpp"
#include "test.hpp"

int main(int argc, const char * argv[]) {
    int train_flag = 0;
    int test_flag = 1;
    
    // get training results
    if (train_flag > 0){
        get_opt_param(1, 1, 0, 0, 1, 1, 0, 3, 0, 3, 0, 1, 0);
    }
    
    // get simulation results
    if (test_flag > 0){
        //get_simulation_result(Algorithm::full,"Full", "");
        //get_simulation_result(Algorithm::single,"Single", "");
        //get_simulation_result(Algorithm::pool,"Pool", "");
        //get_simulation_result(Algorithm::assort_adjust, "Assort Opt", "/Users/hanqiu/proactive_dispatch_c/result/assort_opt_param.csv");
        get_simulation_result(Algorithm::pricing_adjust, "Pricing Opt", "/Users/hanqiu/proactive_dispatch_c/result/price_opt_param.csv");
    }
}
