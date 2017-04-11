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
        get_opt_param(0, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 1);
    }
    
    // get simulation results
    if (test_flag > 0){
        //get_simulation_result(Algorithm::none,0,"None","");
        //get_simulation_result(Algorithm::full,0,"Full", "");
        //get_simulation_result(Algorithm::single,0,"Single", "");
        //get_simulation_result(Algorithm::pool,0,"Pool", "");
        //get_simulation_result(Algorithm::assort_adjust, "Assort Opt", "/Users/hanqiu/proactive_dispatch_c/result/assort_opt_param.csv");
        get_simulation_result(Algorithm::pricing_adjust,0,"Pricing Opt", "/Users/hanqiu/proactive_dispatch_c/result/price_opt_param.csv");
    }
}
