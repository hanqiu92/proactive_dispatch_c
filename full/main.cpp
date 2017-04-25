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
    int dynamic_travel_time_flag = 1;

    // get training results
    if (train_flag > 0){
        get_opt_param(0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0, 0, 0, 0, dynamic_travel_time_flag, 1);
    }

    // get simulation results
    if (test_flag > 0){
        //get_simulation_result(Algorithm::none,dynamic_travel_time_flag,"None","");
        get_simulation_result(Algorithm::full,dynamic_travel_time_flag,"Full", "");
        //get_simulation_result(Algorithm::single,dynamic_travel_time_flag,"Single", "");
        //get_simulation_result(Algorithm::pool,dynamic_travel_time_flag,"Pool", "");
        get_simulation_result(Algorithm::assort,dynamic_travel_time_flag,"Assort", "");
        get_simulation_result(Algorithm::pricing,dynamic_travel_time_flag,"Pricing", "");
        //get_simulation_result(Algorithm::assort_adjust,dynamic_travel_time_flag, "Assort Opt", "/Users/hanqiu/proactive_dispatch_c/result/assort_opt_param.csv");
        //get_simulation_result(Algorithm::pricing_adjust,dynamic_travel_time_flag,"Pricing Opt", "/Users/hanqiu/proactive_dispatch_c/result/price_opt_param.csv");
    }
}
