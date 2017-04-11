//
//  test.hpp
//  simulator_c
//
//  Created by Han Qiu on 3/21/17.
//  Copyright © 2017 Han Qiu. All rights reserved.
//

#ifndef test_hpp
#define test_hpp

#include "scenario.hpp"
#include "load.hpp"

int test(Algorithm algo, std::string algo_name, int congestion_level, float demand_factor, float supply_factor, float p_rate, float tax_congest, float tax_demand,int dynamic_time_flag, Agent_Setting agent_setting);

int get_simulation_result(Algorithm algo,int dynamic_time_flag, std::string algo_name,std::string opt_param_path);

#endif /* test_hpp */
