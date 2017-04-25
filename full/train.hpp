//
//  train.hpp
//  simulator_c
//
//  Created by Han Qiu on 3/21/17.
//  Copyright © 2017 Han Qiu. All rights reserved.
//

#ifndef train_hpp
#define train_hpp

#include "scenario.hpp"
#include "load.hpp"

int train(Algorithm algo, float congestion_factor, float demand_factor, float supply_factor, float p_rate, float tax_congest, float tax_demand, int dynamic_time_flag, int debug_flag);

int get_opt_param(int algo_low, int algo_upp, int congestion_low, int congestion_upp, int demand_low, int demand_upp, int supply_low, int supply_upp, int p_rate_low, int p_rate_upp, int tax_c_low, int tax_c_upp, int tax_d_low, int tax_d_upp, int dynamic_time_flag, int debug_flag);

#endif /* train_hpp */
