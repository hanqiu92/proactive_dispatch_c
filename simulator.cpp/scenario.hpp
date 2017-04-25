//
//  scenario.hpp
//  simulator_c
//
//  Created by Han Qiu on 12/12/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#ifndef scenario_hpp
#define scenario_hpp

#include "agent.hpp"

struct Scenario_Setting{
    int grid_size;
    float p_rate;
    float tax_congest;
    float tax_demand;
    int dynamic_travel_time_flag;
    float dynamic_travel_time_rate;
    std::vector< std::vector<float> > ori_dist;
    std::vector< std::vector<float> > des_dist;
    std::vector< std::vector<float> > travel_time;
    float density_factor;
    float demand_factor;
    Algorithm algorithm;
};

struct DemandProb{
    std::vector<float> ori_dist;
    std::vector<float> des_dist;
    int size;
};

struct simulate_output{
    std::vector< pax_record > pax_out;
    std::vector< agent_record > agent_out;
    std::vector< sys_record > system_out;
    std::vector< std::vector<VehicleState> > vehicle_state;
};

class Scenario{
    static int TIME_TOTAL;
    int grid_size;
    int start_time;
    int end_time;
    float p_rate;
    float tax_congest;
    float tax_demand;
    float dynamic_travel_time_rate;
    int dynamic_travel_time_flag;
    std::vector< std::vector<float> > ori_dist;
    std::vector< std::vector<float> > des_dist;
    std::vector<DemandProb> demand_prob;
    std::vector< std::vector<float> > travel_time;
    float density_factor;
    float demand_factor;
    Routing *routing_module;
    Controller *controller_train;
    Controller *controller_test;
    std::mt19937 mt;
    std::uniform_int_distribution<int> irand;
    std::discrete_distribution<int> distribution_o,distribution_d;
    
public:
    Scenario(Scenario_Setting new_scenario_setting);
    ~Scenario();
    simulate_output simulate(int new_start_time,int new_end_time,int fleet_size,std::vector<VehicleState> vehicle_state,Agent_Setting agent_setting);
    std::vector< std::vector< std::pair<int,int> > > gen_demand();
    
};


#endif /* scenario_hpp */
