//
//  scenario.cpp
//  simulator_c
//
//  Created by Han Qiu on 12/12/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#include "scenario.hpp"

int Scenario::TIME_TOTAL = 1440;

Scenario::Scenario(Scenario_Setting new_scenario_setting){
    grid_size = new_scenario_setting.grid_size;
    p_rate = new_scenario_setting.p_rate;
    tax_congest = new_scenario_setting.tax_congest;
    tax_demand = new_scenario_setting.tax_demand;
    ori_dist = new_scenario_setting.ori_dist;
    des_dist = new_scenario_setting.des_dist;
    travel_time = new_scenario_setting.travel_time;
    density_factor = new_scenario_setting.density_factor;
    demand_factor = new_scenario_setting.demand_factor;
    dynamic_travel_time_flag = new_scenario_setting.dynamic_travel_time_flag;
    dynamic_travel_time_rate = new_scenario_setting.dynamic_travel_time_rate;
    //routing module
    routing_module = new Routing(grid_size);
    Vehicle::setting(p_rate, *routing_module);
    Env::setting(grid_size, dynamic_travel_time_rate, dynamic_travel_time_flag, density_factor, *routing_module);
    Controller::setting(ori_dist,des_dist, demand_factor,new_scenario_setting.algorithm);
    controller_train = new Controller(0);
    controller_test = new Controller(1);
    Agent::setting(grid_size, p_rate, tax_congest, tax_demand, ori_dist, des_dist, demand_factor, *routing_module, *controller_train, *controller_test);
    // deal with demand
    demand_prob.reserve(TIME_TOTAL);
    std::vector<float> temp_ori_dist,temp_des_dist;
    float temp_ori_size,temp_des_size;
    int temp_size;
    for (int t = 0; t < TIME_TOTAL; t++){
        temp_ori_dist = ori_dist[t];
        temp_des_dist = des_dist[t];
        temp_ori_size = std::accumulate(temp_ori_dist.begin(),temp_ori_dist.end(),0.0);
        temp_des_size = std::accumulate(temp_des_dist.begin(),temp_des_dist.end(),0.0);
        temp_size = int((temp_ori_size + temp_des_size) / 2.0);
        for (int i = 0; i < grid_size * grid_size; i++){
            temp_ori_dist[i] = temp_ori_dist[i] / temp_ori_size;
            temp_des_dist[i] = temp_des_dist[i] / temp_des_size;
        }
        demand_prob.push_back({temp_ori_dist,temp_des_dist,temp_size});
    }
    
    // initialize random generator
    std::random_device rd;
    mt.seed(rd());
    irand = std::uniform_int_distribution<int>(0,9);
}

Scenario::~Scenario(){
    Env::clear();
    Agent::clear();
    Controller::clear();
    delete routing_module;
    delete controller_test;
    delete controller_train;
}

simulate_output Scenario::simulate(int new_start_time,int new_end_time,int fleet_size,std::vector<VehicleState> vehicle_state,Agent_Setting agent_setting){
    start_time = new_start_time;
    end_time = new_end_time;
    std::vector< std::vector< std::pair<int,int> > > demand = gen_demand();
    
    //travel time and vehicle setting
    Env_Setting env_setting = {fleet_size,vehicle_state,start_time,&demand,&travel_time};
            
    Agent *a = new Agent(env_setting,agent_setting);
    std::vector< std::vector<VehicleState> > new_state_list;

    for (int t = start_time; t < end_time; t++){
        a->next_step();
        if ((t < 1380) && (irand(mt))) new_state_list.push_back(a->get_veh_state());
    }
    
    // optional output
    std::vector< pax_record > pax_out;
    std::vector< agent_record > agent_out;
    std::vector< sys_record > system_out;
    pax_out = a->get_pax_record();
    agent_out = a->get_agent_record();
    system_out = a->get_sys_record();
    
    return {pax_out,agent_out,system_out,new_state_list};
}

std::vector< std::vector< std::pair<int,int> > > Scenario::gen_demand(){
    int t_size = end_time - start_time;
    std::vector< std::vector< std::pair<int,int> > > demand;
    demand.reserve(t_size);
    std::vector<float> o_dist,d_dist;
    std::vector< std::pair<int,int> > temp_demand;
    int d_size;

    for (int t = start_time; t < end_time; t++){
        o_dist = demand_prob[t].ori_dist;
        d_dist = demand_prob[t].des_dist;
        d_size = demand_prob[t].size;
        temp_demand = {};
        temp_demand.reserve(d_size);
        distribution_o = std::discrete_distribution<int>(o_dist.begin(),o_dist.end());
        distribution_d = std::discrete_distribution<int>(o_dist.begin(),o_dist.end());
        for (int i = 0; i < d_size; i++){
            temp_demand.push_back(std::make_pair(distribution_o(mt),distribution_d(mt)));
        }
        demand.push_back(temp_demand);
    }
    return demand;
}
