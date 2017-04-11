//
//  test.cpp
//  simulator_c
//
//  Created by Han Qiu on 3/21/17.
//  Copyright © 2017 Han Qiu. All rights reserved.
//

#include "test.hpp"

int test(Algorithm algo, std::string algo_name, int congestion_level, float demand_factor, float supply_factor, float p_rate, float tax_congest, float tax_demand, int dynamic_time_flag, Agent_Setting agent_setting){
    // load simulation input
    int grid_size = 10;
    std::uniform_int_distribution<int> irand(0,grid_size * grid_size-1);
    int total_time = 1440;
    float congestion_factor;
    int dynamic_travel_time_flag = dynamic_time_flag;
    float dynamic_travel_time_rate = demand_factor / 30.0;
    int fleet_size = 200 * supply_factor;
    std::vector< std::vector<float> > ori_dist = load("/Users/hanqiu/proactive_dispatch_c/data/o_d_c.csv",total_time,grid_size);
    std::vector< std::vector<float> > des_dist = load("/Users/hanqiu/proactive_dispatch_c/data/d_d_c.csv",total_time,grid_size);
    std::vector< std::vector<float> > travel_time;
    switch (congestion_level) {
        case 1:
            congestion_factor = 1.0;
            travel_time = load("/Users/hanqiu/proactive_dispatch_c/data/t_d_c_1_0.csv",total_time,grid_size);
            break;
        case 2:
            congestion_factor = 0.8;
            travel_time = load("/Users/hanqiu/proactive_dispatch_c/data/t_d_c_0_8.csv",total_time,grid_size);
            break;
        case 3:
            congestion_factor = 1.2;
            travel_time = load("/Users/hanqiu/proactive_dispatch_c/data/t_d_c_1_2.csv",total_time,grid_size);
            break;
        default:
            congestion_factor = 1.0;
            travel_time = load("/Users/hanqiu/proactive_dispatch_c/data/t_d_c_1_0.csv",total_time,grid_size);
            break;
    }
    
    std::vector<VehicleState> vs;
    std::random_device rd;
    std::mt19937 mt(rd());
    vs.reserve(fleet_size);
    for (int i = 0; i < fleet_size; i++){
        vs.push_back({irand(mt),0,0,{}});
    }
    Scenario_Setting scenario_setting = {grid_size, p_rate, tax_congest, tax_demand, dynamic_travel_time_flag, dynamic_travel_time_rate, ori_dist, des_dist,travel_time,algo};
    Scenario s = Scenario(scenario_setting);
    
    // process test output
    int test_sample = 1;
    std::string save_path = "/Users/hanqiu/proactive_dispatch_c/result/results.csv";
    std::ofstream f;
    simulate_output output;
    std::vector< pax_record > pax_out;
    agent_record final_out;
    
    f.open(save_path, std::ios::out|std::ios::app);
    if (f.is_open()) f<<algo_name<<","<<congestion_factor<<","<<demand_factor<<","<<supply_factor<<","<<p_rate<<","<<tax_congest<<","<<tax_demand<<",,";
    for (int i = 0; i < test_sample; i++){
        output = s.simulate(0,1440,fleet_size,vs,agent_setting);
        pax_out = output.pax_out;
        // format output
        final_out = output.agent_out.back();
        int total_demand = int(pax_out.size());
        int fulfill_demand = 0;
        int stay_demand = 0;
        float ave_travel_time = final_out.total_travel_dist/float(fleet_size);
        float ave_distance = final_out.total_travel_time/float(fleet_size);
        float ave_delay = 0.0;
        float ave_fulfill_delay = 0.0;
        float ave_pickup = 0.0;
        for (int i = 0; i < total_demand; i++){
            if ((pax_out[i].service_type == Mode::taxi) or (pax_out[i].service_type == Mode::pool)){
                fulfill_demand += 1;
                ave_fulfill_delay += pax_out[i].delay_time;
                ave_pickup += pax_out[i].pickup_time;
            }
            if (pax_out[i].service_type == Mode::stay){
                stay_demand += 1;
            }
            ave_delay += pax_out[i].delay_time;
        }
        ave_delay = ave_delay / total_demand;
        if (fulfill_demand > 0){
            ave_fulfill_delay = ave_fulfill_delay / fulfill_demand;
            ave_pickup = ave_pickup / fulfill_demand;
        }
        if (f.is_open()){
            f<<final_out.revenue<<","<<final_out.cost<<","<<final_out.tax<<","<<final_out.profit<<","<<final_out.total_pax<<","<<final_out.private_pax<<","<<final_out.pool_pax<<","<<float(final_out.total_pax)/fleet_size<<","<<final_out.assort_type.both<<","<<final_out.assort_type.pri<<","<<final_out.assort_type.pool<<","<<ave_travel_time<<","<<ave_distance<<",,"<<output.system_out.back().average_travel_time<<","<<total_demand<<","<<fulfill_demand<<","<<stay_demand<<","<<float(fulfill_demand)/float(total_demand)<<","<<ave_delay<<","<<ave_fulfill_delay<<","<<ave_pickup<<"\n";
        }
        
    }
    f.close();
    
    return 0;
}

int get_simulation_result(Algorithm algo,int dynamic_time_flag, std::string algo_name,std::string opt_param_path){
    std::cout<<algo_name<<": \n";
    
    std::vector<float> thetaa(8,0.0);
    std::vector<float> sigmaa(8,0.0);
    std::vector<float> muu(8,0.0);
    std::vector<float> kappaa(8,0.0);
    float alphaa = 0.0;
    Agent_Setting agent_setting = {1,false,{thetaa,sigmaa,muu,kappaa,alphaa}};
    
    std::vector<float> in_theta_opt(8,0.0);
    int opt_input_flag = 0;
    if ((algo == Algorithm::assort_adjust) or (algo == Algorithm::pricing_adjust)) opt_input_flag = 1;
    
    std::vector<float> supply_factor_range = {0.25,0.5,1.0,1.5};
    std::vector<float> demand_factor_range = {1.0,3.0,6.0};
    std::vector<float> p_rate_range = {0.5,0.6,0.7,0.8};
    
    int congest_level = 1;
    float congest_factor = 1.0;
    float supply_factor = 1.0;
    float demand_factor = 1.0;
    float p_rate = 0.8;
    float tax_congest = -0.1;
    float tax_demand = -0.5;
    
    if (opt_input_flag > 0){
        agent_setting.detail_output_flag = true;
        std::ifstream f;
        std::string line;
        std::string cell;
        f.open(opt_param_path, std::ifstream::in);
        while (std::getline(f,line,'\n')){
            std::stringstream lineStream(line);
            
            std::getline(lineStream,cell,',');
            congest_factor = std::stof(cell);
            std::getline(lineStream,cell,',');
            demand_factor = std::stof(cell);
            std::getline(lineStream,cell,',');
            supply_factor = std::stof(cell);
            std::getline(lineStream,cell,',');
            p_rate = std::stof(cell);
            std::getline(lineStream,cell,',');
            tax_congest = std::stof(cell);
            std::getline(lineStream,cell,',');
            tax_demand = std::stof(cell);
            
            for(int j = 0; j < 8; j++){
                std::getline(lineStream,cell,',');
                in_theta_opt[j] = std::stof(cell);
            }
            
            agent_setting.params.theta = in_theta_opt;
            if (std::abs(congest_factor - 1.0) < 0.01) congest_level = 1;
            if (std::abs(congest_factor - 0.8) < 0.01) congest_level = 2;
            if (std::abs(congest_factor - 1.2) < 0.01) congest_level = 3;
            std::cout<<"Scenario with "<<congest_level<<","<<supply_factor<<","<<demand_factor<<","<<p_rate<<","<<tax_congest<<","<<tax_demand<<": ";
            test(algo,algo_name,congest_level,demand_factor,supply_factor,p_rate,tax_congest,tax_demand,dynamic_time_flag,agent_setting);
            std::cout<<"Done. \n";
        }
        f.close();
    }
    else{
        for (int i_congest = 0; i_congest <= 2; i_congest++){
            congest_level = i_congest + 1;
            for (int i_demand = 0; i_demand <= 0; i_demand++){
                demand_factor = demand_factor_range[i_demand];
                for (int i_supply = 0; i_supply <= 3; i_supply++){
                    supply_factor = supply_factor_range[i_supply];
                    for (int i_prate = 0; i_prate <= 3; i_prate++){
                        p_rate = p_rate_range[i_prate];
                        for (int i_tax = 0; i_tax <= 1; i_tax++){
                            switch (i_tax) {
                                case 0:
                                    tax_congest = 0.0;
                                    tax_demand = 0.0;
                                    break;
                                case 1:
                                    tax_congest = -0.1;
                                    tax_demand = -0.5;
                                    break;
                                default:
                                    tax_congest = 0.0;
                                    tax_demand = 0.0;
                                    break;
                            }
                            
                            std::cout<<"Scenario with "<<algo_name<<","<<congest_level<<","<<supply_factor<<","<<demand_factor<<","<<p_rate<<","<<tax_congest<<","<<tax_demand<<": ";
                            test(algo,algo_name,congest_level,demand_factor,supply_factor,p_rate,tax_congest,tax_demand,dynamic_time_flag,agent_setting);
                            std::cout<<"Done. \n";
                        }
                    }
                }
            }
        }
    }
    
    return 0;
}
