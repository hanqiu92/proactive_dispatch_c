//
//  main.cpp
//  test
//
//  Created by Han Qiu on 4/25/17.
//  Copyright © 2017 Han Qiu. All rights reserved.
//

#include <iostream>
#include "scenario.hpp"
#include "load.hpp"

int main(int argc, const char * argv[]) {
    if ((argc != 9) and (argc != 3)){
        std::cout<<"error!\n";
        std::cout<<argc<<"\n";
        return -1;
    }
    else{
        // load params
        std::vector<float> thetaa(12,0.0);
        std::vector<float> sigmaa(12,0.0);
        std::vector<float> muu(8,0.0);
        std::vector<float> kappaa(8,0.0);
        float alphaa = 0.0;
        Agent_Setting agent_setting = {1,false,{thetaa,sigmaa,muu,kappaa,alphaa}};
        std::vector<float> supply_factor_range = {0.75,1.0,1.25,1.5};
        std::vector<float> demand_factor_range = {1.0,2.0,4.0};
        std::vector<float> p_rate_range = {0.4,0.6,0.8};
        std::vector<float> tax_c_range = {0.0,0.025};
        std::vector<float> tax_d_range = {0.0,0.25};

        Algorithm algo = Algorithm::full;
        int i_algo = 2;
        std::string algo_name;
        int congest_level = 1;
        float congest_factor = 1.0;
        float supply_factor = 1.0;
        float demand_factor = 1.0;
        float p_rate = 0.5;
        float tax_congest = 0.0;
        float tax_demand = 0.0;
        if (argc == 9){
            // strategies without param
            i_algo = atoi(argv[2]);
            int i_congest = atoi(argv[3]);
            int i_demand = atoi(argv[4]);
            int i_supply = atoi(argv[5]);
            int i_prate = atoi(argv[6]);
            int i_tax_c = atoi(argv[7]);
            int i_tax_d = atoi(argv[8]);

            congest_level = i_congest + 1;
            demand_factor = demand_factor_range[i_demand];
            supply_factor = supply_factor_range[i_supply];
            p_rate = p_rate_range[i_prate];
            tax_congest = - tax_c_range[i_tax_c];
            tax_demand = - tax_d_range[i_tax_d];
        }
        else{
            // strategies with param
            std::ifstream f;
            std::string line;
            std::string cell;
            f.open(argv[2], std::ifstream::in);
            if (std::getline(f,line,'\n')){
                std::stringstream lineStream(line);

                std::getline(lineStream,cell,',');
                i_algo = std::stoi(cell);
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

                std::vector<float> in_theta_opt(12,0.0);
                for(int j = 0; j < 12; j++){
                    std::getline(lineStream,cell,',');
                    in_theta_opt[j] = std::stof(cell);
                }
                agent_setting.params.theta = in_theta_opt;

                if (std::abs(congest_factor - 1.0) < 0.01) congest_level = 1;
                if (std::abs(congest_factor - 0.8) < 0.01) congest_level = 2;
                if (std::abs(congest_factor - 1.2) < 0.01) congest_level = 3;
            }
            f.close();
        }
        switch (i_algo) {
            case 0:
                algo = Algorithm::pricing_adjust;
                algo_name = "Pricing Opt";
                break;
            case 1:
                algo = Algorithm::full;
                algo_name = "Full";
                break;
            case 2:
                algo = Algorithm::pricing;
                algo_name = "Pool";
                break;
            default:
                algo = Algorithm::full;
                algo_name = "Full";
                break;
        }

        // load simulation input
        std::string data_path = argv[1];
        int grid_size = 10;
        std::uniform_int_distribution<int> irand(0,grid_size * grid_size-1);
        int total_time = 1440;
        float congestion_factor;
        int dynamic_travel_time_flag = 1;
        float dynamic_travel_time_rate = demand_factor / 10.0;
        float demand_scale_factor = 10.0;
        int fleet_size = 100 * supply_factor * int(demand_scale_factor);
        std::vector< std::vector<float> > ori_dist = load(data_path+"/o_d_c.csv",total_time,grid_size,demand_scale_factor);
        std::vector< std::vector<float> > des_dist = load(data_path+"/d_d_c.csv",total_time,grid_size,demand_scale_factor);
        std::vector< std::vector<float> > travel_time;
        switch (congest_level) {
            case 1:
                congestion_factor = 1.0;
                travel_time = load(data_path+"/t_d_c_1_0.csv",total_time,grid_size,1.0);
                break;
            case 2:
                congestion_factor = 0.8;
                travel_time = load(data_path+"/t_d_c_0_8.csv",total_time,grid_size,1.0);
                break;
            case 3:
                congestion_factor = 1.2;
                travel_time = load(data_path+"/t_d_c_1_2.csv",total_time,grid_size,1.0);
                break;
            default:
                congestion_factor = 1.0;
                travel_time = load(data_path+"/t_d_c_1_0.csv",total_time,grid_size,1.0);
                break;
        }

        std::vector<VehicleState> vs;
        std::random_device rd;
        std::mt19937 mt(rd());
        vs.reserve(fleet_size);
        for (int i = 0; i < fleet_size; i++){
            vs.push_back({irand(mt),0,0,{}});
        }
        float density_factor = demand_scale_factor * (1.0 + 0.25 * (congestion_factor - 1.0));
        Scenario_Setting scenario_setting = {grid_size, p_rate, tax_congest, tax_demand, dynamic_travel_time_flag, dynamic_travel_time_rate, ori_dist, des_dist,travel_time,density_factor,demand_scale_factor,algo};
        Scenario s = Scenario(scenario_setting);

        // process test output
        simulate_output output;
        std::vector< pax_record > pax_out;
        agent_record final_out;

        // summarize results
        float revenue = 0.0;
        float cost = 0.0;
        float tax = 0.0;
        float profit = 0.0;
        int total_pax = 0;
        int pri_pax = 0;
        int pool_pax = 0;

        float sys_total_travel_time = 0.0;
        float sys_total_density = 0.0;

        int total_demand = 0;
        int fulfill_demand = 0;
        int stay_demand = 0;
        float total_travel_time = 0.0;
        float total_distance = 0.0;
        float total_delay = 0.0;
        float total_fulfill_delay = 0.0;
        float total_pickup = 0.0;

        int test_sample = 50;
        for (int i = 0; i < test_sample; i++){
            output = s.simulate(0,1440,fleet_size,vs,agent_setting);
            pax_out = output.pax_out;
            final_out = output.agent_out.back();
            // operator statistics
            revenue += final_out.revenue;
            cost += final_out.cost;
            tax += final_out.tax;
            profit += final_out.profit;
            total_pax += final_out.total_pax;
            pri_pax += final_out.private_pax;
            pool_pax += final_out.pool_pax;

            // system statistics
            sys_total_travel_time += output.system_out.back().total_travel_time;
            sys_total_density += output.system_out.back().total_density;

            // passenger statistics
            total_demand += int(pax_out.size());
            total_travel_time += final_out.total_travel_dist;
            total_distance += final_out.total_travel_time;
            for (int i = 0; i < int(pax_out.size()); i++){
                if ((pax_out[i].service_type == Mode::taxi) or (pax_out[i].service_type == Mode::pool)){
                    fulfill_demand += 1;
                    total_fulfill_delay += pax_out[i].delay_time;
                    total_pickup += pax_out[i].pickup_time;
                }
                if (pax_out[i].service_type == Mode::stay){
                    stay_demand += 1;
                }
                total_delay += pax_out[i].delay_time;
            }
        }
        float ave_delay = total_delay / total_demand;
        float ave_fulfill_delay = 0.0;
        float ave_pickup = 0.0;
        if (fulfill_demand > 0){
            ave_fulfill_delay = total_fulfill_delay / fulfill_demand;
            ave_pickup = total_pickup / fulfill_demand;
        }

        std::cout<<algo_name<<","<<congestion_factor<<","<<demand_factor<<","<<supply_factor<<","<<p_rate<<","<<tax_congest<<","<<tax_demand<<",,";
        std::cout<<revenue/test_sample<<","<<cost/test_sample<<","<<tax/test_sample<<","<<profit/test_sample<<","<<total_pax/test_sample<<","<<pri_pax/test_sample<<","<<pool_pax/test_sample<<","<<float(total_pax)/fleet_size/test_sample<<","<<total_travel_time/fleet_size/test_sample<<","<<total_distance/fleet_size/test_sample<<",";
        std::cout<<","<<sys_total_travel_time/sys_total_density<<","<<total_demand/test_sample<<","<<fulfill_demand/test_sample<<","<<stay_demand/test_sample<<","<<float(fulfill_demand + stay_demand)/float(total_demand)<<","<<ave_delay<<","<<ave_fulfill_delay<<","<<ave_pickup<<"\n";

        return 0;
    }
}
