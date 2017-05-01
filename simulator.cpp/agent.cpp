//
//  agent.cpp
//  simulator_c
//
//  Created by Han Qiu on 12/12/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#include "agent.hpp"

int Agent::grid_size = 0;
float Agent::p_rate = 1.0;
float Agent::tax_congest = 0.0;
float Agent::bound_congest = 1.5;
float Agent::tax_demand = 0.0;
float Agent::bound_demand = 7.0;
float Agent::demand_norm_factor = 1.0;
Routing *Agent::routing_module = nullptr;
Controller *Agent::train_controller = nullptr;
Controller *Agent::test_controller = nullptr;
std::vector< std::vector<float> > *Agent::ori_dist = nullptr;
std::vector< std::vector<float> > *Agent::des_dist = nullptr;

float Agent::fare_cal(int dist, float time, int typ){
    switch (typ) {
        case 0:
            return (BASE_FARE + U_D_FARE * dist + U_T_FARE * time);
            break;
        case 1:
            return (BASE_FARE + U_D_FARE * dist + U_T_FARE * time) * p_rate;
            break;
        default:
            return 0.0;
            break;
    }
}

float Agent::cost_cal(int dist, float time, int typ){
    return U_D_COST * dist;
}

float Agent::adj_cost_cal(int ori, int des, int dist, float time, int typ){
    return float(typ) * ((time - dist * (1.0 + bound_congest)) * (time > dist * (1.0 + bound_congest)) * tax_congest + (((*des_dist)[curr_time][ori] + (*ori_dist)[curr_time][des]) * demand_norm_factor - bound_demand) * (((*ori_dist)[curr_time][ori] + (*ori_dist)[curr_time][des]) * demand_norm_factor > bound_demand) * tax_demand);
}

void Agent::setting(int new_grid_size,float new_p_rate,float new_tax_congest,float new_tax_demand,std::vector< std::vector<float> > &new_ori_dist,std::vector< std::vector<float> > &new_des_dist, float demand_factor,Routing &new_routing_module,Controller &new_train_controller, Controller&new_test_controller){
    grid_size = new_grid_size;
    p_rate = new_p_rate;
    tax_congest = new_tax_congest;
    tax_demand = new_tax_demand;
    ori_dist = &new_ori_dist;
    des_dist = &new_des_dist;
    routing_module = &new_routing_module;
    train_controller = &new_train_controller;
    test_controller = &new_test_controller;
    demand_norm_factor = float(ori_dist->size()) * float((*ori_dist)[0].size()) / 10000.0 / demand_factor;
}

void Agent::clear(){
    routing_module = nullptr;
    train_controller = nullptr;
    test_controller = nullptr;
    ori_dist = nullptr;
    des_dist = nullptr;
}

Agent::Agent(Env_Setting env_setting,Agent_Setting agent_setting){
    //for simulation purpose
    fleet_size = env_setting.fleet_size;
    curr_time = env_setting.start_time;
    env = new Env(env_setting);
    fleet = env->get_fleet();
    if (agent_setting.status == 0) {
        controller = train_controller;
    }else{
        controller = test_controller;
    }
    controller->reset(curr_time,agent_setting.params);
            
    //for analysis purpose
    detail_output_flag = agent_setting.detail_output_flag;
    freq_counter = {0,0,0,0};
    assort_type_stat = {0,0,0};
}

Agent::~Agent(){
    delete env;
    controller = nullptr;
}

void Agent::next_step(){
    //update controller status
    controller->update_global_state(*env);
    
    //get the demand and vehicle location
    std::vector< std::array<int, 3> > new_hist_demand;
    int demand_size;
    int match_size = 0;
    int ori;
    int des;
    int life_time;
    int flag = 1;
    
    //process the history demand
    demand_size = int(history_demand.size());
    for (int i = 0; i < demand_size; i++){
        ori = history_demand[i][0];
        des = history_demand[i][1];
        life_time = history_demand[i][2];
        if (ori != des){
            flag = process_demand(ori, des, curr_time, life_time);
            switch (flag) {
                case 0:
                    if (life_time < DEMAND_LIFE_PERIOD){
                        new_hist_demand.push_back({ori,des,life_time+1});
                    }
                    else{
                        process_demand(ori, des, curr_time, life_time+1);
                    }
                    break;
                case 1:
                    match_size += 1;
                    break;
                default:
                    break;
            }
        }
    }
    
    //process the current demand
    std::vector< std::pair<int, int> > curr_demand = env->get_curr_demand();
    demand_size = int(curr_demand.size());
    for (int i = 0; i < demand_size; i++){
        ori = std::get<0>(curr_demand[i]);
        des = std::get<1>(curr_demand[i]);
        if (ori != des){
            flag = process_demand(ori, des, curr_time, 0);
            switch (flag) {
                case 0:
                    new_hist_demand.push_back({ori,des,1});
                    break;
                case 1:
                    match_size += 1;
                    break;
                default:
                    break;
            }
        }
    }
    history_demand = new_hist_demand;
    
    //finally do the move
    env->next_step();
    curr_time += 1;
    controller->update_curr_time();
                    
    //summarize the major statistics and save veh state
    float revenue = 0.0;
    float cost = 0.0;
    float tax = 0.0;
    int total_travel_time = 0;
    int total_travel_dist = 0;
    int total_pax = 0;
    int private_pax = 0;
    int pool_pax = 0;
    for (int i = 0; i < fleet_size; i++){
        revenue += fleet[i]->get_total_revenue();
        cost += fleet[i]->get_total_cost();
        tax += fleet[i]->get_total_tax();
        total_travel_time += fleet[i]->get_total_time();
        total_travel_dist += fleet[i]->get_total_dist();
        total_pax += fleet[i]->get_total_pax();
        private_pax += fleet[i]->get_private_pax();
        pool_pax += fleet[i]->get_pool_pax();
    }
    agent_stat.push_back({revenue,cost,tax,revenue - cost - tax,total_travel_time,total_travel_dist,demand_size,match_size,total_pax,private_pax,pool_pax,assort_type_stat});
}

int Agent::process_demand(int ori,int des,int curr_time,int delay_time){
    std::vector<Option> assortment,opt_assortment;
    int private_flag = 0;
    int pool_flag = 0;
    //final out; only need two possible option
    if (delay_time > DEMAND_LIFE_PERIOD){
        assortment = gene_assort(ori, des);
        opt_assortment.push_back(assortment[0]);
        //call the external model to decide the selection
        int choice_ind = model(opt_assortment,int(opt_assortment.size()));
        if (choice_ind >= 0){
            pax_stat.push_back({curr_time-delay_time,curr_time,delay_time,Mode::stay,0,float(std::abs(ori / grid_size - des / grid_size) + std::abs(ori % grid_size - des % grid_size)),0,0,0});
            env->update_usual_traffic(ori,des);
        }else{
            pax_stat.push_back({curr_time-delay_time,curr_time,delay_time,Mode::exit,0,float(std::abs(ori / grid_size - des / grid_size) + std::abs(ori % grid_size - des % grid_size)),0,0,0});
        }
    }else{
        //base on different purpose, add time counter
        if (detail_output_flag){
            //generate the assortment
            assortment = gene_assort(ori, des);
            freq_counter[0] += 1;
            if (int(assortment.size()) > 1){
                //now optimize the assortment
                opt_assortment = (controller->*(controller->optimize))(assortment, ori, des);
                //opt_assortment = assortment;
                freq_counter[1] += 1;
                if (!opt_assortment.empty()){
                    //calculate property of the opt assortment
                    for (int i = 0; i < opt_assortment.size(); i++){
                        if (opt_assortment[i].type == Mode::taxi) private_flag = 1;
                        if (opt_assortment[i].type == Mode::pool) pool_flag = 1;
                    }
                    if ((private_flag == 1) and (pool_flag == 1)) assort_type_stat.both += 1;
                    if ((private_flag == 1) and (pool_flag == 0)) assort_type_stat.pri += 1;
                    if ((private_flag == 0) and (pool_flag == 1)) assort_type_stat.pool += 1;
                    //call the external model to decide the selection
                    int choice_ind = model(opt_assortment,int(opt_assortment.size()));
                    freq_counter[2] += 1;
                    if (choice_ind >= 0){
                        if (opt_assortment[choice_ind].type == Mode::stay){
                            pax_stat.push_back({curr_time-delay_time,curr_time,delay_time,Mode::stay,0,float(std::abs(ori / grid_size - des / grid_size) + std::abs(ori % grid_size - des % grid_size)),0,0,0});
                            env->update_usual_traffic(ori,des);
                            return -1;
                        }else{
                            update(ori,des,opt_assortment[choice_ind],curr_time,delay_time);
                            freq_counter[3] += 1;
                            return 1;
                        }
                    }
                    pax_stat.push_back({curr_time - delay_time,curr_time,delay_time,Mode::exit,0,float(std::abs(ori / grid_size - des / grid_size) + std::abs(ori % grid_size - des % grid_size)),0,0,0});
                    return -1;
                }
            }
        }else{
            //generate the assortment
            assortment = gene_assort(ori, des);
            if (int(assortment.size()) > 1){
                //now optimize the assortment
                opt_assortment = (controller->*(controller->optimize))(assortment, ori, des);
                if (!opt_assortment.empty()){
                    //call the external model to decide the selection
                    int choice_ind = model(opt_assortment,int(opt_assortment.size()));
                    if (choice_ind >= 0){
                        if (opt_assortment[choice_ind].type == Mode::stay){
                            pax_stat.push_back({curr_time-delay_time,curr_time,delay_time,Mode::stay,0,float(std::abs(ori / grid_size - des / grid_size) + std::abs(ori % grid_size - des % grid_size)),0,0,0});
                            env->update_usual_traffic(ori,des);
                            return -1;
                        }else{
                            update(ori,des,opt_assortment[choice_ind],curr_time,delay_time);
                            return 1;
                        }
                    }
                    pax_stat.push_back({curr_time-delay_time,curr_time,delay_time,Mode::exit,0,float(std::abs(ori / grid_size - des / grid_size) + std::abs(ori % grid_size - des % grid_size)),0,0,0});
                    return -1;
                }
            }
        }
    }
    
    return 0;
}

std::vector<Option> Agent::gene_assort(int ori,int des){
    int n = grid_size;
    std::vector<float> travel_time = env->get_travel_time();
    std::vector<int> emp_veh_location = env->get_emp_veh_location();
    std::vector<int> avi_veh_location = env->get_avi_veh_location();
    
    std::vector<Option> assortment;
    assortment.reserve(5);
    std::vector<int>::iterator it;
    //*****************************************************
    //search for both vehicles
    std::vector<int> emp_veh_list;
    std::vector<int> avi_veh_list;
    std::vector<int> dist_emp;
    std::vector<int> dist_avi;
    int dist_emp_min;
    int dist_avi_min;
    dist_emp.reserve(fleet_size);
    dist_avi.reserve(fleet_size);
    dist_emp_min = grid_size * 2;
    dist_avi_min = grid_size * 2;
    for (int i = 0; i < fleet_size; i++){
        dist_emp.push_back(abs(emp_veh_location[i] / grid_size - ori / grid_size) +
                           abs(emp_veh_location[i] % grid_size - ori % grid_size));
        dist_avi.push_back(abs(avi_veh_location[i] / grid_size - ori / grid_size) +
                           abs(avi_veh_location[i] % grid_size - ori % grid_size));
        if (dist_emp_min > dist_emp[i]) dist_emp_min = dist_emp[i];
        if (dist_avi_min > dist_avi[i]) dist_avi_min = dist_avi[i];
    }
    
    if (dist_emp_min < EMP_SEARCH_DIST){
        for (int i = 0; i < fleet_size; i++){
            if (dist_emp[i] == dist_emp_min) emp_veh_list.push_back(i);
        }
    }
    if (dist_avi_min < AVI_SEARCH_DIST){
        for (int i = 0; i < fleet_size; i++){
            if (dist_avi[i] <= (dist_avi_min + 1)) avi_veh_list.push_back(i);
        }
    }
    //*****************************************************
    float pri_fare,pool_fare,pri_cost,pool_cost,pri_on_cost,pool_on_cost;
    float pri_adj_cost,pool_adj_cost;
    int pool_dist;
    float pool_time;
    
    // first include stay option
    RoutingOutput stay_estimate = routing_module->estimate(ori, des);
    float stay_cost = cost_cal(stay_estimate.dist, stay_estimate.time, -1);
    assortment.push_back({Mode::stay,-1,stay_estimate.dist,stay_estimate.time,0,0,0,stay_cost,0,0,-1});
    //*****************************************************
    //process the vehicle list to find the best private/pool option
    if (!emp_veh_list.empty()){
        int best_veh = -1;
        float btime = n * 2;
        RoutingOutput new_estimate = routing_module->estimate(ori, des);
        pri_fare = fare_cal(new_estimate.dist, new_estimate.time, 0);
        pool_fare = fare_cal(new_estimate.dist, new_estimate.time, 1);
        pri_cost = cost_cal(new_estimate.dist, new_estimate.time, 0);
        pool_cost = cost_cal(new_estimate.dist, new_estimate.time, 1);
        pri_adj_cost = adj_cost_cal(ori, des, new_estimate.dist, new_estimate.time, 0);
        pool_adj_cost = adj_cost_cal(ori, des, new_estimate.dist, new_estimate.time, 1);
        
        // search for the vehicle with best start condition
        for (it = emp_veh_list.begin(); it != emp_veh_list.end(); ++it){
            int this_veh_loc = emp_veh_location[*it];
            if ((this_veh_loc >= 0) && (travel_time[this_veh_loc] < btime)){
                btime = travel_time[this_veh_loc];
                best_veh = *it;
            }
        }
                    
        if (best_veh >= 0){
            int best_veh_loc = emp_veh_location[best_veh];
            RoutingOutput new_estimate_pre = routing_module->estimate(best_veh_loc, ori);
            pri_on_cost = cost_cal(new_estimate_pre.dist, new_estimate_pre.time, -1);
            pool_on_cost = cost_cal(new_estimate_pre.dist, new_estimate_pre.time, -1);
            
            assortment.push_back({Mode::taxi,best_veh,new_estimate.dist,new_estimate.time,
                new_estimate_pre.dist,new_estimate_pre.time,
                pri_fare,pri_cost + pri_on_cost + pri_adj_cost,0,pri_adj_cost,-1});
            
            assortment.push_back({Mode::pool,best_veh,new_estimate.dist,new_estimate.time,
                new_estimate_pre.dist,new_estimate_pre.time,
                pool_fare,pool_cost + pool_on_cost + pri_adj_cost,0,pool_adj_cost,0});
        }
    }
    
    //*****************************************************
    if (!avi_veh_list.empty()){
        RoutingOutput new_estimate = routing_module->estimate(ori, des);
        pool_adj_cost = adj_cost_cal(ori, des, new_estimate.dist, new_estimate.time, 1);
        if (avi_veh_list.size() > 5)   std::random_shuffle(avi_veh_list.begin(),avi_veh_list.end());
        for (it = avi_veh_list.begin(); it != avi_veh_list.end(); ++it){
            if ((avi_veh_location[*it] > 0) and (assortment.size() < 5)){
                Vehicle *this_veh = fleet[*it];
                std::vector<Des> des_list = this_veh->get_dest();
                //to simplify, always go to pick up the new pax
                RoutingOutput new_estimate_pre = routing_module->estimate(this_veh->get_loc(), ori);
                pool_on_cost = cost_cal(new_estimate_pre.dist, new_estimate_pre.time, -1);
                if (this_veh->get_pool_remain() >= 2){
                    //only one previous pax, easier
                    int last_des = des_list.back().id;
                    
                    int ori_x,des_x,last_des_x,ori_y,des_y,last_des_y;
                    //get rid of simple bad case to reduce computation
                    if (ori / grid_size > last_des / grid_size){
                        ori_x = n - 1 - ori / grid_size;
                        des_x = n - 1 - des / grid_size;
                        last_des_x = n - 1 - last_des / grid_size;
                    }else{
                        ori_x = ori / grid_size;
                        des_x = des / grid_size;
                        last_des_x = last_des / grid_size;
                    }
                    
                    if (ori % grid_size > last_des % grid_size){
                        ori_y = n - 1 - ori % grid_size;
                        des_y = n - 1 - des % grid_size;
                        last_des_y = n - 1 - last_des % grid_size;
                    }else{
                        ori_y = ori % grid_size;
                        des_y = des % grid_size;
                        last_des_y = last_des % grid_size;
                    }
                    
                    bool in_region_flag = ((des_x >= (ori_x - MAX_REROUTE_DIST_ALLOWED)) &&
                                           (des_x <= (last_des_x + MAX_REROUTE_DIST_ALLOWED)) &&
                                           (des_y >= (ori_y - MAX_REROUTE_DIST_ALLOWED)) &&
                                           (des_y <= (last_des_y + MAX_REROUTE_DIST_ALLOWED)) ) ||
                                          ((des_x >= (last_des_x - MAX_REROUTE_DIST_ALLOWED)) &&
                                           (des_y >= (last_des_y - MAX_REROUTE_DIST_ALLOWED)));
                    
                    if (in_region_flag){
                        //in the correct region
                        RoutingOutput estimate_ori = routing_module->estimate(ori, last_des);
                        RoutingOutput estimate_delta = routing_module->estimate(des, last_des);
                        if (std::abs(std::abs(new_estimate.time - estimate_ori.time) - estimate_delta.time) < MAX_REROUTE_TIME_ALLOWED){
                            //no much rerouting; keep considering
                            if (estimate_ori.time > new_estimate.time){
                                //get to new dest first
                                pool_dist = new_estimate.dist;
                                pool_time = new_estimate.time;
                                pool_fare = fare_cal(pool_dist, pool_time, 1);
                                pool_cost = cost_cal(std::abs(new_estimate.dist + estimate_delta.dist - estimate_ori.dist), std::abs(new_estimate.time + estimate_delta.time - estimate_ori.time), 1);
                                assortment.push_back({Mode::pool,*it,pool_dist,pool_time,
                                    new_estimate_pre.dist,new_estimate_pre.time,
                                    pool_fare,pool_cost + pool_on_cost + pool_adj_cost,0,pool_adj_cost,1});
                            }else{
                                //get to original dest first
                                pool_dist = estimate_delta.dist + estimate_ori.dist;
                                pool_time = estimate_delta.time + estimate_ori.time;
                                pool_fare = fare_cal(pool_dist,pool_time, 1);
                                pool_cost = cost_cal(estimate_delta.dist, estimate_delta.time, 1);
                                assortment.push_back({Mode::pool,*it,pool_dist,pool_time,
                                    new_estimate_pre.dist,new_estimate_pre.time,
                                    pool_fare,pool_cost + pool_on_cost + pool_adj_cost,0,pool_adj_cost,0});
                            }
                        }
                    }
                }else{
                    //two previous pax, more complex analysis
                    int last_des_1 = des_list[int(des_list.size()) - 2].id;
                    int last_des_2 = des_list[int(des_list.size()) - 1].id;
                    //get rid of simple bad case to reduce computation
                    int ori_x,des_x,last_des_1_x,last_des_2_x,ori_y,des_y,last_des_1_y,last_des_2_y;
                    if (ori / grid_size > last_des_2 / grid_size){
                        ori_x = n - 1 - ori / grid_size;
                        des_x = n - 1 - des / grid_size;
                        last_des_1_x = n - 1 - last_des_1 / grid_size;
                        last_des_2_x = n - 1 - last_des_2 / grid_size;

                    }else{
                        ori_x = ori / grid_size;
                        des_x = des / grid_size;
                        last_des_1_x = last_des_1 / grid_size;
                        last_des_2_x = last_des_2 / grid_size;
                    }
                    
                    if (ori % grid_size > last_des_2 % grid_size){
                        ori_y = n - 1 - ori % grid_size;
                        des_y = n - 1 - des % grid_size;
                        last_des_1_y = n - 1 - last_des_1 % grid_size;
                        last_des_2_y = n - 1 - last_des_2 % grid_size;
                    }else{
                        ori_y = ori % grid_size;
                        des_y = des % grid_size;
                        last_des_1_y = last_des_1 % grid_size;
                        last_des_2_y = last_des_2 % grid_size;
                    }
                    
                                    
                    int region = -1;
                    bool in_region_flag = false;
                    if ((des_x >= (std::min(ori_x, last_des_1_x) - MAX_REROUTE_DIST_ALLOWED)) &&
                        (des_x <= (std::max(ori_x, last_des_1_x) + MAX_REROUTE_DIST_ALLOWED)) &&
                        (des_y >= (std::min(ori_y, last_des_1_y) - MAX_REROUTE_DIST_ALLOWED)) &&
                        (des_y <= (std::max(ori_y, last_des_1_y) + MAX_REROUTE_DIST_ALLOWED))){
                        region = 0;
                        in_region_flag = true;
                    }else{
                        if ((des_x >= (std::min(last_des_1_x, last_des_2_x) - MAX_REROUTE_DIST_ALLOWED)) and
                            (des_x <= (std::max(last_des_1_x, last_des_2_x) + MAX_REROUTE_DIST_ALLOWED)) and
                            (des_y >= (std::min(last_des_1_y, last_des_2_y) - MAX_REROUTE_DIST_ALLOWED)) and
                            (des_y <= (std::max(last_des_1_y, last_des_2_y) + MAX_REROUTE_DIST_ALLOWED))){
                            region = 1;
                            in_region_flag = true;
                        }else{
                            if (((des_x >= (last_des_2_x - MAX_REROUTE_DIST_ALLOWED)) and
                                 (des_y >= (last_des_2_y - MAX_REROUTE_DIST_ALLOWED)))){
                                region = 2;
                                in_region_flag = true;
                            }
                        }
                    }
                    
                    if (in_region_flag){
                        //in the correct region
                        if (region == 0){
                            //consider ori and des_1
                            RoutingOutput estimate_ori = routing_module->estimate(ori, last_des_1);
                            RoutingOutput estimate_delta = routing_module->estimate(des, last_des_1);
                            if (std::abs(new_estimate.time + estimate_delta.time - estimate_ori.time) < MAX_REROUTE_TIME_ALLOWED){
                                //no much rerouting; keep considering
                                if (estimate_ori.time > new_estimate.time){
                                    //get to new dest first
                                    pool_dist = new_estimate.dist;
                                    pool_time = new_estimate.time;
                                    pool_fare = fare_cal(pool_dist, pool_time, 1);
                                    pool_cost = cost_cal(std::abs(new_estimate.dist + estimate_delta.dist - estimate_ori.dist), std::abs(new_estimate.time + estimate_delta.time - estimate_ori.time), 1);
                                    assortment.push_back({Mode::pool,*it,pool_dist,pool_time,
                                        new_estimate_pre.dist,new_estimate_pre.time,
                                        pool_fare,pool_cost + pool_on_cost + pool_adj_cost,0,pool_adj_cost,2});
                                }else{
                                    //get to original dest first
                                    pool_dist = estimate_ori.dist + estimate_delta.dist;
                                    pool_time = estimate_ori.time + estimate_delta.time;
                                    pool_fare = fare_cal(pool_dist,pool_time, 1);
                                    pool_cost = cost_cal(estimate_delta.dist, estimate_delta.time, 1);
                                    assortment.push_back({Mode::pool,*it,pool_dist,pool_time,
                                        new_estimate_pre.dist,new_estimate_pre.time,
                                        pool_fare,pool_cost + pool_on_cost + pool_adj_cost,0,pool_adj_cost,1});
                                }
                            }
                        }
                                
                        if (region == 1){
                            //the most complex case
                            RoutingOutput estimate_ori_1 = routing_module->estimate(ori, last_des_1);
                            RoutingOutput estimate_ori_2 = routing_module->estimate(last_des_1, last_des_2);
                            RoutingOutput estimate_delta_1 = routing_module->estimate(last_des_1, des);
                            RoutingOutput estimate_delta_2 = routing_module->estimate(des, last_des_2);
                            RoutingOutput estimate_extra = routing_module->estimate(ori, last_des_2);
                            if ((std::abs(estimate_ori_1.time + estimate_delta_1.time + estimate_delta_2.time - estimate_extra.time) < MAX_REROUTE_TIME_ALLOWED) and (std::abs(estimate_ori_1.time + estimate_delta_1.time - new_estimate.time) < MAX_REROUTE_TIME_ALLOWED)){
                                //no much rerouting; keep considering
                                if (estimate_ori_2.time > estimate_delta_1.time){
                                    //get to new dest first
                                    pool_dist = estimate_ori_1.dist + estimate_delta_1.dist;
                                    pool_time = estimate_ori_1.time + estimate_delta_1.time;
                                    pool_fare = fare_cal(pool_dist,pool_time, 1);
                                    pool_cost = cost_cal(std::abs(estimate_delta_1.dist + estimate_delta_2.dist - estimate_ori_2.dist), std::abs(estimate_delta_1.time + estimate_delta_2.time - estimate_ori_2.time), 1);
                                    assortment.push_back({Mode::pool,*it,pool_dist,pool_time,
                                        new_estimate_pre.dist,new_estimate_pre.time,
                                        pool_fare,pool_cost + pool_on_cost + pool_adj_cost,0,pool_adj_cost,1});
                                }else{
                                    //get to original dest first
                                    pool_dist = estimate_ori_1.dist + estimate_ori_2.dist + estimate_delta_2.dist;
                                    pool_time = estimate_ori_1.time + estimate_ori_2.time + estimate_delta_2.time;
                                    pool_fare = fare_cal(pool_dist,pool_time, 1);
                                    pool_cost = cost_cal(estimate_delta_2.dist, estimate_delta_2.time, 1);
                                    assortment.push_back({Mode::pool,*it,pool_dist,pool_time,
                                        new_estimate_pre.dist,new_estimate_pre.time,
                                        pool_fare,pool_cost + pool_on_cost + pool_adj_cost,0,pool_adj_cost,0});
                                }
                            }
                        }
                                
                        if (region == 2){
                            //consider des_1, des_2
                            RoutingOutput estimate_ori_1 = routing_module->estimate(ori, last_des_1);
                            RoutingOutput estimate_ori_2 = routing_module->estimate(last_des_1, last_des_2);
                            RoutingOutput estimate_delta = routing_module->estimate(last_des_2, des);
                            int pool_dist_ori = estimate_ori_1.dist + estimate_ori_2.dist;
                            float pool_time_ori = estimate_ori_1.time + estimate_ori_2.time;
                            if (std::abs(new_estimate.time - pool_time_ori - estimate_delta.time) < MAX_REROUTE_TIME_ALLOWED){
                                //no much rerouting; keep considering
                                //get to original dest first
                                pool_dist = pool_dist_ori + estimate_delta.dist;
                                pool_time = pool_time_ori + estimate_delta.time;
                                pool_fare = fare_cal(pool_dist,pool_time, 1);
                                pool_cost = cost_cal(estimate_delta.dist, estimate_delta.time, 1);
                                assortment.push_back({Mode::pool,*it,pool_dist,pool_time,
                                    new_estimate_pre.dist,new_estimate_pre.time,
                                    pool_fare,pool_cost + pool_on_cost + pool_adj_cost,0,pool_adj_cost,0});
                            }
                        }
                        
                    }
                }
            }
        }
    }
    
    return assortment;
}

void Agent::update(int ori,int des,Option choice, int curr_time, int delay_time){
    Vehicle *this_veh = fleet[choice.veh_id];
    std::vector<Des> new_des;
    if (choice.type == Mode::taxi){
        new_des = {{ori, 0},
            {des, 1}};
    }else{
        new_des = this_veh->get_dest();
        if(!new_des.empty()){
            //direct pickup
            new_des.insert(new_des.begin(), {ori, 2});
            if (choice.insert == 0){
                //this des is the last one
                new_des.push_back({des, 3});
            }else{
                //original des is the last one
                new_des.insert(new_des.begin() + int(new_des.size()) - choice.insert, {des, 3});
            }
        }else{
            new_des = {{ori, 2},{des, 3}};
        }
    }
    
    int pre_status = this_veh->get_status();
    if (choice.type == Mode::taxi){
        this_veh->update(1, new_des, curr_time, choice.fare + choice.adj_fare,choice.adj_cost);
    }else{
        this_veh->update(2, new_des, curr_time, choice.fare + choice.adj_fare,choice.adj_cost);
    }
    
    //update availability map
    int curr_loc = this_veh->get_loc();
    if (pre_status == 0){
        //from empty to private or pool
        env->set_emp_veh_location_by_row(choice.veh_id, -1);
        if (choice.type == Mode::pool){
            //pool; update avi location
            env->set_avi_veh_location_by_row(choice.veh_id, curr_loc);
        }else{
            env->set_avi_veh_location_by_row(choice.veh_id, -1);
        }
    }else{
        //from pool to pool
        if (this_veh->get_pool_remain() <= 0){
            //no pool; remove status
            env->set_avi_veh_location_by_row(choice.veh_id, -1);
        }
    }
    
    //update training data
    (controller->*(controller->update_training_data))(choice);
    
    //update record
    pax_stat.push_back({curr_time-delay_time,curr_time,delay_time,choice.type,choice.fare + choice.adj_fare,float(choice.dist),choice.time,float(choice.pickup_dist),choice.pickup_time});
}

std::vector<VehicleState> Agent::get_veh_state(){
    std::vector<VehicleState> veh_state;
    veh_state.reserve(fleet_size);
    for (int i = 0; i < fleet_size; i++){
        veh_state.push_back(fleet[i]->save_state());
    }
    return veh_state;
}

std::vector<int> Agent::get_counter(){
    return freq_counter;
}

std::vector<pax_record> Agent::get_pax_record(){
    return pax_stat;
}

std::vector<agent_record> Agent::get_agent_record(){
    return agent_stat;
}

std::vector<sys_record> Agent::get_sys_record(){
    return env->get_sys_record();
}

std::vector<Vehicle *> Agent::get_fleet(){
    return fleet;
}
