//
//  environment.cpp
//  simulator_c
//
//  Created by Han Qiu on 12/11/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#include "environment.hpp"

Routing *Env::routing_module = nullptr;
int Env::grid_size = 0;
float Env::dynamic_travel_time_rate = 0.0;
int Env::dynamic_travel_time_flag = 0;
float Env::density_normalize_factor = 30.0;

float Env::d1 = 1.0;
float Env::d2 = 6.0;
float Env::density_factor = 15.0;

float Env::Greenshield_density_to_time(float density){
    float travel_time;
    if (density < d1){
        travel_time = 1.0;
    }
    else{
        if (density > d2 * 0.9){
            travel_time = 30.0;
        }
        else{
            travel_time = (d2 - d1) / (d2 - density);
            if (travel_time > 30.0){
                travel_time = 30.0;
            }
        }
    }
    return travel_time;
}

float Env::Greenshield_time_to_density(float time){
    float density;
    density = d2 - (d2 - d1) / time;
    return density;
}

void Env::setting(int new_grid_size, float new_dyanmic_travel_time_rate,
                  int new_dynamic_travel_time_flag, Routing &new_routing_module){
    grid_size = new_grid_size;
    dynamic_travel_time_flag = new_dynamic_travel_time_flag;
    dynamic_travel_time_rate = new_dyanmic_travel_time_rate;
    routing_module = &new_routing_module;
}

void Env::clear(){
    routing_module = nullptr;
}

int Env::get_grid_size(){
    return grid_size;
}

int Env::get_fleet_size(){
    return fleet_size;
}

std::vector<Vehicle *> Env::get_fleet(){
    return fleet;
}

std::vector< std::pair<int,int> > Env::get_curr_demand(){
    return (*demand)[relative_curr_time];
}

int Env::get_curr_time(){
    return curr_time;
}

std::vector<float> Env::get_travel_time(){
    return travel_time;
}

std::vector<sys_record> Env::get_sys_record(){
    return sys_stat;
}

std::vector<int> Env::get_emp_veh_location(){
    return emp_veh_location;
}

std::vector<int> Env::get_avi_veh_location(){
    return avi_veh_location;
}

void Env::set_emp_veh_location_by_row(int id,int new_value){
    if (new_value >= 0) emp_veh_location[id] = new_value;
    else emp_veh_location[id] = n_min;
}

void Env::set_avi_veh_location_by_row(int id,int new_value){
    if (new_value >= 0) avi_veh_location[id] = new_value;
    else avi_veh_location[id] = n_min;
}

std::vector<int> Env::get_veh_pool_capacity(){
    return veh_pool_capacity;
}

Env::Env(Env_Setting new_env_setting){
    n_min = - grid_size * grid_size;
    fleet_size = new_env_setting.fleet_size;
    curr_time = new_env_setting.start_time;
    relative_curr_time = 0;
    demand = new_env_setting.demand;
    travel_time_total = new_env_setting.travel_time_total;
    travel_time = (*travel_time_total)[curr_time];
    total_time = int(travel_time_total->size());
    fleet.reserve(fleet_size);
    emp_veh_location.assign(fleet_size,n_min);
    avi_veh_location.assign(fleet_size,n_min);
    veh_pool_capacity.assign(fleet_size,0);
    density.assign(grid_size * grid_size,0.0);
    sys_stat.reserve(travel_time_total->size());
    usual_traffic_record.reserve(travel_time_total->size());
    for (int i = 0; i < travel_time_total->size(); i++){
        std::vector<float> temp(grid_size * grid_size,0.0);
        usual_traffic_record.push_back(temp);
    }
    
    // fleet initiation
    int loc;
    int loc_temp;
    for (int i = 0; i < fleet_size; i++){
        fleet.push_back(new Vehicle(i,new_env_setting.vehicle_state[i]));
        // update location info matrix
        loc = new_env_setting.vehicle_state[i].loc;
        switch (new_env_setting.vehicle_state[i].status) {
            case 0:
                // empty
                emp_veh_location[i] = loc;
                break;
            case 1:
                // private
                break;
            case 2:
                // pool
                veh_pool_capacity[i] = fleet[i]->get_pool_remain();
                if (veh_pool_capacity[i] > 0) avi_veh_location[i] = loc;
                break;
            default:
                break;
        }
        // update density matrix
        if (new_env_setting.vehicle_state[i].status > 0){
            for (int n_col = -1; n_col <= 1; n_col++){
                for (int n_row = -1; n_row <= 1; n_row++){
                    loc_temp = loc + grid_size * n_col + n_row;
                    if ((loc_temp >= 0) and (loc_temp < grid_size * grid_size)){
                        density[loc_temp] += 1.0 / 9.0;
                    }
                }
            }
        }
    }
}

Env::~Env(){
    for (int i = 0; i < fleet_size; i++){
        delete fleet[i];
    }
    demand = nullptr;
    travel_time_total = nullptr;
}

void Env::update_usual_traffic(int ori, int des){
    if (dynamic_travel_time_flag > 0){
        RoutingOutput estimate = routing_module->accurate(ori, des);
        std::vector<int> est_route = estimate.route;
        int accu_time = curr_time;
        int temp_time,temp_loc,temp_loc_near;
        for (int i = 0; i < est_route.size(); i++){
            temp_loc = est_route[i];
            temp_time = int(roundf((*travel_time_total)[curr_time][temp_loc]));
            for (int j = 0; j < temp_time; j++){
                if (accu_time+j < total_time){
                    for (int n_col = -1; n_col <= 1; n_col++){
                        for (int n_row = -1; n_row <= 1; n_row++){
                            temp_loc_near = temp_loc + grid_size * n_col + n_row;
                            if ((temp_loc_near >= 0) and (temp_loc_near < grid_size * grid_size)){
                                usual_traffic_record[accu_time+j][temp_loc_near] += 1.0 / 9.0;
                            }
                        }
                    }
                }
            }
            accu_time += temp_time;
        }
    }
}

void Env::next_step(){
    std::vector<int> new_state;
    int loc_temp;
    density.assign(density.size(),0.0);
    for (int i = 0; i < fleet_size; i++){
        // do the move
        new_state = fleet[i]->move(curr_time,travel_time);
        
        // update informations
        if (new_state[1] > 0) emp_veh_location[i] = new_state[0];
        else emp_veh_location[i] = n_min;
        if (new_state[2] > 0) avi_veh_location[i] = new_state[0];
        else avi_veh_location[i] = n_min;
        if (new_state[3] > 0) veh_pool_capacity[i] = new_state[3];
        else veh_pool_capacity[i] = 0;
        
        // update density matrix
        if (new_state[1] < 1){
            for (int n_col = -1; n_col <= 1; n_col++){
                for (int n_row = -1; n_row <= 1; n_row++){
                    loc_temp = new_state[0] + grid_size * n_col + n_row;
                    if ((loc_temp >= 0) and (loc_temp < grid_size * grid_size)){
                        density[loc_temp] += 1.0 / 9.0;
                    }
                }
            }
        }
    }
    
    float last_sys_travel_time = 0.0;
    float last_sys_density = 0.0;
    float temp_ave_travel_time = 0.0;
    float temp_density;
    if (curr_time > 0){
        last_sys_travel_time = sys_stat.back().total_travel_time;
        last_sys_density = sys_stat.back().total_density;
    }
    
    travel_time = (*travel_time_total)[curr_time];
    if (dynamic_travel_time_flag == 1){
        float temp_d_1 = 0;
        float temp_d_2 = 0;
        for (int i = 0; i < grid_size * grid_size; i++){
            temp_density = (1 - dynamic_travel_time_rate) * Greenshield_time_to_density(travel_time[i]) + dynamic_travel_time_rate * density_normalize_factor * (density[i] + usual_traffic_record[curr_time][i]) / density_factor;
            temp_d_1 += density[i];
            temp_d_2 += usual_traffic_record[curr_time][i];
            travel_time[i] = Greenshield_density_to_time(temp_density);
            last_sys_travel_time += travel_time[i] * temp_density;
            last_sys_density += temp_density;
        }
    }else{
        for (int i = 0; i < grid_size * grid_size; i++){
            temp_density = Greenshield_time_to_density(travel_time[i]);
            last_sys_travel_time += travel_time[i] * temp_density;
            last_sys_density += temp_density;
        }
    }
    if (last_sys_density > 0){
        temp_ave_travel_time = last_sys_travel_time / last_sys_density;
    }
    sys_stat.push_back({last_sys_travel_time,last_sys_density,temp_ave_travel_time});
    
    routing_module->update_travel_time(travel_time);
    curr_time += 1;
    relative_curr_time += 1;
}
