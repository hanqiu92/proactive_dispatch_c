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

float Env::d1 = 1.0;
float Env::d2 = 6.0;
float Env::density_factor_fixed = 0.8;
float Env::density_factor = 1.0;

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
            travel_time = (d2 - d1) / (d2 - density) * density / d1;
            if (travel_time > 30.0){
                travel_time = 30.0;
            }
        }
    }
    return travel_time;
}

float Env::Greenshield_time_to_density(float time){
    float density;
    density = d1 * d2 * time / ( d1 * time + (d2 - d1));
    return density;
}

void Env::setting(int new_grid_size, float new_dyanmic_travel_time_rate,
                  int new_dynamic_travel_time_flag, float ext_density_factor, Routing &new_routing_module){
    grid_size = new_grid_size;
    dynamic_travel_time_flag = new_dynamic_travel_time_flag;
    dynamic_travel_time_rate = new_dyanmic_travel_time_rate;
    routing_module = &new_routing_module;
    density_factor = density_factor_fixed * ext_density_factor;
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
    usual_traffic_dist.reserve(fleet_size);
    usual_traffic_location.reserve(fleet_size);
    usual_traffic_route.reserve(fleet_size);
    usual_traffic_route_index.reserve(fleet_size);
    
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
        usual_traffic_route.push_back(estimate.route);
        usual_traffic_location.push_back(estimate.route[0]);
        usual_traffic_dist.push_back(0.5);
        usual_traffic_route_index.push_back(0);
    }
}

void Env::next_step(){
    std::vector<int> new_state;
    int loc_temp;
    density.assign(density.size(),0.0);
    // deal with operator's traffic
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
    
    if (dynamic_travel_time_flag > 0){
        // deal with usual traffic
        for (int i = 0; i < int(usual_traffic_location.size()); i++){
            // do the move
            if (usual_traffic_location[i] >= 0){
                float cell_dist = usual_traffic_dist[i] + 1.0 / travel_time[usual_traffic_location[i]];
                
                if (cell_dist >= 1.0){
                    cell_dist -= 1.0;
                    usual_traffic_route_index[i] += 1;
                    if (usual_traffic_route_index[i] < int(usual_traffic_route[i].size()) - 1){
                        usual_traffic_location[i] = usual_traffic_route[i][usual_traffic_route_index[i]];
                    }else{
                        usual_traffic_location[i] = -1;
                    }
                }
                usual_traffic_dist[i] = cell_dist;
            }
            
            // update density matrix
            if (usual_traffic_location[i] >= 0){
                for (int n_col = -1; n_col <= 1; n_col++){
                    for (int n_row = -1; n_row <= 1; n_row++){
                        loc_temp = usual_traffic_location[i] + grid_size * n_col + n_row;
                        if ((loc_temp >= 0) and (loc_temp < grid_size * grid_size)){
                            density[loc_temp] += 1.0 / 9.0;
                        }
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
        for (int i = 0; i < grid_size * grid_size; i++){
            temp_density = (1 - dynamic_travel_time_rate) * Greenshield_time_to_density(travel_time[i]) + dynamic_travel_time_rate * density[i] / density_factor;
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
