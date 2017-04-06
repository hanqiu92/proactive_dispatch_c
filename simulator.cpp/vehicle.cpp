//
//  vehicle.cpp
//  simulator_c
//
//  Created by Han Qiu on 12/8/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#include "vehicle.hpp"

int Vehicle::max_capacity = MAX_CAPACITY;
float Vehicle::u_d_cost = U_D_COST;
float Vehicle::u_t_cost = U_T_COST;
float Vehicle::base_cost = BASE_COST;
float Vehicle::pri_u_d_fare = U_D_FARE;
float Vehicle::pri_u_t_fare = U_T_FARE;
float Vehicle::pri_base_fare = BASE_FARE;
float Vehicle::p_rate = 1.0;
float Vehicle::pool_u_d_fare = U_D_FARE;
float Vehicle::pool_u_t_fare = U_T_FARE;
float Vehicle::pool_base_fare = BASE_FARE;

Routing *Vehicle::routing_module = nullptr;
int Vehicle::grid_size = 0;

void Vehicle::setting(float new_p_rate,Routing &new_routing_module){
    p_rate = new_p_rate;
    pool_u_d_fare = p_rate * U_D_FARE;
    pool_u_t_fare = p_rate * U_T_FARE;
    pool_base_fare = p_rate * BASE_FARE;
    routing_module = &new_routing_module;
    grid_size = new_routing_module.get_grid_size();
}

void Vehicle::set_grid_size(int new_grid_size){
    grid_size = new_grid_size;
}

Vehicle::Vehicle(int new_id,VehicleState new_vehicle_state){
    // current status
    id = new_id;
    loc = new_vehicle_state.loc;
    status = new_vehicle_state.status;
    curr_pax = new_vehicle_state.curr_pax;
    des_cell = new_vehicle_state.des_cell;
    pool_remain = max_capacity - (int(des_cell.size()) + curr_pax) / 2;
    
    // current statistics
    cell_time = 0.0;
    travel_dist = 0;
    if (status == 0){
        start_time = -1;
        ETA = -1;
        route = {};
        route_index = -1;
    }else{
        RoutingOutput new_route = routing(des_cell[0].id);
        start_time = 0;
        ETA = int(new_route.time);
        route = new_route.route;
        route_index = 0;
    }
    
    // operating statistics
    total_revenue = 0.0;
    total_cost = 0.0;
    total_tax = 0.0;
    total_pax = 0;
    pool_pax = 0;
    private_pax = 0;
    total_time = 0;
    total_dist = 0;
}

RoutingOutput Vehicle::routing(int des){
    return routing_module->accurate(loc, des);
}

void Vehicle::update(int new_status,std::vector<Des> new_des_cell,int curr_time,float fare, float tax){
    //status, des, and route should be provided by agent
    if (status == 0){
        start_time = curr_time;
    }
    status = new_status;
    des_cell = new_des_cell;
    total_revenue += fare;
    total_tax += tax;
    
    if (new_status == 2){
        pool_remain = max_capacity - (int(new_des_cell.size()) + curr_pax) / 2;
    }else{
        pool_remain = 0;
    }
    RoutingOutput new_route = routing(new_des_cell[0].id);
    cell_time = 0.0;
    travel_dist = 0;
    ETA = curr_time + int(new_route.time);
    route = new_route.route;
    route_index = 0;
}

void Vehicle::update_route(int curr_time, std::vector<float> &cell_travel_time){
    // execute at the beginning to check whether need to change to another route
    // sum the time and check if it is too large
    float est_time = 0.0;
    int route_size = int(route.size());
    for (int i = route_index; i < route_size; i++){
        est_time += cell_travel_time[route[i]];
    }
    //if there is great delay in estimation
    if (ETA < int(est_time) + curr_time - 10){
        //record current progress
        travel_dist += route_index;
        //generate new route
        RoutingOutput new_route = routing(des_cell[0].id);
        ETA = curr_time + int(new_route.time);
        route_index = 0;
    }
}

std::vector<int> Vehicle::move(int curr_time, std::vector<float> &cell_travel_time){
    if (status != 0){
        //check if need to reroute per 10 mins
        if (curr_time % 10 == 0){
            update_route(curr_time, cell_travel_time);
        }
        //follow the route
        int curr_cell = route[route_index];
        //check travel time
        cell_time += 1.0;
        total_cost += u_t_cost;
        total_time += 1;
        
        float curr_cell_travel_time = cell_travel_time[curr_cell];
        if (cell_time >= curr_cell_travel_time){
            cell_time -= curr_cell_travel_time;
            //check if it is move within cell
            if (int(route.size()) > 1){
                route_index += 1;
                loc = route[route_index];
                total_dist += 1;
                total_cost += u_d_cost;
            }
        }
                
        //if arrive, remove the destination and update status
        if (loc == des_cell[0].id){
            
            /*
            //take out parameter for re-use
            int curr_travel_time = curr_time - start_time;
            //summarize operation statistics
            total_cost += u_t_cost * curr_travel_time;
            total_time += curr_travel_time;
            */
            
            Des finished = des_cell[0];
            des_cell.erase(des_cell.begin());
            if (!des_cell.empty()){
                Des next_des = des_cell[0];
                //update status
                switch (finished.type) {
                    case 0:
                        curr_pax = 1;
                        total_pax += 1;
                        private_pax += 1;
                        //total_revenue += pri_base_fare;
                        total_cost += base_cost;
                        break;
                    case 1:
                        curr_pax = 0;
                        if (next_des.type == 2) status = 2;
                        break;
                    case 2:
                        curr_pax += 1;
                        total_pax += 1;
                        pool_pax += 1;
                        //total_revenue += pool_base_fare;
                        total_cost += base_cost;
                        break;
                    case 3:
                        curr_pax -= 1;
                        //if (curr_pax < 0) curr_pax = 0;
                        if ((curr_pax == 0) and (next_des.type == 0)) status = 1;
                        break;
                    default:
                        break;
                }
                //generate new route
                RoutingOutput new_route = routing(next_des.id);
                cell_time = 0.0;
                travel_dist = 0;
                start_time = curr_time;
                ETA = curr_time + int(new_route.time);
                route = new_route.route;
                route_index = 0;
                if (status == 2){
                    pool_remain = max_capacity - (int(des_cell.size()) + curr_pax) / 2;
                }else{
                    pool_remain = 0;
                }
            }else{
                //update status
                status = 0;
                curr_pax = 0;
                pool_remain = max_capacity;
                cell_time = 0.0;
                travel_dist = 0;
                start_time = -1;
                ETA = -1;
                route = {};
                route_index = -1;
            }
        }
    }

    switch (status) {
        case 0:
            return {loc,1,0,0};
            break;
        case 1:
            return {loc,0,0,0};
            break;
        case 2:
            if (pool_remain > 0) return {loc,0,1,pool_remain};
            else return {loc,0,0,0};
            break;
        default:
            return {-1,-1,-1,-1};
            break;
    }
}

int Vehicle::get_loc(){
    return loc;
}

int Vehicle::get_status(){
    return status;
}

int Vehicle::get_curr_pax(){
    return curr_pax;
}

int Vehicle::get_pool_remain(){
    return pool_remain;
}

std::vector<Des> Vehicle::get_dest(){
    return des_cell;
}

int Vehicle::get_total_pax(){
    return total_pax;
}

int Vehicle::get_private_pax(){
    return private_pax;
}

int Vehicle::get_pool_pax(){
    return pool_pax;
}

float Vehicle::get_total_revenue(){
    return total_revenue;
}

float Vehicle::get_total_cost(){
    return total_cost;
}

float Vehicle::get_total_tax(){
    return total_tax;
}

int Vehicle::get_total_time(){
    return total_time;
}

int Vehicle::get_total_dist(){
    return total_dist;
}

VehicleState Vehicle::save_state(){
    //save vehicle operation state for future initiation
    return {loc,status,curr_pax,des_cell};
}

