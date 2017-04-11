//
//  vehicle.hpp
//  simulator_c
//
//  Created by Han Qiu on 12/8/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#ifndef vehicle_hpp
#define vehicle_hpp

#include "routing.hpp"

struct Des{
    int id;
    int type; // 0 private on, 1 private off, 2 pool on, 3 pool off
};

struct VehicleState{
    int loc;
    int status; // 0 empty, 1 private, 2 pool
    int curr_pax;
    std::vector<Des> des_cell;
};

class Vehicle{
    static int max_capacity;
    static float u_d_cost;
    static float u_t_cost;
    static float base_cost;
    static float pri_u_d_fare;
    static float pri_u_t_fare;
    static float pri_base_fare;
    static float p_rate;
    static float pool_u_d_fare;
    static float pool_u_t_fare;
    static float pool_base_fare;
    
    static int grid_size;
    static Routing *routing_module;
    
    // current status
    int id;
    int loc;
    int status; // 0 for empty, 1 for private, 2 for pool
    int curr_pax;
    int pool_remain;
    std::vector<Des> des_cell;
    
    // current statistics
    float cell_time;
    int travel_dist;
    int start_time;
    int ETA;
    std::vector<int> route;
    int route_index;
    
    // operating statistics
    float total_revenue;
    float total_cost;
    float total_tax;
    int total_dist;
    int total_time;
    int total_pax;
    int private_pax;
    int pool_pax;
    
public:
    static void setting(float new_p_rate,Routing &new_routing_module);
    static void set_grid_size(int new_grid_size);
    Vehicle(int new_id,VehicleState new_vehicle_state);
    RoutingOutput routing(int des);
    void update(int new_status,std::vector<Des> new_des_cell,int curr_time,float fare,float tax);
    void update_route(int curr_time, std::vector<float> &cell_travel_time);
    std::vector<int> move(int curr_time, std::vector<float> &cell_travel_time);
    
    int get_loc();
    int get_status();
    int get_curr_pax();
    int get_pool_remain();
    std::vector<Des> get_dest();
    
    int get_total_pax();
    int get_private_pax();
    int get_pool_pax();
    float get_total_revenue();
    float get_total_cost();
    float get_total_tax();
    int get_total_time();
    int get_total_dist();
    
    VehicleState save_state();
};

#endif /* vehicle_hpp */
