//
//  environment.hpp
//  simulator_c
//
//  Created by Han Qiu on 12/11/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#ifndef environment_hpp
#define environment_hpp

#include "vehicle.hpp"

struct Env_Setting{
    int fleet_size;
    std::vector<VehicleState> vehicle_state;
    int start_time;
    std::vector< std::vector< std::pair<int,int> > > *demand;
    std::vector< std::vector<float> > *travel_time_total;
};

struct sys_record{
    float total_travel_time;
    float total_density;
    float average_travel_time;
};

class Env{
    static int grid_size;
    static Routing *routing_module;
    static float dynamic_travel_time_rate;
    static float density_normalize_factor;
    static int dynamic_travel_time_flag;
    static float d1,d2;
    static float density_factor;
    int fleet_size;
    int curr_time;
    int relative_curr_time;
    int n_min;
    std::vector< std::vector< std::pair<int,int> > > *demand;
    std::vector< std::vector<float> > *travel_time_total;
    std::vector<Vehicle *> fleet;
    std::vector<float> travel_time;
    std::vector<float> density;
    std::vector<int> emp_veh_location;
    std::vector<int> avi_veh_location;
    std::vector<int> veh_pool_capacity;
    std::vector<sys_record> sys_stat;
    
public:
    static void setting(int new_grid_size, float new_dynamic_travel_time_rate,
                        int new_dynamic_travel_time_flag, Routing &new_routing_module);
    static void clear();
    int get_grid_size();
    int get_fleet_size();
    std::vector<Vehicle *> get_fleet();
    std::vector< std::pair<int,int> > get_curr_demand();
    int get_curr_time();
    std::vector<float> get_travel_time();
    std::vector<sys_record> get_sys_record();
    std::vector<int> get_emp_veh_location();
    std::vector<int> get_avi_veh_location();
    void set_emp_veh_location_by_row(int id,int new_value);
    void set_avi_veh_location_by_row(int id,int new_value);
    std::vector<int> get_veh_pool_capacity();
    Env(Env_Setting new_env_setting);
    ~Env();
    void next_step();
    float Greenshield_density_to_time(float density);
    float Greenshield_time_to_density(float time);
    
};

#endif /* environment_hpp */
