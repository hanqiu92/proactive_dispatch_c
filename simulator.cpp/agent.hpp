//
//  agent.hpp
//  simulator_c
//
//  Created by Han Qiu on 12/12/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#ifndef agent_hpp
#define agent_hpp

#include "model.hpp"
#include "controller.hpp"

struct pax_record{
    int request_time;
    int curr_time;
    int delay_time;
    Mode service_type;
    float fare;
    float dist;
    float time;
    float pickup_dist;
    float pickup_time;
};

struct assort_type_record{
    int both;
    int pri;
    int pool;
};

struct agent_record{
    float revenue;
    float cost;
    float tax;
    float profit;
    int total_travel_time;
    int total_travel_dist;
    int new_demand;
    int fulfill_demand;
    int total_pax;
    int private_pax;
    int pool_pax;
    assort_type_record assort_type;
};

struct Agent_Setting{
    int status;
    bool detail_output_flag;
    Params params;
};

class Agent{    
    static int grid_size;
    static float p_rate;
    static float tax_congest;
    static float bound_congest;
    static float tax_demand;
    static float bound_demand;
    static Routing *routing_module;
    static Controller *train_controller;
    static Controller *test_controller;
    static std::vector< std::vector<float> > *ori_dist;
    static std::vector< std::vector<float> > *des_dist;
    static float demand_norm_factor;
    
    Controller *controller;
    Env *env;
    int fleet_size;
    std::vector<Vehicle *> fleet;
    int curr_time;
    bool detail_output_flag;
    std::vector<pax_record> pax_stat;
    std::vector<agent_record> agent_stat;
    std::vector<int> freq_counter;
    std::vector< std::array<int,3> > history_demand;
    assort_type_record assort_type_stat;
    
public:
    float fare_cal(int dist, float time, int typ);
    float cost_cal(int dist, float time, int typ);
    float adj_cost_cal(int ori, int des, int dist, float time, int typ);
    static void setting(int new_grid_size,float new_p_rate, float new_tax_congest, float new_tax_demand, std::vector< std::vector<float> > &new_ori_dist,std::vector< std::vector<float> > &new_des_dist, float demand_factor,Routing &new_routing_module,Controller &new_train_controller, Controller&new_test_controller);
    static void clear();
    Agent(Env_Setting env_setting,Agent_Setting agent_setting);
    ~Agent();
    void next_step();
    int process_demand(int ori,int des,int curr_time,int delay_time);
    std::vector<Option> gene_assort(int ori,int des);
    void update(int ori,int des,Option choice, int curr_time, int delay_time);
    std::vector<VehicleState> get_veh_state();
    std::vector<int> get_counter();
    std::vector<pax_record> get_pax_record();
    std::vector<agent_record> get_agent_record();
    std::vector<sys_record> get_sys_record();
    std::vector<Vehicle *> get_fleet();
};



#endif /* agent_hpp */
