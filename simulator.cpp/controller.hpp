//
//  controller.hpp
//  simulator_c
//
//  Created by Han Qiu on 12/12/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#ifndef controller_hpp
#define controller_hpp

#include "environment.hpp"

enum class Algorithm {
    none, full, random, single, pool, assort, pricing, assort_adjust, pricing_adjust, assort_adjust_rand, assort_adjust_post_rand,
    pricing_adjust_rand, pricing_adjust_post_rand, assort_model_free, pricing_model_free
};

struct Params{
    std::vector<float> theta;
    std::vector<float> sigma;
    std::vector<float> mu;
    std::vector<float> kappa;
    float alpha;
};

struct TrainingData{
    int T;
    float R;
    Params grad;
};

class InnerModel{
    static float ASC_private,ASC_pool,ASC_ori;
    static float b_dist_private,b_dist_pool;
    static float b_time_private,b_time_pool,b_time_ori;
    static float b_fare_ext_private,b_fare_ext_pool,b_fare_ori;
public:
    InnerModel();
    float get_U(Option assort);
    float get_R(Option assort);
    float get_U0();
    float get_k(Mode assort_type);
};


class Controller{
    static float MAX_PRICE_SURGE;
    static float MAX_BIAS;
    static InnerModel *in_model;
    static std::vector< std::vector<float> > *ori_dist;
    static std::vector< std::vector<float> > *des_dist;
    int curr_time;
    std::vector<float> t;
    std::vector<float> d_in,d_out,a_pri,a_pool;
    Params params;
    std::vector<TrainingData> training_data;
    static Algorithm algorithm;
    
    // random generator
    std::mt19937 mt;
    std::normal_distribution<float> nrand;
    std::uniform_real_distribution<float> urand;
    
public:
    void (Controller::*update_training_data)(Option);
    static std::vector<Option> (Controller::*optimize_train)(std::vector<Option>, int, int);
    static std::vector<Option> (Controller::*optimize_test)(std::vector<Option>, int, int);
    std::vector<Option> (Controller::*optimize)(std::vector<Option>, int, int);
    
    static void setting(std::vector< std::vector<float> > &new_ori_dist,std::vector< std::vector<float> > &new_des_dist,Algorithm new_algorithm);
    static void clear();
    Controller(int status); // 0 is training, 1 is test
    
    void reset(int new_curr_time, Params new_params);
    void update_curr_time();
    
    // function to calculate states
    void update_global_state(Env &env);
    std::vector<float> get_local_state(int ori,int des);
    std::vector<float> get_assort_state(Option assort);
    
    // auxillary functions
    // maybe remove them to reduce function call cost?
    float get_assort_opt_z(int size, std::vector<float> V, std::vector<float> R, float V0);
    float get_pricing_opt_z(int size, std::vector<float> V, std::vector<float> R, std::vector<float> k, float V0);
    std::vector<float> get_assort_revenue(std::vector<Option> assortment, int size);
    std::vector<float> get_assort_revenue_adjust(std::vector<Option> assortment, int size, std::vector<float> control_value);
    std::vector<float> get_assort_preference(std::vector<Option> assortment, int size);
    std::vector<float> get_assort_elasticity(std::vector<Option> assortment, int size);
    
    // update training data
    void update_training_data_train(Option choice);
    void update_training_data_test(Option choice);
    
    // control algorithms
    std::vector<Option> none(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> full(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> single(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> pool(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> random(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> assort(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> pricing(std::vector<Option> assortment, int ori, int des);
    
    std::vector<Option> assort_adjust(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> assort_model_free(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> pricing_adjust(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> pricing_model_free(std::vector<Option> assortment, int ori, int des);
    
    std::vector<Option> assort_adjust_rand(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> assort_adjust_post_rand(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> assort_model_free_rand(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> pricing_adjust_rand(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> pricing_adjust_post_rand(std::vector<Option> assortment, int ori, int des);
    std::vector<Option> pricing_model_free_rand(std::vector<Option> assortment, int ori, int des);

    
};

#endif /* controller_hpp */
