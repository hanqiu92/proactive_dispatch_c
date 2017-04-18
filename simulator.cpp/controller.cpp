//
//  controller.cpp
//  simulator_c
//
//  Created by Han Qiu on 12/12/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#include "controller.hpp"

// ****************************************
// inner model part

float U0 = 0.0;
float a = 10.0;
float b = 0.5;
float c = 5.0;
float d = 0.5;

float InnerModel::ASC_private = a + 1.0;
float InnerModel::ASC_pool = a;
float InnerModel::b_dist_private = 0.0;
float InnerModel::b_dist_pool = b;
float InnerModel::b_time_private = b;
float InnerModel::b_time_pool = 0.5 * b;
float InnerModel::b_fare_private = c;
float InnerModel::b_fare_pool = 2 * c;

InnerModel::InnerModel(){

}

float InnerModel::get_U(Option assort){
    switch (assort.type) {
        case Mode::pool:
            return (ASC_pool - b_dist_pool * (assort.dist - 5) -
                    b_time_pool * (assort.time - assort.dist)) * d;
            break;
        case Mode::taxi:
            return (ASC_private - b_dist_private * (assort.dist - 5) -
                    b_time_private * (assort.time - assort.dist)) * d;
            break;
        default:
            return 0;
            break;
    }
}

float InnerModel::get_R(Option assort){
    return (assort.fare - assort.cost);
}

float InnerModel::get_U0(){
    return U0;
}

float InnerModel::get_k(Mode assort_type){
    if (assort_type == Mode::taxi) return b_fare_private;
    else return b_fare_pool;
}

// ****************************************
// controller part

float Controller::MAX_PRICE_SURGE = 10.0;
float Controller::MAX_BIAS = 10.0;
InnerModel *Controller::in_model = new InnerModel();
std::vector< std::vector<float> > *Controller::ori_dist = nullptr;
std::vector< std::vector<float> > *Controller::des_dist = nullptr;
void (Controller::*update_training_data)(Option) = &Controller::update_training_data_test;
Algorithm Controller::algorithm = Algorithm::full;

std::vector<Option> (Controller::*(Controller::optimize_train))(std::vector<Option>, int, int) = &Controller::full;
std::vector<Option> (Controller::*(Controller::optimize_test))(std::vector<Option>, int, int) = &Controller::full;

void Controller::setting(std::vector< std::vector<float> > &new_ori_dist, std::vector< std::vector<float> > &new_des_dist, Algorithm new_algorithm){
    if (in_model == NULL) in_model = new InnerModel();
    ori_dist = &new_ori_dist;
    des_dist = &new_des_dist;
    algorithm = new_algorithm;
}

void Controller::clear(){
    ori_dist = nullptr;
    des_dist = nullptr;
}

Controller::Controller(int status){
    curr_time = 0;
    params.theta.assign(12,0.0);
    params.sigma.assign(12,0.0);
    params.mu.assign(8,0.0);
    params.kappa.assign(8,0.0);

    // initialize random generator
    std::random_device rd;
    mt.seed(rd());
    nrand = std::normal_distribution<float>(0.0,1.0);
    urand = std::uniform_real_distribution<float>(0.0,1.0);

    if (!training_data.empty()){
        training_data.clear();
    }
    if (status == 0){
        switch (algorithm) {
            case Algorithm::none:
                optimize = &Controller::none;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::full:
                optimize = &Controller::full;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::single:
                optimize = &Controller::single;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::pool:
                optimize = &Controller::pool;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::random:
                optimize = &Controller::random;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::assort:
                optimize = &Controller::assort;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::assort_adjust:
                optimize = &Controller::assort_adjust;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::pricing:
                optimize = &Controller::pricing;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::pricing_adjust:
                optimize = &Controller::pricing_adjust;
                update_training_data = &Controller::update_training_data_test;
                break;
            case Algorithm::assort_adjust_rand:
                optimize = &Controller::assort_adjust_rand;
                update_training_data = &Controller::update_training_data_train;
                break;
            case Algorithm::assort_adjust_post_rand:
                optimize = &Controller::assort_adjust_post_rand;
                update_training_data = &Controller::update_training_data_train;
                break;
            case Algorithm::pricing_adjust_rand:
                optimize = &Controller::pricing_adjust_rand;
                update_training_data = &Controller::update_training_data_train;
                break;
            case Algorithm::pricing_adjust_post_rand:
                optimize = &Controller::pricing_adjust_post_rand;
                update_training_data = &Controller::update_training_data_train;
                break;
            case Algorithm::assort_model_free:
                optimize = &Controller::assort_model_free_rand;
                update_training_data = &Controller::update_training_data_train;
                break;
            case Algorithm::pricing_model_free:
                optimize = &Controller::pricing_model_free_rand;
                update_training_data = &Controller::update_training_data_train;
                break;
            default:
                optimize = &Controller::full;
                update_training_data = &Controller::update_training_data_test;
                break;
        }
    }else{
        update_training_data = &Controller::update_training_data_test;
        switch (algorithm) {
            case Algorithm::none:
                optimize = &Controller::none;
                break;
            case Algorithm::full:
                optimize = &Controller::full;
                break;
            case Algorithm::single:
                optimize = &Controller::single;
                break;
            case Algorithm::pool:
                optimize = &Controller::pool;
                break;
            case Algorithm::random:
                optimize = &Controller::random;
                break;
            case Algorithm::assort:
                optimize = &Controller::assort;
                break;
            case Algorithm::assort_adjust:
                optimize = &Controller::assort_adjust;
                break;
            case Algorithm::pricing:
                optimize = &Controller::pricing;
                break;
            case Algorithm::pricing_adjust:
                optimize = &Controller::pricing_adjust;
                break;
            case Algorithm::assort_adjust_rand:
                optimize = &Controller::assort_adjust;
                break;
            case Algorithm::assort_adjust_post_rand:
                optimize = &Controller::assort_adjust;
                break;
            case Algorithm::pricing_adjust_rand:
                optimize = &Controller::pricing_adjust;
                break;
            case Algorithm::pricing_adjust_post_rand:
                optimize = &Controller::pricing_adjust;
                break;
            case Algorithm::assort_model_free:
                optimize = &Controller::assort_model_free;
                break;
            case Algorithm::pricing_model_free:
                optimize = &Controller::pricing_model_free;
                break;
            default:
                optimize = &Controller::full;
                break;
        }
    }
}

void Controller::reset(int new_curr_time, Params new_params){
    if (!training_data.empty()){
        training_data.clear();
    }
    curr_time = new_curr_time;
    params = new_params;
}

void Controller::update_curr_time(){
    curr_time += 1;
}

void Controller::update_global_state(Env &env){
    int n = env.get_grid_size();
    int fleet_size = env.get_fleet_size();
    
    // time state
    t = env.get_travel_time();

    // demand state
    d_in = (*des_dist)[curr_time];
    d_out = (*ori_dist)[curr_time];
    float normalize_factor = n * n * 1440.0 / 7600.0;
    for (int i = 0; i < n * n; i++){
        d_in[i] = d_in[i] * normalize_factor;
        d_out[i] = d_out[i] * normalize_factor;
    }

    // supply state
    std::vector<int> emp_veh_location = env.get_emp_veh_location();
    std::vector<int> avi_veh_location = env.get_avi_veh_location();
    std::vector<int> veh_pool_capacity = env.get_veh_pool_capacity();
    std::vector<float> a_pri_temp(n * n, 0.0);
    std::vector<float> a_pool_temp(n * n, 0.0);
    a_pri.assign(n * n, 0.0);
    a_pool.assign(n * n, 0.0);

    for (int i = 0; i < fleet_size; i++){
        if (emp_veh_location[i] >= 0){
            a_pri_temp[emp_veh_location[i]] += 1.0;
            a_pool_temp[emp_veh_location[i]] += 3.0;
        }
        if (avi_veh_location[i] >= 0){
            a_pool_temp[avi_veh_location[i]] += float(veh_pool_capacity[i]);
        }
    }
    float supply_factor = 2.0;
    float a_pri_factor = 1.0 / fleet_size * n * n * supply_factor;
    float a_pool_factor = 1.0 / 3.0 / fleet_size * n * n * supply_factor;
    int temp_i;
    for (int i = 0; i < n * n; i++){
        for (int j = 0; j < 3 * 3; j++){
            temp_i = i - 1 - n + (j / 3) * n + (j % 3);
            if ((temp_i >= 0) && (temp_i < n * n)){
                a_pri[i] += a_pri_temp[temp_i];
                a_pool[i] += a_pool_temp[temp_i];
            }
        }
        a_pri[i] = a_pri[i] * a_pri_factor / (3 * 3);
        a_pool[i] = a_pool[i] * a_pool_factor / (3 * 3);
    }
}

std::vector<float> Controller::get_local_state(int ori, int des){
    return {1, d_out[ori], d_in[des], a_pri[ori], t[ori], t[des], 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, d_out[ori], d_in[des], a_pool[ori], t[ori], t[des]};
}
std::vector<float> Controller::get_assort_state(Option assort){
    if (assort.type == Mode::taxi) return {assort.fare, assort.cost, float(assort.dist), float(assort.time), 0, 0, 0, 0};
    else return {0, 0, 0, 0,assort.fare, assort.cost, float(assort.dist), float(assort.time)};
}

float Controller::get_assort_opt_z(int size, std::vector<float> V, std::vector<float> R, float V0){
    float z = 0.0;
    float delta_z = 1.0;
    float S_sum;
    float V_sum;
    while (delta_z > 1e-3){
        S_sum = 0.0;
        V_sum = V0;
        for (int i = 0; i < size; i++){
            if ((R[i] - z) >= 0.0){
                S_sum += V[i] * R[i];
                V_sum += V[i];
            }
        }
        delta_z = S_sum / V_sum - z;
        z += delta_z;
    }

    return z;
}

float Controller::get_pricing_opt_z(int size, std::vector<float> V, std::vector<float> R, std::vector<float> k, float V0){
    std::vector<float> dis_temp;
    dis_temp.assign(size,0.0);
    float S_sum,V_sum,z,delta_z,discount,max_dis;
    S_sum = 0.0;
    V_sum = V0;
    for (int i = 0; i < size; i++){
        S_sum += V[i] * R[i];
        V_sum += V[i];
    }

    z = S_sum / V_sum;
    delta_z = 1.0;
    while (std::abs(delta_z) > 1e-3){
        max_dis = 0.0;
        for (int i = 0; i < size; i++){
            dis_temp[i] = (R[i]-z) / k[i] - 1;
            if (dis_temp[i] > max_dis) max_dis = dis_temp[i];
        }
        S_sum = 0.0;
        V_sum = V0 / exp(max_dis);
        for (int i = 0; i < size; i++){
            discount = exp(dis_temp[i] - max_dis);
            S_sum += V[i] * (z + k[i]) * discount;
            V_sum += V[i] * discount;
        }
        delta_z = S_sum / V_sum - z;
        // code to check if there is numerical problem in computation
        if (isnan(delta_z)){
            std::cout<<S_sum<<","<<V_sum<<"\n";
            std::cout<<z<<"\n";
            for (int i = 0; i < size; i++){
                std::cout<<V[i]<<","<<R[i]<<","<<k[i]<<"\n";
                std::cout<<exp( (R[i]-z) / k[i] - 1)<<"\n";
            }
        }

        z += delta_z;
    }

    return z;
}


void Controller::update_training_data_test(Option choice){
}

void Controller::update_training_data_train(Option choice){
    training_data.back().R = choice.fare + choice.adj_fare - choice.cost - choice.adj_cost;
}

std::vector<float> Controller::get_assort_revenue(std::vector<Option> assortment, int size){
    std::vector<float> R(size,0.0);
    for (int i = 0; i < size; i++){
        R[i] = in_model->get_R(assortment[i+1]);
    }
    return R;
}

std::vector<float> Controller::get_assort_revenue_adjust(std::vector<Option> assortment, int size, std::vector<float> control_value){
    std::vector<float> R(size,0.0);
    for (int i = 0; i < size; i++){
        R[i] = in_model->get_R(assortment[i+1]);
        if (assortment[i+1].type == Mode::taxi) R[i] += control_value[0];
        else R[i] += control_value[1];
    }
    return R;
}

std::vector<float> Controller::get_assort_preference(std::vector<Option> assortment, int size){
    std::vector<float> U(size+1,0.0);
    std::vector<float> V(size+1,0.0);
    U[size] = in_model->get_U(assortment[0]);
    for (int i = 0; i < size; i++){
        U[i] = in_model->get_U(assortment[i+1]);
    }
    float max_U = *std::max_element(U.begin(), U.end());
    for (int i = 0; i < size+1; i++){
        V[i] = exp(U[i] - max_U);
    }
    V[size] += exp(- max_U);
    return V;
}

std::vector<float> Controller::get_assort_elasticity(std::vector<Option> assortment, int size){
    std::vector<float> k(size,1.0);
    for (int i = 0; i < size; i++){
        k[i] = 1.0 / in_model->get_k(assortment[i+1].type);
    }
    return k;
}

std::vector<Option> Controller::none(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size());
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size);
    for (int i = 0; i < size; i++){
        if (assortment[i].type == Mode::stay){
            opt_assortment.push_back(assortment[i]);
        }
    }
    return opt_assortment;
}

std::vector<Option> Controller::full(std::vector<Option> assortment, int ori, int des){
    return assortment;
}

std::vector<Option> Controller::single(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size());
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size);
    for (int i = 0; i < size; i++){
        if ((assortment[i].type == Mode::taxi) or (assortment[i].type == Mode::stay)){
            opt_assortment.push_back(assortment[i]);
        }
    }
    return opt_assortment;
}

std::vector<Option> Controller::pool(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size());
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size);
    for (int i = 0; i < size; i++){
        if ((assortment[i].type == Mode::pool) or (assortment[i].type == Mode::stay)){
            opt_assortment.push_back(assortment[i]);
        }
    }
    return opt_assortment;
}

std::vector<Option> Controller::random(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size());
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size);
    for (int i = 0; i < size; i++){
        if ((urand(mt) < 0.5) or (assortment[i].type == Mode::stay)){
            opt_assortment.push_back(assortment[i]);
        }
    }
    return opt_assortment;
}

std::vector<Option> Controller::assort(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size()) - 1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size + 1);
    std::vector<float> R = get_assort_revenue(assortment, size);
    std::vector<float> V = get_assort_preference(assortment, size);
    float V0 = V.back();
    V.pop_back();
    float z = get_assort_opt_z(size, V, R, V0);
    opt_assortment.push_back(assortment[0]);
    for (int i = 0; i < size; i++){
        if (R[i] >= z){
            opt_assortment.push_back(assortment[i+1]);
        }
    }
    return opt_assortment;
}

std::vector<Option> Controller::pricing(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size()) - 1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size + 1);
    std::vector<float> R = get_assort_revenue(assortment, size);
    std::vector<float> V = get_assort_preference(assortment, size);
    float V0 = V.back();
    V.pop_back();
    std::vector<float> k = get_assort_elasticity(assortment, size);
    float z = get_pricing_opt_z(size, V, R, k, V0);
    opt_assortment.push_back(assortment[0]);
    Option temp;
    for (int i = 0; i < size; i++){
        temp = assortment[i+1];
        temp.adj_fare = std::max(std::min(z - R[i] + k[i], MAX_PRICE_SURGE), -MAX_PRICE_SURGE);
        opt_assortment.push_back(temp);
    }
    return opt_assortment;
}

std::vector<Option> Controller::assort_adjust(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size()) - 1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size + 1);
    std::vector<float> control_value(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> local_state = get_local_state(ori, des);
    int param_size = int(theta.size());
    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
    }
    std::vector<float> R = get_assort_revenue_adjust(assortment, size, control_value);
    std::vector<float> V = get_assort_preference(assortment, size);
    float V0 = V.back();
    V.pop_back();
    float z = get_assort_opt_z(size, V, R, V0);
    opt_assortment.push_back(assortment[0]);
    for (int i = 0; i < size; i++){
        if (R[i] >= z){
            opt_assortment.push_back(assortment[i+1]);
        }
    }
    return opt_assortment;
}

std::vector<Option> Controller::assort_model_free(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size()) - 1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    std::vector<float> control_value(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> mu = params.mu;
    int param_size = int(theta.size());
    std::vector<float> local_state = get_local_state(ori, des);
    std::vector<float> assort_state;
    float prob, temp;

    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
    }
    opt_assortment.push_back(assortment[0]);
    for (int i = 0; i < size; i++){
        assort_state = get_assort_state(assortment[i+1]);
        temp = 0;
        for (int j = 0; j < param_size; j++){
            temp += mu[j] * assort_state[j];
        }
        if (assortment[i].type == Mode::taxi) prob = 1.0 / (1.0 + exp(control_value[1] + temp));
        else prob = 1.0 / (1.0 + exp(control_value[0] + temp));
        if (urand(mt) < prob){
            opt_assortment.push_back(assortment[i+1]);
        }
    }
    return opt_assortment;
}

std::vector<Option> Controller::pricing_adjust(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size()) - 1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    std::vector<float> control_value(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> local_state = get_local_state(ori, des);
    int param_size = int(theta.size());
    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
    }
    std::vector<float> R = get_assort_revenue_adjust(assortment, size, control_value);
    std::vector<float> V = get_assort_preference(assortment, size);
    float V0 = V.back();
    V.pop_back();
    std::vector<float> k = get_assort_elasticity(assortment, size);
    float z = get_pricing_opt_z(size, V, R, k, V0);
    opt_assortment.push_back(assortment[0]);
    Option temp;
    for (int i = 0; i < size; i++){
        temp = assortment[i+1];
        temp.adj_fare = std::max(std::min(z - R[i] + k[i], MAX_PRICE_SURGE), -MAX_PRICE_SURGE);
        opt_assortment.push_back(temp);
    }
    return opt_assortment;
}

std::vector<Option> Controller::pricing_model_free(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size())-1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    std::vector<float> control_value(2,0.0);
    std::vector<float> control_value_std(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> mu = params.mu;
    std::vector<float> sigma = params.sigma;
    std::vector<float> kappa = params.kappa;
    std::vector<float> local_state = get_local_state(ori, des);
    std::vector<float> assort_state;
    int param_size = int(theta.size());

    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
        control_value_std[0] += sigma[i] * local_state[i];
        control_value_std[1] += sigma[i] * local_state[i+param_size];
    }

    float p, temp, temp_std;
    Option assort;
    float sample;

    opt_assortment.push_back(assortment[0]);
    for (int i = 0; i < size; i++){
        assort = assortment[i+1];
        assort_state = get_assort_state(assort);
        sample = nrand(mt);
        temp = 0;
        temp_std = 0;
        for (int j = 0; j < param_size; j++){
            temp += mu[j] * assort_state[j];
            temp_std += kappa[j] * assort_state[j];
        }
        if (assortment[i].type == Mode::taxi) p = control_value[0] + temp + exp(control_value_std[0] + temp_std) * sample;
        else p = control_value[1] + temp + exp(control_value_std[1] + temp_std) * sample;
        p = std::max(std::max(std::min(p, MAX_PRICE_SURGE), -MAX_PRICE_SURGE), -assort.fare);
        assort.adj_fare = p;
        opt_assortment.push_back(assort);
    }
    return opt_assortment;
}


std::vector<Option> Controller::assort_adjust_rand(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size())-1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    std::vector<float> control_value(2,0.0);
    std::vector<float> control_value_std(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> sigma = params.sigma;
    std::vector<float> theta_grad(12,0.0);
    std::vector<float> sigma_grad(12,0.0);
    std::vector<float> mu_grad(8,0.0);
    std::vector<float> kappa_grad(8,0.0);
    float alpha_grad = 0;
    std::vector<float> local_state = get_local_state(ori, des);
    int param_size = int(theta.size());
    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
        control_value_std[0] += sigma[i] * local_state[i];
        control_value_std[1] += sigma[i] * local_state[i+param_size];
    }
    float sample;
    std::vector<float> R(size,0.0);
    for (int i = 0; i < size; i++){
        R[i] = in_model->get_R(assortment[i]);
        sample = nrand(mt);
        if (assortment[i].type == Mode::taxi){
            R[i] += control_value[0] + exp(control_value_std[0]) * sample;
            for (int j = 0; j < param_size; j++){
                theta_grad[j] += (sample / exp(control_value_std[0])) * local_state[j];
                sigma_grad[j] += ((sample * sample) - 1.0) * local_state[j];
            }
        }else{
            R[i] += control_value[1] + exp(control_value_std[1]) * sample;
            for (int j = 0; j < param_size; j++){
                theta_grad[j] += (sample / exp(control_value_std[1])) * local_state[j + param_size];
                sigma_grad[j] += ((sample * sample) - 1.0) * local_state[j + param_size];
            }
        }
    }

    std::vector<float> V = get_assort_preference(assortment, size);
    float V0 = V.back();
    V.pop_back();
    float z = get_assort_opt_z(size, V, R, V0);
    opt_assortment.push_back(assortment[0]);
    for (int i = 0; i < size; i++){
        if (R[i] >= z){
            opt_assortment.push_back(assortment[i+1]);
        }
    }

    training_data.push_back({curr_time, 0, {theta_grad, sigma_grad, mu_grad, kappa_grad, alpha_grad}});

    return opt_assortment;
}

std::vector<Option> Controller::assort_adjust_post_rand(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size())-1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    std::vector<float> control_value(2,0.0);
    std::vector<float> control_value_std(2,0.0);
    std::vector<float> theta = params.theta;
    float alpha = params.alpha;
    std::vector<float> theta_grad(12,0.0);
    std::vector<float> sigma_grad(12,0.0);
    std::vector<float> mu_grad(8,0.0);
    std::vector<float> kappa_grad(8,0.0);
    float alpha_grad = 0;
    std::vector<float> local_state = get_local_state(ori, des);
    int param_size = int(theta.size());
    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
    }
    std::vector<float> R = get_assort_revenue_adjust(assortment, size, control_value);
    std::vector<float> V = get_assort_preference(assortment, size);
    float V0 = V.back();
    V.pop_back();
    float z = get_assort_opt_z(size, V, R, V0);
    opt_assortment.push_back(assortment[0]);
    float temp_p;
    int temp_ind;
    float diff,prob;
    for (int i = 0; i < size; i++){
        diff = std::max( std::min( (z - R[i]) / alpha, MAX_BIAS), -MAX_BIAS);
        prob = 1.0 / (1.0 + exp(diff));
        if (urand(mt) < prob){
            opt_assortment.push_back(assortment[i+1]);
            alpha_grad += (1.0 - prob) * (z - R[i]);
            temp_p = (1.0 - prob);
        }else{
            alpha_grad += - prob * (z - R[i]);
            temp_p = - prob;
        }
        if (assortment[i].type == Mode::taxi) temp_ind = 0;
        else temp_ind = param_size;

        for (int j = 0; j < param_size; j++){
            theta_grad[j] += temp_p * local_state[j + temp_ind];
        }
    }

    training_data.push_back({curr_time, 0, {theta_grad, sigma_grad, mu_grad, kappa_grad, alpha_grad}});

    return opt_assortment;
}

std::vector<Option> Controller::assort_model_free_rand(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size()) - 1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    opt_assortment.push_back(assortment[0]);
    std::vector<float> control_value(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> mu = params.mu;
    std::vector<float> theta_grad(12,0.0);
    std::vector<float> sigma_grad(12,0.0);
    std::vector<float> mu_grad(8,0.0);
    std::vector<float> kappa_grad(8,0.0);
    float alpha_grad = 0;
    int param_size = int(theta.size());
    std::vector<float> local_state = get_local_state(ori, des);
    std::vector<float> assort_state;
    float prob, temp;

    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
    }
    float temp_p;
    int temp_ind;
    for (int i = 0; i < size; i++){
        assort_state = get_assort_state(assortment[i+1]);
        temp = 0;
        for (int j = 0; j < param_size; j++){
            temp += mu[j] * assort_state[j];
        }
        if (assortment[i].type == Mode::taxi) prob = 1.0 / (1.0 + exp(control_value[1] + temp));
        else prob = 1.0 / (1.0 + exp(control_value[0] + temp));
        if (urand(mt) < prob){
            opt_assortment.push_back(assortment[i+1]);
            temp_p = prob - 1.0;
        }else{
            temp_p = prob;
        }
        if (assortment[i].type == Mode::taxi) temp_ind = 0;
        else temp_ind = param_size;
        for (int j = 0; j < param_size; j++){
            mu_grad[j] += temp_p * assort_state[j];
            theta_grad[j] += temp_p * local_state[j + temp_ind];
        }
    }

    training_data.push_back({curr_time, 0, {theta_grad, sigma_grad, mu_grad, kappa_grad, alpha_grad}});

    return opt_assortment;
}

std::vector<Option> Controller::pricing_adjust_rand(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size())-1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    opt_assortment.push_back(assortment[0]);
    std::vector<float> control_value(2,0.0);
    std::vector<float> control_value_std(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> sigma = params.sigma;
    std::vector<float> theta_grad(12,0.0);
    std::vector<float> sigma_grad(12,0.0);
    std::vector<float> mu_grad(8,0.0);
    std::vector<float> kappa_grad(8,0.0);
    float alpha_grad = 0;
    std::vector<float> local_state = get_local_state(ori, des);
    int param_size = int(theta.size());
    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
        control_value_std[0] += sigma[i] * local_state[i];
        control_value_std[1] += sigma[i] * local_state[i+param_size];
    }
    std::vector<float> R(size, 0.0);
    float sample;
    for (int i = 0; i < size; i++){
        R[i] = in_model->get_R(assortment[i+1]);
        sample = nrand(mt);
        if (assortment[i].type == Mode::taxi){
            R[i] += control_value[0] + exp(control_value_std[0]) * sample;
            for (int j = 0; j < param_size; j++){
                theta_grad[j] += (sample / exp(control_value_std[0])) * local_state[j];
                sigma_grad[j] += ((sample * sample) - 1.0) * local_state[j];
            }
        }else{
            R[i] += control_value[1] + exp(control_value_std[1]) * sample;
            for (int j = 0; j < param_size; j++){
                theta_grad[j] += (sample / exp(control_value_std[0])) * local_state[j + param_size];
                sigma_grad[j] += ((sample * sample) - 1.0) * local_state[j + param_size];
            }
        }
    }
    std::vector<float> V = get_assort_preference(assortment, size);
    float V0 = V.back();
    V.pop_back();
    std::vector<float> k = get_assort_elasticity(assortment, size);
    float z = get_pricing_opt_z(size, V, R, k, V0);
    Option temp;
    for (int i = 0; i < size; i++){
        temp = assortment[i+1];
        temp.adj_fare = std::max(std::min(z - R[i] + k[i], MAX_PRICE_SURGE), -MAX_PRICE_SURGE);
        opt_assortment.push_back(temp);
    }

    training_data.push_back({curr_time, 0, {theta_grad, sigma_grad, mu_grad, kappa_grad, alpha_grad}});

    return opt_assortment;
}

std::vector<Option> Controller::pricing_adjust_post_rand(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size())-1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    opt_assortment.push_back(assortment[0]);
    std::vector<float> control_value(2,0.0);
    std::vector<float> control_value_std(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> sigma = params.sigma;
    std::vector<float> theta_grad(12,0.0);
    std::vector<float> sigma_grad(12,0.0);
    std::vector<float> mu_grad(8,0.0);
    std::vector<float> kappa_grad(8,0.0);
    float alpha_grad = 0;
    std::vector<float> local_state = get_local_state(ori, des);
    int param_size = int(theta.size());
    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
        control_value_std[0] += sigma[i] * local_state[i];
        control_value_std[1] += sigma[i] * local_state[i+param_size];
    }
    std::vector<float> R = get_assort_revenue(assortment, size);
    std::vector<float> V = get_assort_preference(assortment, size);
    float V0 = V.back();
    V.pop_back();
    std::vector<float> k = get_assort_elasticity(assortment, size);
    float z = get_pricing_opt_z(size, V, R, k, V0);
    Option temp;
    float temp_adj_fare;
    float sample;
    for (int i = 0; i < size; i++){
        temp = assortment[i+1];
        temp_adj_fare = z - R[i] + k[i];
        sample = nrand(mt);
        if (temp.type == Mode::taxi){
            temp_adj_fare += control_value[0] + exp(control_value_std[0]) * sample;
            for (int j = 0; j < param_size; j++){
                theta_grad[j] += (sample / exp(control_value_std[0])) * local_state[j];
                sigma_grad[j] += ((sample * sample) - 1.0) * local_state[j];
            }
        }else{
            temp_adj_fare += control_value[1] + exp(control_value_std[1]) * sample;
            for (int j = 0; j < param_size; j++){
                theta_grad[j] += (sample / exp(control_value_std[1])) * local_state[j + param_size];
                sigma_grad[j] += ((sample * sample) - 1.0) * local_state[j + param_size];
            }
        }
        temp.adj_fare = std::max(std::min(temp_adj_fare, MAX_PRICE_SURGE), -MAX_PRICE_SURGE);
        opt_assortment.push_back(temp);
    }

    training_data.push_back({curr_time, 0, {theta_grad, sigma_grad, mu_grad, kappa_grad, alpha_grad}});

    return opt_assortment;
}

std::vector<Option> Controller::pricing_model_free_rand(std::vector<Option> assortment, int ori, int des){
    int size = int(assortment.size())-1;
    std::vector<Option> opt_assortment;
    opt_assortment.reserve(size+1);
    opt_assortment.push_back(assortment[0]);
    std::vector<float> control_value(2,0.0);
    std::vector<float> control_value_std(2,0.0);
    std::vector<float> theta = params.theta;
    std::vector<float> mu = params.mu;
    std::vector<float> sigma = params.sigma;
    std::vector<float> kappa = params.kappa;
    std::vector<float> theta_grad(12,0.0);
    std::vector<float> sigma_grad(12,0.0);
    std::vector<float> mu_grad(8,0.0);
    std::vector<float> kappa_grad(8,0.0);
    float alpha_grad = 0;
    std::vector<float> local_state = get_local_state(ori, des);
    std::vector<float> assort_state;
    int param_size = int(theta.size());

    for (int i = 0; i < param_size; i++){
        control_value[0] += theta[i] * local_state[i];
        control_value[1] += theta[i] * local_state[i+param_size];
        control_value_std[0] += sigma[i] * local_state[i];
        control_value_std[1] += sigma[i] * local_state[i+param_size];
    }

    float p, temp, temp_std;
    Option assort;
    float sample;

    for (int i = 0; i < size; i++){
        assort = assortment[i+1];
        assort_state = get_assort_state(assort);
        sample = nrand(mt);
        temp = 0;
        temp_std = 0;
        for (int j = 0; j < param_size; j++){
            temp += mu[j] * assort_state[j];
            temp_std += kappa[j] * assort_state[j];
        }
        if (assortment[i+1].type == Mode::taxi){
            p = control_value[0] + temp + exp(control_value_std[0] + temp_std) * sample;
            for (int j = 0; j < param_size; j++){
                theta_grad[j] += (sample / exp(control_value_std[0] + temp_std)) * local_state[j];
                mu_grad[j] += (sample / exp(control_value_std[0] + temp_std)) * assort_state[j];
                sigma_grad[j] += (sample * sample - 1.0) * local_state[j];
                kappa_grad[j] += (sample * sample - 1.0) * assort_state[j];
            }
        }else{
            p = control_value[1] + temp + exp(control_value_std[1] + temp_std) * sample;
            for (int j = 0; j < param_size; j++){
                theta_grad[j] += (sample / exp(control_value_std[1] + temp_std)) * local_state[j + param_size];
                mu_grad[j] += (sample / exp(control_value_std[1] + temp_std)) * assort_state[j];
                sigma_grad[j] += (sample * sample - 1.0) * local_state[j + param_size];
                kappa_grad[j] += (sample * sample - 1.0) * assort_state[j];
            }
        }
        p = std::max(std::max(std::min(p, MAX_PRICE_SURGE), -MAX_PRICE_SURGE), -assort.fare);
        assort.adj_fare = p;
        opt_assortment.push_back(assort);
    }

    training_data.push_back({curr_time, 0, {theta_grad, sigma_grad, mu_grad, kappa_grad, alpha_grad}});

    return opt_assortment;
}
