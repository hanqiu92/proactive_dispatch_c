//
//  train.cpp
//  simulator_c
//
//  Created by Han Qiu on 3/21/17.
//  Copyright © 2017 Han Qiu. All rights reserved.
//

#include "train.hpp"
#include <cmath>
#include <armadillo.h>

typedef std::vector<float> stdvec;

int train(Algorithm algo, int congestion_level, float demand_factor, float supply_factor, float p_rate, float tax_congest, float tax_demand, int dynamic_time_flag, int debug_flag){
    if ((algo == Algorithm::assort_adjust) or (algo == Algorithm::pricing_adjust)){
        // load simulation input
        int grid_size = 10;
        std::uniform_int_distribution<int> irand(0,grid_size * grid_size-1);
        int total_time = 1440;
        float congestion_factor;
        int dynamic_travel_time_flag = dynamic_time_flag;
        float dynamic_travel_time_rate = demand_factor / 10.0;
        float demand_scale_factor = 3.0;
        int fleet_size = 50 * supply_factor * int(demand_scale_factor);
        std::vector< std::vector<float> > ori_dist = load("/Users/hanqiu/proactive_dispatch_c/data/o_d_c.csv",total_time,grid_size,demand_scale_factor);
        std::vector< std::vector<float> > des_dist = load("/Users/hanqiu/proactive_dispatch_c/data/d_d_c.csv",total_time,grid_size,demand_scale_factor);
        std::vector< std::vector<float> > travel_time;
        switch (congestion_level) {
            case 1:
                congestion_factor = 1.0;
                travel_time = load("/Users/hanqiu/proactive_dispatch_c/data/t_d_c_1_0.csv",total_time,grid_size,1.0);
                break;
            case 2:
                congestion_factor = 0.8;
                travel_time = load("/Users/hanqiu/proactive_dispatch_c/data/t_d_c_0_8.csv",total_time,grid_size,1.0);
                break;
            case 3:
                congestion_factor = 1.2;
                travel_time = load("/Users/hanqiu/proactive_dispatch_c/data/t_d_c_1_2.csv",total_time,grid_size,1.0);
                break;
            default:
                congestion_factor = 1.0;
                travel_time = load("/Users/hanqiu/proactive_dispatch_c/data/t_d_c_1_0.csv",total_time,grid_size,1.0);
                break;
        }

        std::vector<float> thetaa(12,0.0);
        std::vector<float> sigmaa(12,0.0);
        std::vector<float> muu(8,0.0);
        std::vector<float> kappaa(8,0.0);
        float alphaa = 0.0;
        Agent_Setting agent_setting = {1,false,{thetaa,sigmaa,muu,kappaa,alphaa}};
        std::vector<VehicleState> vs;
        vs.reserve(fleet_size);
        std::random_device rd;
        std::mt19937 mt(rd());
        for (int i = 0; i < fleet_size; i++){
            vs.push_back({irand(mt),0,0,{}});
        }
        float density_factor = demand_scale_factor * (1.0 + 0.25 * (congestion_factor - 1.0));
        Scenario_Setting scenario_setting = {grid_size, p_rate, tax_congest, tax_demand, dynamic_travel_time_flag, dynamic_travel_time_rate, ori_dist, des_dist,travel_time,density_factor,demand_scale_factor,algo};
        Scenario s = Scenario(scenario_setting);

        // process training input
        int train_iter = 1;
        int train_sample = 5;
        std::string save_path;
        switch (algo) {
            case Algorithm::assort_adjust:
                save_path = "/Users/hanqiu/proactive_dispatch_c/result/assort_opt_param.csv";
                break;
            case Algorithm::pricing_adjust:
                save_path = "/Users/hanqiu/proactive_dispatch_c/result/price_opt_param.csv";
                break;
            default:
                save_path = "/Users/hanqiu/proactive_dispatch_c/result/assort_opt_param.csv";
                break;
        }

        // define basic parameters
        int n = 12;
        int lam = 12;
        float sigma = 0.25;
        int mu = 3;
        float c_c = 4.0 / float(n + 4.0);
        float c_sigma = c_c;
        float c_w = std::sqrt(mu);
        float d_sigma = 1.0 / c_sigma + 1.0;
        //float alpha_cov = 1.0 / mu;
        float alpha_cov = 1.0;
        float c_cov = alpha_cov * 2 / ((n + std::sqrt(2)) * (n + std::sqrt(2))) + (1 - alpha_cov) * std::min( 1, (2 * mu - 1) / ( (n + 2) * (n + 2) - mu) );
        float chi = std::sqrt(n) * (1 - 1.0 / 4 / n + 1.0 / 21 / (n * n));

        arma::vec p_c(n,arma::fill::zeros);
        arma::vec p_sigma(n,arma::fill::zeros);
        arma::mat C(n,n,arma::fill::eye);
        arma::vec d(n,arma::fill::ones);
        arma::mat D(n,n,arma::fill::eye);
        arma::mat B(n,n,arma::fill::eye);

        arma::mat theta(n,lam,arma::fill::zeros);
        arma::mat z(n,lam,arma::fill::zeros);
        arma::vec theta_m(n,arma::fill::zeros);
        arma::vec z_m(n,arma::fill::zeros);
        arma::mat H(n,n,arma::fill::zeros);
        arma::mat Z(n,n,arma::fill::zeros);

        arma::uvec ind(lam,arma::fill::zeros);
        arma::vec re(lam,arma::fill::zeros);
        arma::vec theta_opt(n,arma::fill::zeros);
        arma::vec z_temp(n,arma::fill::zeros);
        float r;
        float r_opt = 0.0;

        for (int i = 0; i < train_iter; i++){
            if (i > 0){
                theta_m.zeros();
                for (int j = 0; j < mu; j++){
                    theta_m += theta.col(ind(j));
                    theta_m = theta_m / double(mu);
                }
            }

            // get sample value
            for (int j = 0; j < lam; j++){
                z.col(j).randn();
                theta.col(j) = theta_m + sigma * B * D * z.col(j);
                thetaa = arma::conv_to<stdvec>::from(theta.col(j));
                agent_setting = {1,true,{thetaa,sigmaa,muu,kappaa,alphaa}};
                re(j) = 0.0;
                for (int i = 0; i < train_sample; i++){
                    re(j) += s.simulate(0,1440,fleet_size,vs,agent_setting).agent_out.back().profit;
                }
                re(j) = re(j) / float(train_sample);
                if (re(j) > r_opt){
                    r_opt = re(j);
                    theta_opt = theta.col(j);
                }
            }
            ind = arma::sort_index(re,"descend");

            // get current value
            thetaa = arma::conv_to<stdvec>::from(theta_m);
            agent_setting = {1,true,{thetaa,sigmaa,muu,kappaa,alphaa}};
            r = 0.0;
            for (int i = 0; i < train_sample; i++){
                r += s.simulate(0,1440,fleet_size,vs,agent_setting).agent_out.back().profit;
            }
            r = r / float(train_sample);
            if (r > r_opt){
                r_opt = r;
                theta_opt = theta_m;
            }

            if (debug_flag > 0){
                std::cout<<"iter "<<i<<":\n";
                std::cout<<"theta = "<<theta_m.t()<<", sigma = "<<sigma<<"\n";
                std::cout<<"value = "<<r<<", value mu = "<<arma::mean(re(ind.subvec(0,mu)))<<", value all = "<<arma::mean(re)<<", pool = "<<re(ind).t()<<"\n";
            }

            z_m.zeros();
            H.zeros();
            for (int j = 0; j < mu; j++){
                z_temp = z.col(ind(j));
                z_m += z_temp;
                H += z_temp * z_temp.t();

            }
            z_m = z_m / double(mu);
            H = H / double(mu);

            p_sigma = (1.0 - c_sigma) * p_sigma + std::sqrt(c_sigma * (2.0 - c_sigma)) * c_w * B * z_m;
            sigma = sigma * std::exp( (arma::norm(p_sigma,2) - chi) / d_sigma / chi);
            if (debug_flag > 0){
                std::cout<<"norm_z: "<<arma::norm(z_m,2) * c_w<<", norm_p: "<<arma::norm(p_sigma,2)<<", chi: "<<chi<<"\n\n";
            }

            p_c = (1.0 - c_c) * p_c + std::sqrt(c_c * (2.0 - c_c)) * c_w * B * D * z_m;
            Z = (B * D) * H * arma::trans(B * D);
            C = (1-c_cov) * C + c_cov * (alpha_cov * p_c * p_c.t() + (1 - alpha_cov) * Z);
            arma::eig_sym(d,B,C,"std");
            D = arma::diagmat(arma::sqrt(d % (d >= 0.0)));
        }

        // get current value
        thetaa = arma::conv_to<stdvec>::from(theta_opt);
        if (debug_flag > 0){
            std::cout<<"optimal theta = "<<theta_opt.t()<<"\n";
        }

        // format output
        std::ofstream f;
        f.open(save_path, std::ios::out|std::ios::app);
        f<<congestion_factor<<","<<demand_factor<<","<<supply_factor<<","<<p_rate<<","<<tax_congest<<","<<tax_demand<<",";
        for (int i = 0; i < n-1; i++){
            f<<thetaa[i]<<",";
        }
        f<<thetaa[n-1]<<"\n";

        f.close();
    }

    return 0;
}

int get_opt_param(int algo_low, int algo_upp, int congestion_low, int congestion_upp, int demand_low, int demand_upp, int supply_low, int supply_upp, int p_rate_low, int p_rate_upp, int tax_c_low, int tax_c_upp, int tax_d_low, int tax_d_upp, int dynamic_time_flag, int debug_flag){
    std::vector<float> supply_factor_range = {0.75,1.0,1.25,1.5};
    std::vector<float> demand_factor_range = {1.0,2.0,4.0};
    std::vector<float> p_rate_range = {0.4,0.6,0.8};
    std::vector<float> tax_c_range = {0.0,0.025};
    std::vector<float> tax_d_range = {0.0,0.25};

    Algorithm algo = Algorithm::full;
    int congest_level = 1;
    float supply_factor = 1.0;
    float demand_factor = 1.0;
    float p_rate = 0.8;
    float tax_congest = 0.0;
    float tax_demand = 0.0;

    for (int i_algo = algo_low; i_algo <= algo_upp; i_algo++){
        if (i_algo == 0){
            algo = Algorithm::pricing_adjust;
            std::cout<<"Get pricing params: \n";
        }
        if (i_algo == 1){
            algo = Algorithm::assort_adjust;
            std::cout<<"Get assortment params: \n";
        }
        for (int i_congest = congestion_low; i_congest <= congestion_upp; i_congest++){
            congest_level = i_congest + 1;
            for (int i_demand = demand_low; i_demand <= demand_upp; i_demand++){
                demand_factor = demand_factor_range[i_demand];
                for (int i_supply = supply_low; i_supply <= supply_upp; i_supply++){
                    supply_factor = supply_factor_range[i_supply];
                    for (int i_prate = p_rate_low; i_prate <= p_rate_upp; i_prate++){
                        p_rate = p_rate_range[i_prate];
                        for (int i_tax_c = tax_c_low; i_tax_c <= tax_c_upp; i_tax_c++){
                            tax_congest = - tax_c_range[i_tax_c];
                            for (int i_tax_d = tax_d_low; i_tax_d <= tax_d_upp; i_tax_d++){
                                tax_demand = - tax_d_range[i_tax_d];
                                
                                std::cout<<"Scenario with "<<congest_level<<","<<supply_factor<<","<<demand_factor<<","<<p_rate<<","<<tax_congest<<","<<tax_demand<<": ";
                                train(algo,congest_level,demand_factor,supply_factor,p_rate,tax_congest,tax_demand,dynamic_time_flag,debug_flag);
                                std::cout<<"Done. \n";
                            }
                        }
                    }
                }
            }
        }
    }

    return 0;
}
