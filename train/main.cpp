//
//  main.cpp
//  train
//
//  Created by Han Qiu on 4/4/17.
//  Copyright © 2017 Han Qiu. All rights reserved.
//

#include <iostream>
#include <cmath>
#include <armadillo.h>
#include "scenario.hpp"
#include "load.hpp"


typedef std::vector<float> stdvec;

int main(int argc, const char * argv[]) {
    if (argc != 9){
        std::cout<<"error!\n";
        return -1;
    }
    else{
        // load training input
        std::string data_path = argv[1];
        int dynamic_time_flag = atoi(argv[2]);
        int i_algo = atoi(argv[3]);
        int i_demand = atoi(argv[4]);
        int i_congest = atoi(argv[5]);
        int i_supply = atoi(argv[6]);
        int i_prate = atoi(argv[7]);
        int i_tax = atoi(argv[8]);

        // initial scenario setting
        std::vector<float> supply_factor_range = {0.6,0.8,1.0,1.2,1.4};
        std::vector<float> demand_factor_range = {1.0,3.0,6.0};
        std::vector<float> p_rate_range = {0.3,0.4,0.5,0.6,0.7,0.8};

        Algorithm algo = Algorithm::full;
        int congest_level = 1;
        float supply_factor = 1.0;
        float demand_factor = 1.0;
        float p_rate = 0.5;
        float tax_congest = -0.1;
        float tax_demand = -0.5;

        if (i_algo == 0) algo = Algorithm::pricing_adjust;
        if (i_algo == 1) algo = Algorithm::assort_adjust;
        congest_level = i_congest + 1;
        demand_factor = demand_factor_range[i_demand];
        supply_factor = supply_factor_range[i_supply];
        p_rate = p_rate_range[i_prate];
        switch (i_tax) {
            case 0:
                tax_congest = 0.0;
                tax_demand = 0.0;
                break;
            case 1:
                tax_congest = -0.1;
                tax_demand = -0.5;
                break;
            default:
                tax_congest = 0.0;
                tax_demand = 0.0;
                break;
        }

        int debug_flag = 0;

        // load simulation input
        int grid_size = 10;
        std::uniform_int_distribution<int> irand(0,grid_size * grid_size-1);
        int total_time = 1440;
        float congestion_factor;
        int dynamic_travel_time_flag = dynamic_time_flag;
        float dynamic_travel_time_rate = demand_factor / 30.0;
        int fleet_size = 50 * supply_factor;
        std::vector< std::vector<float> > ori_dist = load(data_path+"/o_d_c.csv",total_time,grid_size);
        std::vector< std::vector<float> > des_dist = load(data_path+"/d_d_c.csv",total_time,grid_size);
        std::vector< std::vector<float> > travel_time;
        switch (congest_level) {
            case 1:
                congestion_factor = 1.0;
                travel_time = load(data_path+"/t_d_c_1_0.csv",total_time,grid_size);
                break;
            case 2:
                congestion_factor = 0.8;
                travel_time = load(data_path+"/t_d_c_0_8.csv",total_time,grid_size);
                break;
            case 3:
                congestion_factor = 1.2;
                travel_time = load(data_path+"/t_d_c_1_2.csv",total_time,grid_size);
                break;
            default:
                congestion_factor = 1.0;
                travel_time = load(data_path+"/t_d_c_1_0.csv",total_time,grid_size);
                break;
        }

        // initialize simulation input
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
        Scenario_Setting scenario_setting = {grid_size, p_rate, tax_congest, tax_demand, dynamic_travel_time_flag, dynamic_travel_time_rate, ori_dist, des_dist,travel_time,algo};
        Scenario s = Scenario(scenario_setting);

        // define basic parameters
        int train_iter = 30;
        int train_sample = 5;
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
        std::cout<<congestion_factor<<","<<demand_factor<<","<<supply_factor<<","<<p_rate<<","<<tax_congest<<","<<tax_demand;
        for (int i = 0; i < 12; i++){
            std::cout<<","<<theta_opt(i);
        }
        std::cout<<"\n";
    }

    return 0;
}
