//
//  load.cpp
//  proactive_dispatch_c
//
//  Created by Han Qiu on 4/25/17.
//  Copyright © 2017 Han Qiu. All rights reserved.
//

#include "load.hpp"

std::vector< std::vector<float> > load(std::string path,int total_time, int grid_size, float factor){
    std::vector< std::vector<float> > dist;
    dist.reserve(total_time);
    std::vector<float> dist_temp;
    dist_temp.reserve(grid_size * grid_size);
    std::ifstream f;
    std::string line;
    std::string cell;
    f.open(path, std::ifstream::in);
    for(int i = 0; i < total_time; i++)
    {
        std::getline(f,line);
        std::stringstream lineStream(line);
        dist_temp = {};
        for(int j = 0; j < grid_size * grid_size; j++)
        {
            std::getline(lineStream,cell,',');
            dist_temp.push_back(std::stof(cell) * factor);
        }
        dist.push_back(dist_temp);
    }
    f.close();
    return dist;
}
