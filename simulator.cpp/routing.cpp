//
//  routing.cpp
//  simulator_c
//
//  Created by Han Qiu on 12/8/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#include "routing.hpp"

Routing::Routing(int n){
    // initiate variables
    grid_size = n;
    int m = n * n;
    max_value = 60 * n;
    network.reserve(m);
    travel_time.reserve(m);
    
    int n_cell = 0;
    for (int n_col = 0; n_col < n; n_col++){
        for (int n_row = 0; n_row < n; n_row++){
            n_cell = n * n_col + n_row;
            travel_time.push_back(1.0);
            // initiate the location info matrix
            network.push_back({n_cell - 1, n_cell + 1, n_cell - n, n_cell + n});
            // deal with margin manually
            if (n_col == 0) network[n_cell][2] = -1;
            if (n_col == (n-1)) network[n_cell][3] = -1;
            if (n_row == 0) network[n_cell][0] = -1;
            if (n_row == (n-1)) network[n_cell][1] = -1;
        }
    }
    
    // temporary variables
    ori_time = 1.0;
    des_time = 1.0;
    cameFromOri.assign(m,-1);
    cameFromDes.assign(m,-1);
    fScoreOri.assign(m,max_value);
    fScoreDes.assign(m,max_value);
    gScoreOri.assign(m,max_value);
    fScoreDes.assign(m,max_value);
    
}

void Routing::update_travel_time(std::vector<float> new_travel_time){
    travel_time = new_travel_time;
}

// A star
RoutingOutput Routing::accurate(int ori, int des){
    std::set<int> closedSet;
    std::set<int> openSet;
    openSet.insert(ori);

    int current,node,temp;
    float value,tentative_gScore,total_time;
    ori_time = travel_time[ori];
    des_time = travel_time[des];
    
    cameFromOri.assign(grid_size * grid_size,-1);
    gScoreOri.assign(grid_size * grid_size,max_value);
    fScoreOri.assign(grid_size * grid_size,max_value);
    
    gScoreOri[ori] = ori_time / 2.0;
    fScoreOri[ori] = ori_time / 2.0 + abs(ori % grid_size - des % grid_size) +
                  abs(ori / grid_size - des / grid_size) + des_time / 2.0 - 0.5;

    std::set<int>::iterator it;
    while (!openSet.empty()){
        value = max_value;
        current = -1;
        for (it = openSet.begin(); it!= openSet.end(); ++it){
            temp = *it;
            if (fScoreOri[temp] < value){
                value = fScoreOri[temp];
                current = temp;
            }
        }
        
        if (current == des){
            // do the output
            total_time = fScoreOri[current];
            std::vector<int> total_path = {current};
            while (cameFromOri[current] >= 0){
                current = cameFromOri[current];
                total_path.push_back(current);
            }
            std::reverse(total_path.begin(),total_path.end());
            RoutingOutput out = {total_time,total_path,int(total_path.size())};
            return out;
        }else{
            openSet.erase(current);
            closedSet.insert(current);
            for (int i = 0; i < 4; i++){
                node = network[current][i];
                if (node >= 0){
                    if (closedSet.find(node) == closedSet.end()){
                        tentative_gScore = gScoreOri[current] + (travel_time[current] + travel_time[node]) / 2.0;
                        if (openSet.find(node) == openSet.end()){
                            openSet.insert(node);
                            cameFromOri[node] = current;
                            gScoreOri[node] = tentative_gScore;
                            fScoreOri[node] = gScoreOri[node] + abs(node / grid_size - des / grid_size) + abs(node % grid_size - des % grid_size) + des_time / 2.0 - 0.5;
                        }else{
                            if (tentative_gScore < gScoreOri[node]){
                                cameFromOri[node] = current;
                                gScoreOri[node] = tentative_gScore;
                                fScoreOri[node] = gScoreOri[node] + abs(node / grid_size - des / grid_size) + abs(node % grid_size - des % grid_size) + des_time / 2.0 - 0.5;
                            }
                        }
                    }
                }
            }
        }
    }
    
    std::vector<int> total_path = {-1};
    RoutingOutput out = {-1,total_path,-1};
    return out;
}


// bi A star
RoutingOutput Routing::estimate(int ori, int des){
    std::set<int> closedSetOri;
    std::set<int> openSetOri;
    std::set<int> closedSetDes;
    std::set<int> openSetDes;
    
    cameFromOri.assign(grid_size * grid_size,-1);
    cameFromDes.assign(grid_size * grid_size,-1);
    gScoreOri.assign(grid_size * grid_size,max_value);
    fScoreOri.assign(grid_size * grid_size,max_value);
    gScoreDes.assign(grid_size * grid_size,max_value);
    fScoreDes.assign(grid_size * grid_size,max_value);
    
    ori_time = travel_time[ori];
    des_time = travel_time[des];

    int current,node;
    float value,tentative_gScore,total_time;
    int direction_flag = 0;
    
    openSetOri.insert(ori);
    openSetDes.insert(des);
    
    gScoreOri[ori] = ori_time / 2.0;
    fScoreOri[ori] = ori_time / 2.0 + abs(ori % grid_size - des % grid_size) +
    abs(ori / grid_size - des / grid_size) + des_time / 2.0 - 0.5;
    
    gScoreDes[des] = des_time / 2.0;
    fScoreDes[des] = des_time / 2.0 + abs(ori % grid_size - des % grid_size) +
    abs(ori / grid_size - des / grid_size) + ori_time / 2.0 - 0.5;
    
    std::vector<int> intersection;
    std::set<int>::iterator it;
    while ((!openSetOri.empty()) && (!openSetDes.empty())){
        std::set_intersection(openSetOri.begin(), openSetOri.end(),
                              openSetDes.begin(), openSetDes.end(),
                              std::back_inserter(intersection));
        
        if (!intersection.empty()){
            // do the output
            current = intersection[0];
            total_time = gScoreOri[current] + gScoreDes[current];
            std::vector<int> total_path_ori = {current};
            std::vector<int> total_path_des = {};
            int currentOri = current;
            int currentDes = current;
            while (cameFromOri[currentOri] >= 0){
                currentOri = cameFromOri[currentOri];
                total_path_ori.push_back(currentOri);
            }
            std::reverse(total_path_ori.begin(), total_path_ori.end());
            while (cameFromDes[currentDes] >= 0){
                currentDes = cameFromDes[currentDes];
                total_path_des.push_back(currentDes);
            }
            total_path_ori.insert(total_path_ori.end(),total_path_des.begin(),total_path_des.end());
            RoutingOutput out = {total_time,total_path_ori,int(total_path_ori.size())};
            return out;
        }else{
            value = max_value;
            if (direction_flag == 0){
                current = -1;
                for (it = openSetOri.begin(); it!= openSetOri.end(); ++it){
                    int temp = *it;
                    if (fScoreOri[temp] < value){
                        value = fScoreOri[temp];
                        current = temp;
                    }
                }
                openSetOri.erase(current);
                closedSetOri.insert(current);
                for (int i = 0; i < 4; i++){
                    node = network[current][i];
                    if (node >= 0){
                        if (closedSetOri.find(node) == closedSetOri.end()){
                            tentative_gScore = gScoreOri[current] + (travel_time[current] + travel_time[node]) / 2.0;
                            if (openSetOri.find(node) == openSetOri.end()){
                                openSetOri.insert(node);
                                cameFromOri[node] = current;
                                gScoreOri[node] = tentative_gScore;
                                fScoreOri[node] = gScoreOri[node] + abs(node / grid_size - des / grid_size) + abs(node % grid_size - des % grid_size) + des_time / 2.0 - 0.5;
                            }else{
                                if (tentative_gScore < gScoreOri[node]){
                                    cameFromOri[node] = current;
                                    gScoreOri[node] = tentative_gScore;
                                    fScoreOri[node] = gScoreOri[node] + abs(node / grid_size - des / grid_size) + abs(node % grid_size - des % grid_size) + des_time / 2.0 - 0.5;
                                }
                            }
                        }
                    }
                }
            }else{
                current = -1;
                for (it = openSetDes.begin(); it!= openSetDes.end(); ++it){
                    int temp = *it;
                    if (fScoreDes[temp] < value){
                        value = fScoreDes[temp];
                        current = temp;
                    }
                }
                openSetDes.erase(current);
                closedSetDes.insert(current);
                for (int i = 0; i < 4; i++){
                    node = network[current][i];
                    if (node >= 0){
                        if (closedSetDes.find(node) == closedSetDes.end()){
                            tentative_gScore = gScoreDes[current] + (travel_time[current] + travel_time[node]) / 2.0;
                            if (openSetDes.find(node) == openSetDes.end()){
                                openSetDes.insert(node);
                                cameFromDes[node] = current;
                                gScoreDes[node] = tentative_gScore;
                                fScoreDes[node] = gScoreDes[node] + abs(node / grid_size - ori / grid_size) + abs(node % grid_size - ori % grid_size) + ori_time / 2.0 - 0.5;
                            }else{
                                if (tentative_gScore < gScoreDes[node]){
                                    cameFromDes[node] = current;
                                    gScoreDes[node] = tentative_gScore;
                                    fScoreDes[node] = gScoreDes[node] + abs(node / grid_size - ori / grid_size) + abs(node % grid_size - ori % grid_size) + ori_time / 2.0 - 0.5;
                                }
                            }
                        }
                    }
                }
            }
            
            direction_flag = 1 - direction_flag;
        }
    }
    
    std::vector<int> total_path = {-1};
    RoutingOutput out = {-1,total_path,-1};
    return out;
}

int Routing::get_grid_size(){
    return grid_size;
}
