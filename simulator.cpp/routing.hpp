//
//  routing.hpp
//  simulator_c
//
//  Created by Han Qiu on 12/8/16.
//  Copyright © 2016 Han Qiu. All rights reserved.
//

#ifndef routing_hpp
#define routing_hpp

#include "common.hpp"

class Routing{
    int grid_size = 0;
    int max_value = 0;
    std::vector< std::array<int, 4> > network;
    std::vector<float> travel_time;
    
    // reserved temporary variable to reduce overhead
    float ori_time,des_time;
    std::vector<int> cameFromOri,cameFromDes;
    std::vector<float> gScoreOri,gScoreDes;
    std::vector<float> fScoreOri,fScoreDes;
    
public:
    Routing(int n);
    void update_travel_time(std::vector<float> new_travel_time);
    RoutingOutput accurate(int ori, int des);
    RoutingOutput estimate(int ori, int des);
    int get_grid_size();
};

#endif /* routing_hpp */
