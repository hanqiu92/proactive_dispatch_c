//
//  load.hpp
//  proactive_dispatch_c
//
//  Created by Han Qiu on 4/25/17.
//  Copyright © 2017 Han Qiu. All rights reserved.
//

#ifndef load_hpp
#define load_hpp

#include <fstream>
#include <sstream>
#include <vector>

std::vector< std::vector<float> > load(std::string path, int total_time, int grid_size, float factor);

#endif /* load_hpp */
