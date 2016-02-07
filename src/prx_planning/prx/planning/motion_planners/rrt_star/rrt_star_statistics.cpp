/**
 * @file rrt_star_statistics.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/motion_planners/rrt_star/rrt_star_statistics.hpp"

namespace prx 
{ 
    using namespace util;
    namespace plan 
    {

rrt_star_statistics_t::rrt_star_statistics_t() : rrt_statistics_t()
{
}

rrt_star_statistics_t::~rrt_star_statistics_t() { }

std::string rrt_star_statistics_t::get_statistics() const
{
    std::stringstream out(std::stringstream::out);
    
    out << statistics_t::get_statistics()  << ", " << num_vertices <<",  "<<solution_quality  << ", " << average_cost << std::endl ;
    
    return out.str();
}


    }
}
