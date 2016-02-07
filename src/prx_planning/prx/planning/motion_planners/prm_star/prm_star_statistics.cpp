/**
 * @file prm_star_statistics.cpp
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

#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"

namespace prx 
{ 
    using namespace util;
    namespace plan 
    {

prm_star_statistics_t::prm_star_statistics_t() 
{
    num_vertices = 0;
    num_edges = 0;
}

prm_star_statistics_t::~prm_star_statistics_t() { }

std::string prm_star_statistics_t::get_statistics() const
{
    std::stringstream out(std::stringstream::out);
    
    out << num_vertices << " , " << num_edges;
    
    return out.str();
}

bool prm_star_statistics_t::serialize(std::ofstream& stream) const
{
    if(stream.is_open())
    {
        stream << get_statistics();
        return true;
    }
    return false;
}

    }
}
