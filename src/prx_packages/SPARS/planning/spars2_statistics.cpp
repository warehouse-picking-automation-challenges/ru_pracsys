/**
 * @file spars2_statistics.cpp
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

#include "planning/spars2_statistics.hpp"

namespace prx 
{ 
    using namespace util;
    namespace packages
    {
        namespace spars
        {

spars2_statistics_t::spars2_statistics_t() 
{
    num_vertices = 0;
    num_edges = 0;
    num_coverage = 0;
    num_connectivity = 0;
    num_interface = 0;
    num_quality = 0;
}

spars2_statistics_t::~spars2_statistics_t() { }

std::string spars2_statistics_t::get_statistics() const
{
    std::stringstream out(std::stringstream::out);
    
    out << statistics_t::get_statistics()  << num_vertices << " , " << num_edges;
    out << " , " << num_coverage << " , " << num_connectivity << " , ";
    out << num_interface << " < " << num_quality;

    return out.str();
}

bool spars2_statistics_t::serialize(std::ofstream& stream) const
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
}
