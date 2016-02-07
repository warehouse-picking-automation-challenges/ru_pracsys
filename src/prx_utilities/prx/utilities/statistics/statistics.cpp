/**
 * @file statistics.cpp
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

#include "prx/utilities/statistics/statistics.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        statistics_t::statistics_t()
        {
            time = 0;
            steps = 0;
        }
        
        statistics_t::~statistics_t() { }
        
        std::string statistics_t::get_statistics() const
        {
            std::stringstream out(std::stringstream::out);
            
            out << time << " , " << steps;            
            
            return out.str();
        }
        
        bool statistics_t::serialize(std::ofstream& stream) const
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
