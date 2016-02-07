/**
 * @file sst_statistics.cpp
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

#include "prx/planning/motion_planners/sst/sst_statistics.hpp"

namespace prx
{
    using namespace util;
    namespace plan
    {
        sst_statistics_t::sst_statistics_t() : rrt_statistics_t()
        {
        }

        sst_statistics_t::~sst_statistics_t() { }


        std::string sst_statistics_t::get_statistics() const
        {
            std::stringstream out(std::stringstream::out);

            out << statistics_t::get_statistics()  << ", " << num_vertices <<",  "<<solution_quality  << ", " << failure_count << ", "<<average_cost<<", "<<first_solution<<", "<<best_near<<", "<<drain<<", "<<removed_by_validity<<", "<<removed_by_drain<<", "<<removed_by_bnb<<", "<<removed_by_losing<<std::endl ;

            return out.str();
        }
    }
}
