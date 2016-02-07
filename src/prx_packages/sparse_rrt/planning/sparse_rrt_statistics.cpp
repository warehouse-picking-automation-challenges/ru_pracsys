/**
 * @file sparse_rrt_statistics.cpp
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

#include "sparse_rrt_statistics.hpp"

namespace prx
{
    using namespace util;
    using namespace plan;
    namespace packages
    {
        namespace sparse_rrt
        {
            sparse_rrt_statistics_t::sparse_rrt_statistics_t() : rrt_statistics_t()
            {
            }

            sparse_rrt_statistics_t::~sparse_rrt_statistics_t() { }


            std::string sparse_rrt_statistics_t::get_statistics() const
            {
                std::stringstream out(std::stringstream::out);

                out << statistics_t::get_statistics()  << ", " << num_vertices <<",  "<<solution_quality  << ", " << failure_count << ", "<<average_cost<<", "<<best_near<<", "<<drain<<", "<<avg_lifespan<<", "<<children_of_root<<", "<<avg_distance_from_root<<std::endl ;

                return out.str();
            }
        }
    }
}
