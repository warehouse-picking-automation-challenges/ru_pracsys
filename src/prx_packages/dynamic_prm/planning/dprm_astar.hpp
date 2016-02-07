/**
 * @file dprm_astar.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_DPRM_ASTAR_HPP
#define PRX_DPRM_ASTAR_HPP

#include "prx/utilities/heuristic_search/astar_search.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"


namespace prx
{
    namespace packages
    {
        namespace dynamic_prm
        {

            class dprm_astar_t : public util::astar_search_t
            {

              public:
                virtual double heuristic(util::directed_vertex_index_t current, util::directed_vertex_index_t goal);
                void setMetric(util::distance_metric_t* m);

              protected:
                util::distance_metric_t* metric;
            };
        }
    }
}



#endif