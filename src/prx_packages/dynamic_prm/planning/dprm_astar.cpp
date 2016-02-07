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

#include "dprm_astar.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_graph.hpp"



namespace prx
{
    namespace packages
    {
        namespace dynamic_prm
        {

            double dprm_astar_t::heuristic(util::directed_vertex_index_t current, util::directed_vertex_index_t goal)
            {
                util::directed_node_t *c, *g;
                c = graph->get_vertex_as<util::directed_node_t > (current);
                g = graph->get_vertex_as<util::directed_node_t > (goal);
                return metric->distance_function(c->point, g->point);
            }

            void dprm_astar_t::setMetric(util::distance_metric_t* m)
            {
                metric = m;
            }
        }
    }
}
