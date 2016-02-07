/**
 * @file prm_astar.hpp 
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

#ifndef PRX_PRM_ASTAR_HPP
#define PRX_PRM_ASTAR_HPP

#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_graph.hpp"

namespace prx
{
    namespace plan
    {

        class prm_astar_t : public astar_module_t
        {

          public:

            prm_astar_t();
            ~prm_astar_t();

            virtual bool examine_edge(util::undirected_vertex_index_t start_index, util::undirected_edge_index_t edge, util::undirected_vertex_index_t end_index);
            virtual bool examine_vertex(util::undirected_vertex_index_t v);
            virtual double heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal);
            virtual double heuristic(util::undirected_vertex_index_t current, const std::vector<util::undirected_vertex_index_t>& goals);
        };

    }
}



#endif