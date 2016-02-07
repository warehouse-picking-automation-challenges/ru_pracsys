/**
 * @file astar_search.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/modules/heuristic_search/astar_module.hpp"

#include "prx/utilities/definitions/defs.hpp"


#include <algorithm>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/subgraph.hpp>



namespace prx
{
    using namespace util;
    namespace plan
    {

        astar_module_t::~astar_module_t() { }

        void astar_module_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader) { }

        void astar_module_t::restart()
        {
            blocked_edges.clear();
            edge_path.clear();
            edge_plan.clear();
        }

        void astar_module_t::link_graph(util::undirected_graph_t *g)
        {
            astar_search_t::link_graph(g);
            restart();
        }

        void astar_module_t::link_spaces(const space_t* state_sp, const space_t* control_sp)
        {
            state_space = state_sp;
            control_space = control_sp;
            edge_path.link_space(state_sp);
            edge_plan.link_control_space(control_sp);
        }

        void astar_module_t::link_distance_metric(distance_metric_t* m)
        {
            metric = m;
        }

        void astar_module_t::link_validity_checker(validity_checker_t* checker)
        {
            validity_checker = checker;
        }

        void astar_module_t::link_local_planner(local_planner_t* loc_planner)
        {
            local_planner = loc_planner;
        }

        void astar_module_t::link_modules(validity_checker_t* checker, local_planner_t* loc_planner)
        {
            link_validity_checker(checker);
            link_local_planner(loc_planner);
        }

        void astar_module_t::setup_astar_module(undirected_graph_t *g, const space_t* state_sp, const space_t* control_sp, validity_checker_t* checker, local_planner_t* loc_planner)
        {
            link_graph(g);
            link_spaces(state_sp, control_sp);
            link_validity_checker(checker);
            link_local_planner(loc_planner);
        }

        void astar_module_t::set_astar_mode(astar_mode mode)
        {
            solve_mode = mode;
        }

        void astar_module_t::block_edge(undirected_edge_index_t e)
        {
            blocked_edges.push_back(e);
            //            PRX_DEBUG_COLOR("blocked edges size: " << blocked_edges.size() << " / " << boost::num_vertices(graph->graph),PRX_TEXT_MAGENTA);
        }

        bool astar_module_t::examine_edge(undirected_vertex_index_t u, undirected_edge_index_t edge, undirected_vertex_index_t v)
        {
            if( std::find(blocked_edges.begin(), blocked_edges.end(), edge) != blocked_edges.end() )
                return false;

            return true;
        }

        pluginlib::ClassLoader<astar_module_t> astar_module_t::loader("prx_planning", "prx::plan::astar_module_t");

        pluginlib::ClassLoader<astar_module_t>& astar_module_t::get_loader()
        {
            return loader;
        }
    }
}
