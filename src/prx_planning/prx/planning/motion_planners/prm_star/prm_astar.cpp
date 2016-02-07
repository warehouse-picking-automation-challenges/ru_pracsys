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


#include "prx/planning/motion_planners/prm_star/prm_astar.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::prm_astar_t, prx::plan::astar_module_t)

namespace prx
{
    using namespace util;

    namespace plan
    {

        prm_astar_t::prm_astar_t()
        {
            validity_checker = NULL;
            metric = NULL;
            local_planner = NULL;
            state_space = NULL;
            control_space = NULL;
            solve_mode = PRX_NO_EDGE_INFO;


            blocked_edges.clear();
        }

        prm_astar_t::~prm_astar_t()
        {
            edge_path.clear();
            edge_plan.clear();
        }

        bool prm_astar_t::examine_vertex(undirected_vertex_index_t end_index)
        {
            undirected_vertex_index_t start_index = graph->predecessors[end_index];
            if(start_index==NULL)
            {
                return true;
            }
            undirected_edge_index_t edge = boost::edge(start_index, end_index, graph->graph).first;

            //            if( start_index == end_index )
            //            {
            //                return true;
            //            }
            //            examine_edge(start_index, edge, end_index);

            if( std::find(blocked_edges.begin(), blocked_edges.end(), edge) != blocked_edges.end() )
            {
                //PRX_PRINT("There is a blocked vertex...", PRX_TEXT_BROWN);
                return false;
            }

            if( solve_mode == PRX_NO_EDGE_INFO || start_index == end_index )
            {
                return true;
            }
            else if( solve_mode == PRX_REPROPAGATE_EDGE_INFO )
            {
                prm_star_node_t* u = graph->get_vertex_as<prm_star_node_t > (start_index);
                prm_star_node_t* v = graph->get_vertex_as<prm_star_node_t > (end_index);

                if( local_planner != NULL )
                {
                    edge_plan.clear();
                    edge_path.clear();
                    local_planner->steer(u->point, v->point, edge_plan, edge_path);

                    if( validity_checker != NULL )
                    {
                        return validity_checker->is_valid(edge_path);
                    }
                    else
                    {
                        PRX_ERROR_S("Validity checker not specified in prm_astar.");
                    }

                }
                else
                {
                    PRX_ERROR_S("Local Planner not specified in prm_astar.");
                }
            }
            else if( solve_mode == PRX_REUSE_EDGE_INFO )
            {
                prm_star_node_t* u = graph->get_vertex_as<prm_star_node_t > (start_index);
                prm_star_node_t* v = graph->get_vertex_as<prm_star_node_t > (end_index);

                if( validity_checker != NULL )
                {
                    prm_star_edge_t* e = graph->get_edge_as<prm_star_edge_t > (edge);

                    if( e->path.size() > 0 )
                    {
                        return validity_checker->is_valid(e->path);
                    }
                    else if( e->plan.size() > 0 )
                    {
                        if( local_planner != NULL )
                        {
                            edge_plan.clear();
                            edge_path.clear();
                            local_planner->propagate(u->point, e->plan, edge_path);

                            return validity_checker->is_valid(edge_path);

                        }
                        else
                        {
                            PRX_WARN_S("Missing path and local planner information on edges in PRX_REUSE_EDGE_INFO mode.");
                            return true;
                        }
                    }
                    else
                    {
                        PRX_WARN_S("Missing path and plan information on edges in PRX_REUSE_EDGE_INFO mode.");
                        return true;
                    }

                }
                else
                {
                    PRX_ERROR_S("Validity checker not specified in prm_astar.");
                }

            }
            else
            {
                PRX_ERROR_S("Invalid mode of prm_astar.");
            }

            return false;
        }

        bool prm_astar_t::examine_edge(undirected_vertex_index_t start_index, undirected_edge_index_t edge, undirected_vertex_index_t end_index) //TODO: Figure out where this is called.  Fix this.
        {
            if( std::find(blocked_edges.begin(), blocked_edges.end(), edge) != blocked_edges.end() )
            {
                //PRX_PRINT("There is a blocked edge...", PRX_TEXT_BROWN);
                return false;
            }

            if( solve_mode == PRX_NO_EDGE_INFO )
            {
                return true;
            }
            else if( solve_mode == PRX_REPROPAGATE_EDGE_INFO )
            {
                prm_star_node_t* u = graph->get_vertex_as<prm_star_node_t > (start_index);
                prm_star_node_t* v = graph->get_vertex_as<prm_star_node_t > (end_index);

                if( local_planner != NULL )
                {
                    edge_plan.clear();
                    edge_path.clear();
                    local_planner->steer(u->point, v->point, edge_plan, edge_path);

                    if( validity_checker != NULL )
                    {
                        return validity_checker->is_valid(edge_path);
                    }
                    else
                    {
                        PRX_ERROR_S("Validity checker not specified in prm_astar.");
                    }

                }
                else
                {
                    PRX_ERROR_S("Local Planner not specified in prm_astar.");
                }
            }
            else if( solve_mode == PRX_REUSE_EDGE_INFO )
            {
                prm_star_node_t* u = graph->get_vertex_as<prm_star_node_t > (start_index);
                prm_star_node_t* v = graph->get_vertex_as<prm_star_node_t > (end_index);

                if( validity_checker != NULL )
                {

                    prm_star_edge_t* e = graph->get_edge_as<prm_star_edge_t > (edge);

                    if( e->path.size() > 0 )
                    {
                        return validity_checker->is_valid(e->path);
                    }
                    else if( e->plan.size() > 0 )
                    {
                        if( local_planner != NULL )
                        {
                            edge_plan.clear();
                            edge_path.clear();
                            local_planner->propagate(u->point, e->plan, edge_path);

                            return validity_checker->is_valid(edge_path);

                        }
                        else
                        {
                            PRX_WARN_S("Missing path and local planner information on edges in PRX_REUSE_EDGE_INFO mode.");
                            return true;
                        }
                    }
                    else
                    {
                        PRX_WARN_S("Missing path and plan information on edges in PRX_REUSE_EDGE_INFO mode.");
                        return true;
                    }
                }
                else
                {
                    PRX_ERROR_S("Validity checker not specified in prm_astar.");
                }
            }
            else
            {
                PRX_ERROR_S("Invalid mode of prm_astar.");
            }
        }

        double prm_astar_t::heuristic(undirected_vertex_index_t current, undirected_vertex_index_t goal)
        {
            undirected_node_t *s, *t;
            s = graph->get_vertex_as<undirected_node_t > (current);
            t = graph->get_vertex_as<undirected_node_t > (goal);
            return metric->distance_function(s->point, t->point);
        }

        double prm_astar_t::heuristic(undirected_vertex_index_t current, const std::vector<undirected_vertex_index_t>& goals)
        {
            double minH, h;
            size_t i;

            minH = heuristic(current, goals[0]);

            undirected_node_t* s = graph->get_vertex_as<undirected_node_t > (current);
            undirected_node_t* t;
            for( i = 1; i < goals.size(); ++i )
            {
                t = graph->get_vertex_as<undirected_node_t > (goals[i]);
                h = metric->distance_function(s->point, t->point);
                if( h < minH )
                    minH = h;
            }

            return minH;
        }
    }
}
