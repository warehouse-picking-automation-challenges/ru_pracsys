/**
 * @file obstacle_aware_astar.cpp
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

#include "planning/modules/obstacle_aware_astar.hpp"
#include "planning/modules/system_name_validity_checker.hpp"
#include "planning/graphs/manipulation_graph.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"
#include "planning/graphs/pebble_graph.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::obstacle_aware_astar_t, prx::plan::astar_module_t)

namespace prx
{
    namespace packages
    {
        using namespace util;
        using namespace plan;

        namespace rearrangement_manipulation
        {

            obstacle_aware_astar_t::obstacle_aware_astar_t()
            {
                max_length = PRX_INFINITY;
                collision_penalty = PRX_INFINITY;
                minimum_conflict = false;
                shortest_path = true;
                system_checker = NULL;
                valid_constraints = NULL;
            }

            void obstacle_aware_astar_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                if( parameters::has_attribute("system_name_validity_checker", reader, template_reader) )
                {
                    system_checker = static_cast<system_name_validity_checker_t*>(parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "system_name_validity_checker", template_reader, "system_name_validity_checker"));
                    validity_checker = system_checker;
                }
                else
                    PRX_WARN_S("Missing system_name_validity_checker attribute for obstacle_aware_astar_t!");

                max_length = parameters::get_attribute_as<double>("max_length", reader, template_reader, PRX_INFINITY);
                collision_penalty = parameters::get_attribute_as<double>("collision_penalty", reader, template_reader, PRX_INFINITY);
                minimum_conflict = parameters::get_attribute_as<bool>("minimum_conflict", reader, template_reader, false);
                shortest_path = parameters::get_attribute_as<bool>("shortest_path", reader, template_reader, true);
            }

            void obstacle_aware_astar_t::link_validity_checker(validity_checker_t* checker)
            {
                if( dynamic_cast<system_name_validity_checker_t*>(checker) == NULL )
                {
                    PRX_WARN_S("Obstacle_aware_A* can only use system_name_validity_checker");
                    return;
                }

                if( validity_checker != NULL )
                    PRX_WARN_S("Obstacle_aware_A* will use the given system_name_validity_checker!");

                validity_checker = checker;
                system_checker = static_cast<system_name_validity_checker_t*>(checker);

            }

            bool obstacle_aware_astar_t::solve(undirected_vertex_index_t start, const std::vector<undirected_vertex_index_t>& goals)
            {
                return solve(start, goals[0]);
            }

            bool obstacle_aware_astar_t::solve(undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                checked_edges.clear();
                double dist = 0;
                path_constraints.clear();
                final_path.clear();
                open_set.clear();

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    graph->get_vertex_as<manipulation_node_t > (v)->shortest_distance = PRX_INFINITY;
                    graph->get_vertex_as<manipulation_node_t > (v)->constraints_size = PRX_INFINITY;
                }

                //                PRX_DEBUG_COLOR("start: " << start << "    goal: " << goal, PRX_TEXT_MAGENTA);
                rearrangement_astar_node_t* nd = new rearrangement_astar_node_t(start, 0, heuristic(start, goal));
                nd->path.push_back(start);
                open_set.insert(nd);
                graph->distances[start] = 0;

                sys_clock_t clock;
                clock.reset();
                while( !open_set.empty() )
                {
                    node = dynamic_cast<rearrangement_astar_node_t*>(open_set.remove_min());
                    if( node->vertex == goal )
                    {
                        foundGoal = goal;
                        final_path = node->path;
                        path_constraints = node->constraints;
                        final_path_distance = node->f;

                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("======================     Final Path     ======================", PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("Size       : " << final_path.size(), PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("constraints: |" << path_constraints.size() << "|   : " << node->print_constraints(), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Cost       : " << final_path_distance, PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Time       : " << clock.measure(), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        return true;
                    }

                    //                    PRX_DEBUG_COLOR("node: " << node->vertex << "    f:" << node->f << "    dist:" << node->distance << "    neighs:" << boost::degree(node->vertex, graph->graph), PRX_TEXT_GREEN);

                    //                    PRX_DEBUG_COLOR("For new node : " << graph->get_vertex_as<undirected_node_t>(node->vertex)->node_id << ")  f: " << node->f << "     dist:" << node->distance  << "    heap_size: " << open_set.size(), PRX_TEXT_CYAN);
                    new_constraints_size = node->constraints.size();
                    new_dist = node->distance;
                    if( examine_vertex(node->vertex) )
                    {

                        foreach(undirected_vertex_index_t v, boost::adjacent_vertices(node->vertex, graph->graph))
                        {
                            //                        PRX_DEBUG_COLOR("Going to examine one neighbor : " << v, PRX_TEXT_CYAN);
                            if( !node->path_has_vertex(v) )
                            {
                                undirected_edge_index_t e = boost::edge(node->vertex, v, graph->graph).first;
                                dist = graph->weights[e] + node->distance;
                                //                            PRX_DEBUG_COLOR("v is fine: dist:" << dist << "    max_len:" << max_length, PRX_TEXT_GREEN);
                                if( dist < max_length )
                                {
                                    if( examine_edge(node->vertex, e, v) )
                                    {
                                        rearrangement_astar_node_t * new_node = new rearrangement_astar_node_t(v);
                                        new_node->merge(node);
                                        new_node->add_constraints(path_constraints);
                                        double penalty = new_node->no_constraints() * collision_penalty;
                                        new_node->set_f(dist, heuristic(v, goal) + penalty);

                                        if( new_node->constraints.size() == 0 )
                                            graph->get_vertex_as<manipulation_node_t > (v)->shortest_distance = dist;
                                        //                                    PRX_DEBUG_COLOR(graph->get_vertex_as<undirected_node_t>(node->vertex)->node_id << "->" << graph->get_vertex_as<undirected_node_t>(v)->node_id << ") New node:   f: " << new_node->f << "     dist:" << new_node->distance  << "    heap_size: " << open_set.size(), PRX_TEXT_MAGENTA);
                                        //                                    if( new_node->no_constraints() != 0 )
                                        //                                        PRX_DEBUG_COLOR("penalty : " << penalty << "   constraints:" << new_node->print_constraints(), PRX_TEXT_MAGENTA);
                                        open_set.insert(new_node);

                                    }
                                }
                            }
                        }
                    }
                    delete node;
                }
                return false;
            }

            void obstacle_aware_astar_t::extract_path(undirected_vertex_index_t start, undirected_vertex_index_t goal, std::deque<undirected_vertex_index_t>& vertices)
            {
                vertices = final_path;
            }

            void obstacle_aware_astar_t::get_path_constraints(std::set<unsigned>& constraints)
            {

                constraints.insert(path_constraints.begin(), path_constraints.end());
            }

            bool obstacle_aware_astar_t::examine_vertex(undirected_vertex_index_t vertex)
            {
                manipulation_node_t* v_node = graph->get_vertex_as<manipulation_node_t > (vertex);

                //For the case that we arrived on the same node with no constraints but longest distance then we don't add this point.
                if( v_node->constraints_size == 0 && new_constraints_size == 0 && new_dist > v_node->shortest_distance )
                    return false;


                if( shortest_path )
                {
                    if( v_node->constraints_size == 0 )
                    {
                        if( new_constraints_size != 0 )
                        {
                            PRX_ASSERT(false);
                            if( new_dist >= v_node->shortest_distance )
                            {
                                return false;
                            }
                        }
                        else
                        {
                            v_node->shortest_distance = new_dist;
                        }
                    }
                }
                else
                {
                    PRX_ASSERT(minimum_conflict);
                    if( v_node->constraints_size == 0 && new_constraints_size > 0 )
                        return false;
                    //At this point both nodes have 0 constraints and the new node has better distance than the existed node (because of the first check in this function)
                    v_node->shortest_distance = new_dist;
                }

                if( new_constraints_size < v_node->constraints_size )
                {
                    v_node->shortest_distance = new_dist;
                    v_node->constraints_size = new_constraints_size;
                }
                return true;
            }

            bool obstacle_aware_astar_t::examine_edge(undirected_vertex_index_t start_index, undirected_edge_index_t e, undirected_vertex_index_t end_index)
            {
                path_constraints.clear();

                //check if the edge is in the blocked edges.
                if( astar_module_t::examine_edge(start_index, e, end_index) )
                {
                    new_constraints_size = 0;
                    new_dist = node->distance + graph->weights[e];

                    if( solve_mode == PRX_REUSE_EDGE_INFO )
                    {
                        manipulation_edge_t* edge = graph->get_edge_as<manipulation_edge_t > (e);

                        //First check if the edge has any constraint (valid or not))
                        if( !edge->is_valid(valid_constraints) )
                        {
                            if( !minimum_conflict )
                                return false;

                            edge->get_valid_constraints(path_constraints,valid_constraints);
                            new_constraints_size = node->constraints.size() + path_constraints.size();
                        }
                    }
                    //if the mode is PRX_NO_EDGE_INFO we only need to check the new vertex.
                    return examine_vertex(end_index);

                }
                return false;
            }

            double obstacle_aware_astar_t::heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal)
            {
                return metric->distance_function(graph->operator[](current)->point, graph->operator[](goal)->point);
            }

            double obstacle_aware_astar_t::heuristic(util::undirected_vertex_index_t current, const std::vector< util::undirected_vertex_index_t >& goals)
            {
                PRX_FATAL_S("Why are you using this heuristic function?");
                return 0;
            }

            void obstacle_aware_astar_t::link_valid_constraints(std::set<unsigned>* constraints)
            {
                valid_constraints = constraints;
            }

            void obstacle_aware_astar_t::set_minimum_conflict(bool flag)
            {
                minimum_conflict = flag;
            }

            void obstacle_aware_astar_t::set_shortest_path_flag(bool flag)
            {
                shortest_path = flag;
            }

            void obstacle_aware_astar_t::set_max_length(double length)
            {
                max_length = length;
                if( minimum_conflict )
                    collision_penalty = 2 * max_length;
            }

            void obstacle_aware_astar_t::set_collision_penalty(double penalty)
            {
                collision_penalty = penalty;
            }

            double obstacle_aware_astar_t::get_collision_penalty() const
            {
                return collision_penalty;
            }

            double obstacle_aware_astar_t::get_path_distance()
            {
                return final_path_distance;
            }

            void obstacle_aware_astar_t::name_to_pose_constraints(std::vector<std::string> constraints, std::set<unsigned>& poses_constraints)
            {

                foreach(std::string name, constraints)
                {
                    PRX_DEBUG_COLOR("Constraint: " << name, PRX_TEXT_RED);
                    poses_constraints.insert(0);
                }
            }

            int obstacle_aware_astar_t::find_checked_edge(util::undirected_edge_index_t e)
            {
                for( unsigned i = 0; i < checked_edges.size(); ++i )
                    if( checked_edges[i].e == e )
                        return i;
                return -1;
            }
        }
    }
}
