/**
 * @file obstacle_aware_astar.hpp
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
#pragma once

#ifndef PRX_OBSTACLE_AWARE_ASTAR_HPP
#define	PRX_OBSTACLE_AWARE_ASTAR_HPP


#include "prx/planning/modules/heuristic_search/astar_module.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/heuristic_search/astar_open_set.hpp"

#include <set>
#include <boost/range/adaptor/map.hpp>

#define PRX_OA_ASTAR_PENALTY 10000

/**
 * Writing this assuming the following variables are (somehow) available:
 *
 * directed_graph_t graph (With node and edge types from arrange_graph.hpp)
 * system_name_validity_checker_t naming_validity_checker
 * std::vector< directed_vertex_index_t > OA_predecessor_map
 */

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            class system_name_validity_checker_t;

            class rearrangement_astar_node_t : public util::astar_node_t
            {

              public:

                rearrangement_astar_node_t(){ }

                rearrangement_astar_node_t(util::undirected_vertex_index_t vertex) : astar_node_t(vertex, 0)
                {
                    this->vertex = vertex;
                }

                rearrangement_astar_node_t(util::undirected_vertex_index_t vertex, double distance, double h) : astar_node_t(vertex, distance + h)
                {
                    this->distance = distance;
                }

                rearrangement_astar_node_t(const rearrangement_astar_node_t & n) : astar_node_t(n.vertex, n.f)
                {
                    constraints = n.constraints;
                    path = n.path;
                }

                virtual ~rearrangement_astar_node_t(){ }

                virtual const rearrangement_astar_node_t& operator=(const rearrangement_astar_node_t & other)
                {
                    vertex = other.vertex;
                    f = other.f;
                    constraints = other.constraints;
                    path = other.path;
                    return (*this);
                }

                virtual bool operator<(const rearrangement_astar_node_t & n) const
                {
                    //                    PRX_DEBUG_COLOR("Yes checking: c:" << constraints.size() << "/" << n.constraints.size() << "   f:" << f << "/" << n.f, PRX_TEXT_BROWN);
                    //                    if( constraints.size() == n.constraints.size() )
                    return f < n.f;

                    //                    return constraints.size() < n.constraints.size();
                }

                virtual bool operator<(const astar_node_t & n) const
                {
                    const rearrangement_astar_node_t* node = dynamic_cast<const rearrangement_astar_node_t*>(&n);
                    //                    PRX_DEBUG_COLOR("Yes2 checking: c:" << constraints.size() << "/" << node->constraints.size() << "   f:" << f << "/" << node->f, PRX_TEXT_BROWN);
                    //                    if( constraints.size() == node->constraints.size() )
                    return f < node->f;

                    //                    return constraints.size() < node->constraints.size();
                }

                virtual operator unsigned() const
                {
                    return *(unsigned int*)(&vertex);
                }

                virtual void set_f(double distance, double h)
                {
                    this->distance = distance;
                    f = distance + h;
                }

                virtual void add_a_constraints(unsigned constraint)
                {
                    constraints.insert(constraint);
                }

                virtual void add_constraints(const std::set<unsigned>& new_constraints)
                {
                    constraints.insert(new_constraints.begin(), new_constraints.end());
                }

                virtual bool path_has_vertex(util::undirected_vertex_index_t v)
                {
                    return std::find(path.begin(), path.end(), v) != path.end();
                }

                virtual void merge(const rearrangement_astar_node_t* node)
                {
                    add_constraints(node->constraints);
                    path = node->path;
                    path.push_back(vertex);
                }

                virtual unsigned no_constraints() const
                {
                    return constraints.size();
                }

                std::string print_constraints() const
                {
                    std::stringstream output(std::stringstream::out);

                    foreach(unsigned i, constraints)
                    {
                        output << i << " , ";
                    }
                    return output.str();
                }

                std::set<unsigned> constraints;
                std::deque<util::undirected_vertex_index_t > path;
                double distance;
            };

            class edge_status_t
            {

              public:

                edge_status_t()
                {
                    valid = false;
                }

                edge_status_t(util::undirected_edge_index_t e, bool valid, std::set<unsigned>& constraints)
                {
                    this->e = e;
                    this->valid = valid;
                    this->constraints = constraints;
                }
                //                

                virtual ~edge_status_t()
                {
                    constraints.clear();
                }

                util::undirected_edge_index_t e;
                bool valid;
                std::set<unsigned> constraints;
            };

            class obstacle_aware_astar_t : public plan::astar_module_t
            {

              public:
                obstacle_aware_astar_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void link_validity_checker(plan::validity_checker_t* checker);

                virtual bool solve(util::undirected_vertex_index_t start, const std::vector<util::undirected_vertex_index_t>& goals);
                virtual bool solve(util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal);
                virtual void extract_path(util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal, std::deque<util::undirected_vertex_index_t>& vertices);
                virtual void get_path_constraints(std::set<unsigned>& constraints);

                virtual bool examine_vertex(util::undirected_vertex_index_t vertex);                
                virtual bool examine_edge(util::undirected_vertex_index_t start_index, util::undirected_edge_index_t e, util::undirected_vertex_index_t end_index);
                virtual double heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal);
                virtual double heuristic(util::undirected_vertex_index_t current, const std::vector< util::undirected_vertex_index_t >& goals);
                
                virtual void link_valid_constraints(std::set<unsigned>* constraints);
                virtual void set_minimum_conflict(bool flag);
                virtual void set_shortest_path_flag(bool flag);
                virtual void set_max_length(double length);
                virtual void set_collision_penalty(double penalty);
                virtual double get_collision_penalty() const;
                
                virtual double get_path_distance();

              private:
                void name_to_pose_constraints(std::vector<std::string> constraints, std::set<unsigned>& poses_constraints);
                int find_checked_edge(util::undirected_edge_index_t e);

                double max_length;
                double collision_penalty;
                bool minimum_conflict;
                bool shortest_path;
                std::deque<util::undirected_vertex_index_t> final_path;                
                std::set<unsigned> path_constraints;
                double final_path_distance;
                system_name_validity_checker_t* system_checker;
                std::vector<edge_status_t> checked_edges;
                std::set<unsigned>* valid_constraints;
                
                //helping variable
                rearrangement_astar_node_t* node;
                unsigned new_constraints_size;
                double new_dist;
            };
        }
    }
}

#endif
