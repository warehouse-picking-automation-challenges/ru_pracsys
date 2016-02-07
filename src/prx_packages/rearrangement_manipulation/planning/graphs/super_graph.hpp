/**
 * @file super_graph.hpp
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

#ifndef PRX_SUPER_GRAPH_HPP
#define	PRX_SUPER_GRAPH_HPP


#include <vector>
#include <set>

#include "planning/graphs/pebble_graph.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/simulation/state.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             *
             */
            class hnode_data_t
            {

              public:
                hnode_data_t();
                hnode_data_t(const hnode_data_t& other);
                hnode_data_t(util::undirected_graph_t* pebble_graph);
                ~hnode_data_t();

                void link_pebble_graph(util::undirected_graph_t* pebble_graph);
                void link_pebble_graph(util::undirected_graph_t* pebble_graph, const std::vector< unsigned >& insigma);

                bool is_useful();

                std::vector< std::vector< unsigned > > generate_signatures(unsigned in_k) const;

                static std::vector< std::vector< unsigned > > generate_signatures(unsigned in_k, const std::vector< unsigned >& max_cc)
                {
                    std::vector< std::vector< unsigned > > ret;
                    std::vector< std::vector< unsigned > > hold;

                    //Error handling: If someone gives us an empty set of cc's.
                    if( max_cc.size() == 0 )
                        return ret;

                    //If we still have connected components to try to fill
                    if( max_cc.size() > 1 )
                    {
                        unsigned max_nr = PRX_MINIMUM(in_k, max_cc.back());
                        for( unsigned i = 0; i <= max_nr; ++i )
                        {
                            hold = generate_signatures(in_k - i, std::vector< unsigned >(max_cc.begin(), max_cc.end() - 1));
                            if( !hold.empty() )
                            {

                                foreach(std::vector< unsigned > u, hold)
                                {
                                    u.push_back(i);
                                    ret.resize(ret.size() + 1);
                                    ret.back() = u;
                                }
                            }
                        }
                    }
                    else //We have only one component to fill.
                    {
                        //If we're still supposed to assign some pebbles.
                        if( in_k > 0 )
                        {
                            //Now, if there's enough room to put the remaining pebbles
                            if( in_k <= max_cc[0] )
                            {
                                //Assign that many pebbles
                                ret.resize(1);
                                ret[0].push_back(in_k);
                                return ret;
                            }
                            //Otherwise, we can't make the assignment, return empty.
                            return ret;
                        }
                        else //Otherwise, we have succesfully assigned all of the pebbles
                        {
                            //And, we didn't have to do any work here, so we report 0 new assignments.
                            ret.resize(1);
                            ret[0].push_back(0);
                            return ret;
                        }
                    }

                    return ret;
                }

                static std::vector< std::vector< unsigned > > generate_arrangements(unsigned nr_poses, unsigned in_k)
                {
                    std::vector< unsigned > poses;
                    for( unsigned i = 0; i < nr_poses; ++i )
                        poses.push_back(1);
                    return generate_signatures(in_k, poses);
                }
                void generate_valid_arrangement(std::vector< unsigned >& arrangement) const;
                void update_signature(const std::vector<unsigned>& positions);
                void replace_signature(const std::vector<unsigned>& new_signature);

                const hnode_data_t& deep_copy(const hnode_data_t& other);
                const hnode_data_t& operator=(const hnode_data_t& other);
                bool operator!=(const hnode_data_t& other);
                bool operator==(const hnode_data_t& other);
                bool is_equal(const hnode_data_t* other);
                unsigned same_poses(std::set<unsigned> poses);
                bool have_the_same_poses(std::set<unsigned> poses);

                bool is_data_ok(std::string str) const;

                std::string print_signature() const;
                std::string print_cc_data() const;
                std::string print_positions() const;
                void print(std::string additional_string = std::string()) const;

                //Put function here to go from vertex_index to position_index
                unsigned get_position_index(util::undirected_vertex_index_t v);
                util::undirected_vertex_index_t get_vertex_index(int pi) const;
                util::undirected_vertex_index_t get_vertex_index(util::undirected_graph_t* graph, int position) const;

                void copy_pebble_graph(util::undirected_graph_t& destination, const util::undirected_graph_t& source);

                void set(util::undirected_graph_t& ingraph);
                void set(util::undirected_graph_t& ingraph, const std::vector< unsigned >& insigma);

                //Graph rearrangement_manipulation junk...
                util::undirected_graph_t* p_graph;
                std::vector< std::set< unsigned > > cc_data;
                std::vector< unsigned > signature;
                std::vector< unsigned > cc_sizes;
                std::set< unsigned > positions;
            };

            /**
             *
             */
            class super_node_t : public util::undirected_node_t
            {

              public:
                super_node_t();
                ~super_node_t();

                hnode_data_t* data;
            };

            /**
             *
             */
            class super_edge_t : public plan::motion_planner_edge_t
            {

              public:
                void init_as_motion(util::undirected_vertex_index_t v, const std::set< unsigned >& in_poses, const std::pair< unsigned, unsigned >& in_motion, const sim::plan_t& new_plan);
                void init_as_switch(const std::set< unsigned >& in_poses);
                unsigned get_source_position(util::undirected_vertex_index_t v);
                unsigned get_the_other_position(unsigned pose);
                void get_plan(util::undirected_vertex_index_t v, sim::plan_t& computed_plan);

                //If the edge has a motion, then poses represents a set of poses which must be cleared,
                //  if it does not, the poses represents a specific arrangement which must be reached.
                util::undirected_vertex_index_t s_vertex; //Keeps the source vertex of the super graph that the edge is initialized 
                bool has_motion;
                std::pair< unsigned, unsigned > motion;
                std::set< unsigned > poses;
                std::set< unsigned > full_constraints;
                unsigned reaching_point;
                unsigned retracting_point;
            };

            struct super_graph_goal_t
            {

                /** @brief Vertex index of the goal node. */
                util::undirected_vertex_index_t v_goal;

                /**
                 * @brief Function used by boost::astar to determine if the goal has been expanded.
                 *
                 * @param v The node to make the goal.
                 */
                super_graph_goal_t(util::undirected_vertex_index_t v)
                {
                    v_goal = v;
                }
            };
        }
    }
}

#endif	






