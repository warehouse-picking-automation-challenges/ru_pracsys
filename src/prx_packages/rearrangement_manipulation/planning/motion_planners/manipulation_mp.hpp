/**
 * @file manipulation_mp.hpp
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

#ifndef PRX_MANIPULATION_MP_HPP
#define	PRX_MANIPULATION_MP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star.hpp"

#include "planning/graphs/manipulation_graph.hpp"

namespace prx
{
    namespace plan
    {
        class specification_t;
    }
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            class manipulation_mp_specification_t;
            class manipulator_mp_query_t;
            class obstacle_aware_astar_t;

            /**
             * @anchor manipulation_mp_t

             * @author Athanasios Krontiris
             */
            class manipulation_mp_t : public plan::prm_star_t
            {

              public:
                manipulation_mp_t();
                virtual ~manipulation_mp_t();

                /**
                 * @copydoc prm_star_t::init(const util::parameter_reader_t*,const util::parameter_reader_t*) 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc prm_star_t::reset() 
                 */
                virtual void reset();

                /**
                 * @copydoc prm_star_t::link_specification(specification_t*) 
                 */
                void link_specification(plan::specification_t* new_spec);
                
                /**
                 * @copydoc prm_star_t::link_query()
                 */
                virtual void link_query(plan::query_t* new_query);

                /** 
                 * @copydoc prm_star_t::setup() 
                 */
                virtual void setup();

                /**
                 * @copydoc prm_star_t::execute()
                 */
                virtual bool execute();

                /** 
                 * @copydoc prm_star_t::resolve_query() 
                 */
                virtual void resolve_query();

                /**
                 * @copydoc prm_star_t::succeeded() const 
                 */
                virtual bool succeeded() const;

                /**
                 * @copydoc prm_star_t::serialize() const 
                 */
                virtual bool serialize();

                /**
                 * @copydoc prm_star_t::deserialize() const 
                 */
                virtual bool deserialize();

                virtual bool deserialize_graph(std::string graph_file);

              protected:

                /**
                 * Adds a new node in the graph and trying to connect it with the 
                 * existing graph. 
                 * 
                 * @brief Add a new node to the graph.
                 *
                 * @param n_state The new state that I want to add in the graph.
                 * 
                 * @return The index for the node in the boost::graph.
                 */
                virtual std::pair<bool, util::undirected_vertex_index_t> add_node(const util::space_point_t* n_state);

                /**
                 * @brief Tries to link the node v with the neighbor nodes in the vector neighbors.
                 * 
                 * @param v Its the index of an existing node on the graph, that we want to connect on the graph.
                 * @param neighbors The nodes that we will try to connect with the node v.
                 */
                virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);

                virtual void inform_graph(const std::vector< std::pair<unsigned, sim::state_t*> >* poses);

                virtual void inform_edges(const std::vector<util::undirected_edge_index_t>& edges, const std::vector< std::pair<unsigned, sim::state_t*> >* poses);

                /** @brief A flag indicating whether or not this planner is able to deserialize a graph from a file. */
                bool graph_deserialize_flag;
                /** @brief The file from which the planner will deserialize a graph that it needs to inform.*/
                std::string graph_deserialization_file;

              private:
                manipulation_mp_specification_t* specs;
                manipulator_mp_query_t* in_query;
                obstacle_aware_astar_t* oa_astar;
                std::vector<util::undirected_edge_index_t> new_edges;
                std::string prx_output_dir;
                std::string prx_input_dir;
                int lazy_runs;

            };
        }
    }
}

#endif
