/**
 * @file spars2_graph.hpp
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

#ifndef PRX_SPARS2_GRAPH_HPP
#define	PRX_SPARS2_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

namespace prx
{
    namespace packages
    {
        namespace spars
        {

        struct point_record_t
        {
            bool valid;

            util::undirected_vertex_index_t v;
            util::undirected_vertex_index_t vp;

            util::space_point_t* q;
            util::space_point_t* s;
            util::space_point_t* qp;
            util::space_point_t* sp;
        };


        /**
         * @anchor spars2_node_t
         *
         * SPARS2 nodes must remember the state space point they represent.  These nodes
         * additionally carry extra information for SPARS2 and its variants.
         *
         * @brief <b> Node class used by the SPARS2 planning structure. </b>
         *
         * @author Andrew Dobson
         */
        class spars2_node_t : public plan::prm_star_node_t
        {
          public:
            spars2_node_t(){ }
            ~spars2_node_t(){ }

            /**
             * @brief Initialize the node with required space and point information.
             *
             * @param space The state space this node exists in.
             * @param new_point The point in the space where this node exsists.
             */
            void init_node(const util::space_t* space, const util::space_point_t* new_point)
            {
                plan::prm_star_node_t::init_node(space, new_point);
            }

            /**
             * @brief Output node information to a stream.
             *
             * @param output_stream The stream to which to serialize the node information.
             * @param point_space The space where the space point lies. TODO : Do we really need this?
             */
            void serialize(std::ofstream& output_stream, const util::space_t* point_space)
            {
                util::undirected_node_t::serialize(output_stream, point_space);
            }

            /**
             * @brief Read in node information from a stream.
             *
             * @param input_stream The stream from which to read the node information.
             * @param point_space The space where the space point lies. TODO : Do we really need this?
             */
            void deserialize(std::ifstream& input_stream, const util::space_t* point_space)
            {
                util::undirected_node_t::deserialize(input_stream, point_space);
            }

            point_record_t* get_point_record(util::undirected_vertex_index_t vp, util::undirected_vertex_index_t vpp)
            {
                util::undirected_vertex_index_t neighbor_one;
                util::undirected_vertex_index_t neighbor_two;

                if(vp < vpp)
                {
                    neighbor_one = vp;
                    neighbor_two = vpp;
                }
                else
                {
                    neighbor_one = vpp;
                    neighbor_two = vp;
                }

                //If we have such a record, return it.
                for(unsigned i=0; i<point_data.size(); ++i)
                {
                    if(neighbor_one == point_data[i].v && neighbor_two == point_data[i].vp)
                    {
                        return &point_data[i];
                    }
                }

                //Otherwise, no such record exists.  Create one, and return it.
                point_data.resize(point_data.size()+1);
                point_record_t* last_point = &(point_data[point_data.size()-1]);
                last_point->valid = false;
                last_point->v = neighbor_one;
                last_point->vp = neighbor_two;
                return last_point;
            }

          protected:
            //The bookkeeping data needed for dense path checks
            std::vector<point_record_t> point_data;
        };

        /**
         * @anchor spars2_edge_t
         *
         * Edges in the SPARS2 planning structure which must hold information such as weights,
         * controls used for generation, and an identifier.
         *
         * @brief <b> Edge class used in the SPARS2 planning structure. </b>
         *
         * @author Andrew Dobson
         */
        class spars2_edge_t : public plan::prm_star_edge_t
        {
          public:
            ~spars2_edge_t(){ }

        };

        }
    }
}

#endif


