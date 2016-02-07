/**
 * @file edge.hpp
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
#ifndef PRX_EDGE_HPP
#define	PRX_EDGE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <fstream>

namespace prx
{
    namespace util
    {

        typedef boost::adjacency_list_traits< boost::listS, boost::listS, boost::directedS >::edge_descriptor directed_edge_index_t;
        typedef boost::adjacency_list_traits< boost::listS, boost::listS, boost::undirectedS >::edge_descriptor undirected_edge_index_t;

        /**
         * An abstract edge class for graph operations. Used to abstract out the particular 
         * requirements of motion planners so the same graph can be used for all of them.
         * 
         * @brief <b> An abstract edge class for graph operations. </b>
         */
        class abstract_edge_t
        {

          public:

            abstract_edge_t(){ }

            virtual ~abstract_edge_t(){ }


            /**
             * @brief Node id's for the source and target of this edge.
             */
            int source_vertex, target_vertex;

            /**
             * Outputs relevant information to a stream.
             * @brief Outputs relevant information to a stream.
             * @param output_stream Where to output the information.
             */
            virtual void serialize(std::ofstream& output_stream)
            {
                output_stream << source_vertex << " " << target_vertex;
            }

            /**
             * Inputs relevant information from a stream. Currently does nothing.
             * @brief Inputs relevant information from a stream.
             * @param input_stream Where to get the information.
             */
            virtual void deserialize(std::ifstream& input_stream){
                // Does nothing currently, due to the way prx_graph is set up
            }

        };

    }
}

#endif

