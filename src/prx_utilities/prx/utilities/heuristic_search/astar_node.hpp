/**
 * @file astar_search.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_ASTAR_NODE_HPP
#define	PRX_ASTAR_NODE_HPP

#include "prx/utilities/graph/undirected_graph.hpp"

namespace prx
{
    namespace util
    {

        /**
         * A structure to represent a node on the A* open set. The main purpose 
         * is to provide a place to store vertices' f-values and some lightweight 
         * comparison and hashing functions.
         * 
         * @brief <b> A structure to represent a node on the A* open set. </b>
         * 
         * @Author: Justin Cardoza
         */
        class astar_node_t
        {

          public:
            undirected_vertex_index_t vertex;
            double f;

            astar_node_t()
            {
                vertex = NULL;
                f = 0;
            }

            astar_node_t(undirected_vertex_index_t vertex, double f) : vertex(vertex), f(f){ }

            astar_node_t(const astar_node_t & n) : vertex(n.vertex), f(n.f){ }

            virtual ~astar_node_t(){ }

            virtual const astar_node_t& operator=(const astar_node_t& other)
            {
                vertex = other.vertex;
                f = other.f;
                return (*this);
            }

            virtual bool operator<(const astar_node_t & n) const
            {
                if( vertex == n.vertex )
                    return false;
                if( f == n.f )
                    return vertex < n.vertex;
                return f < n.f;
            }

            virtual operator unsigned() const
            {
                return *(unsigned int*)(&vertex);
            }
        };
    }
}
#endif	

