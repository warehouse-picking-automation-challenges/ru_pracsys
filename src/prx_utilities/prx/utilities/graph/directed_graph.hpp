/**
 * @file directed_graph.hpp 
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

#ifndef PRACSYS_GRAPH_HPP
#define PRACSYS_GRAPH_HPP

#include <iostream>
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/graph/directed_node.hpp"
#include "prx/utilities/graph/directed_edge.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <boost/graph/graph_traits.hpp>
#include <boost/config.hpp>
#include <ros/ros.h>

namespace prx
{
    namespace util
    {

        /**
         * @brief <b>Boost graph abstraction<b>
         */

        struct vertex_wrapper
        {

            directed_node_t* node;
        };

        struct edge_wrapper
        {

            directed_edge_t* link;
        };

        struct component_type_t
        {

            typedef boost::vertex_property_tag kind;
        };

        struct edge_component_type_t
        {

            enum
            {

                num = -1
            };
            typedef boost::edge_property_tag kind;
        };

        typedef boost::adjacency_list< boost::listS, boost::listS, boost::directedS,
        boost::property<component_type_t, int,
        boost::property<boost::vertex_index_t, int,
        boost::property<boost::vertex_distance_t, double,
        boost::property<boost::vertex_predecessor_t, directed_vertex_index_t,
        boost::property<boost::vertex_color_t, boost::default_color_type, vertex_wrapper> > > > >,
        boost::property<boost::edge_weight_t, double,
        boost::property<edge_component_type_t, int, edge_wrapper > > > directed_graph_type;

        /**
         * A wrapper around the Boost directed graph.
         * @brief <b> A wrapper around the Boost directed graph. </b>
         */
        class directed_graph_t

        {

          public:

            directed_graph_t()
            {
                space = NULL;
                weights = boost::get(boost::edge_weight, graph);
                predecessors = boost::get(boost::vertex_predecessor, graph);
                distances = boost::get(boost::vertex_distance, graph);
                indices = boost::get(boost::vertex_index, graph);
                colors = boost::get(boost::vertex_color, graph);
                components = boost::get(component_type_t(), graph);
                edge_components = boost::get(edge_component_type_t(), graph);
                count = 0;
            }

            /**
             * @brief Constructor
             * @param inspace The state space to use in this graph.
             */
            directed_graph_t(space_t* inspace)
            {
                space = inspace;
                dont_care_about_indices = false;
            }

            ~directed_graph_t()
            {
                PRX_DEBUG_S("Directed graph destructor!");
                if( space != NULL )
                {
                    PRX_DEBUG_S("not null son");

                    foreach(directed_edge_index_t ed, boost::edges(graph))
                    {
                        remove_edge(ed);
                    }

                    foreach(directed_vertex_index_t nd, boost::vertices(graph))
                    {
                        remove_vertex(nd);
                    }
                    space = NULL;
                }
            }

            /**
             * Adds a templated vertex type into the graph.
             * @brief Adds a templated vertex type into the graph.
             * @return The vertex index corresponding to the newly added vertex.
             */
            template<class node_type>
            directed_vertex_index_t add_vertex()
            {
                directed_vertex_index_t ret = boost::add_vertex(graph);
                graph[ret].node = new node_type;
                graph[ret].node->index = ret;
                graph[ret].node->point = NULL;
                graph[ret].node->node_id = count;
                indices[ret] = count;
                colors[ret] = boost::color_traits<boost::default_color_type > ().white();
                count++;
                return ret;
            }

            /**
             * Returns the node in the graph as a certain templated type.
             * @brief Returns the node in the graph as a certain templated type.
             * @param i The vertex index of the node to be returned.
             * @return The casted node.
             */
            template<class node_type>
            node_type* get_vertex_as(directed_vertex_index_t i)
            {
                return (node_type*)graph[i].node;
            }

            /**
             * Returns the node in the graph as a certain templated type. (const version)
             * @brief Returns the node in the graph as a certain templated type. (const version)
             * @param i The vertex index of the node to be returned.
             * @return The casted node.
             */
            template<class node_type>
            node_type* get_vertex_as(directed_vertex_index_t i) const
            {
                return (node_type*)graph[i].node;
            }

            /**
             * Removes a vertex from the graph.
             * @brief Removes a vertex from the graph.
             * @param v The index of the vertex to remove.
             */
            void remove_vertex(directed_vertex_index_t v)
            {
                if( space != NULL )
                    if( graph[v].node->point != NULL )
                        space->free_point(graph[v].node->point);
                graph[v].node->point = NULL;
                graph[v].node->index = NULL;
                if( !dont_care_about_indices )
                {
                    int num = indices[v];

                    foreach(directed_vertex_index_t u, boost::vertices(graph))
                    {
                        if( indices[u] > num )
                        {
                            indices[u]--;
                            graph[u].node->node_id--;
                        }
                    }
                }
                delete graph[v].node;
                count--;
                boost::remove_vertex(v, graph);
            }
            
            /**
             * Clears the graph.
             * @brief Clears the graph.
             */
            void clear()
            {
                dont_care_about_indices = true;

                foreach(directed_edge_index_t e, boost::edges(graph))
                {
                    delete graph[e].link;
                }

                foreach(directed_vertex_index_t v, boost::vertices(graph))
                {
                    if( space != NULL )
                        if( graph[v].node->point != NULL )
                            space->free_point(graph[v].node->point);
                    graph[v].node->point = NULL;
                    graph[v].node->index = NULL;
                    if( !dont_care_about_indices )
                    {
                        int num = indices[v];

                        foreach(directed_vertex_index_t u, boost::vertices(graph))
                        {
                            if( indices[u] > num )
                            {
                                indices[u]--;
                                graph[u].node->node_id--;
                            }
                        }
                    }
                    delete graph[v].node;
                }
                dont_care_about_indices = false;
                graph.clear();
            }

            /**
             * Wrapper for the Boost function clear_vertex
             * @brief Wrapper for the Boost function clear_vertex
             * @param v The index of the vertex index to clear.
             */
            void clear_vertex(directed_vertex_index_t v)
            {

                foreach(directed_vertex_index_t u, boost::adjacent_vertices(v, graph))
                {
                    directed_edge_index_t e;
                    bool exists;
                    boost::tie(e, exists) = boost::edge(v, u, graph);
                    if( exists )
                        delete graph[e].link;
                    boost::tie(e, exists) = boost::edge(u, v, graph);
                    if( exists )
                        delete graph[e].link;
                }
                boost::clear_vertex(v, graph);
            }

            /**
             * Performs a clear, then removes the vertex from the graph.
             * @brief Performs a clear, then removes the vertex from the graph.
             * @param v The index of the vertex to clear and remove.
             */
            void clear_and_remove_vertex(directed_vertex_index_t v)
            {

                foreach(directed_vertex_index_t u, boost::adjacent_vertices(v, graph))
                {
                    directed_edge_index_t e;
                    bool exists;
                    boost::tie(e, exists) = boost::edge(v, u, graph);
                    if( exists )
                        delete graph[e].link;
                    boost::tie(e, exists) = boost::edge(u, v, graph);
                    if( exists )
                        delete graph[e].link;
                }
                boost::clear_vertex(v, graph);
                remove_vertex(v);
            }

            /**
             * Add an edge onto the graph.
             * @brief Add an edge onto the graph.
             * @param from Vertex index of the source of this edge.
             * @param to Vertex index of the target of this edge
             * @param weight The edge weight.
             * @return An index to this edge in the graph.
             */
            template<class edge_type>
            directed_edge_index_t add_edge(directed_vertex_index_t from, directed_vertex_index_t to, double weight = 0.0001)
            {
                directed_edge_index_t ret = (boost::add_edge(from, to, graph)).first;
                weights[ret] = weight;
                graph[ret].link = new edge_type;
                graph[ret].link->index = ret;
                graph[ret].link->source_vertex = graph[from].node->node_id;
                graph[ret].link->target_vertex = graph[to].node->node_id;
                return ret;
            }

            /**
             * Gets the edge as a templated type.
             * @brief Gets the edge as a templated type.
             * @param i The edge index to retrieve.
             * @return The casted edge.
             */
            template<class edge_type>
            edge_type* get_edge_as(directed_edge_index_t i)
            {
                return (edge_type*)graph[i].link;
            }

            /**
             * Gets the edge as a templated type. (const version)
             * @brief Gets the edge as a templated type. (const version)
             * @param i The edge index to retrieve.
             * @return The casted edge.
             */
            template<class edge_type>
            edge_type* get_edge_as(directed_edge_index_t i) const
            {
                return (edge_type*)graph[i].link;
            }

            /**
             * Removes and edge from the graph
             * @brief Removes and edge from the graph
             * @param e The index of the edge to remove.
             */
            void remove_edge(directed_edge_index_t e)
            {
                delete graph[e].link;
                boost::remove_edge(e, graph);
            }

            /**
             * Removes and edge from the graph
             * @brief Removes and edge from the graph
             * @param v1 The source of the edge to remove
             * @param v2 The target of the edge to remove.
             */
            void remove_edge(directed_vertex_index_t v1, directed_vertex_index_t v2)
            {
                directed_edge_index_t e;
                bool valid;
                boost::tie(e, valid) = boost::edge(v1, v2, graph);
                if( valid )
                {
                    remove_edge(e);
                }
            }

            /**
             * Links the state space to the graph.
             * @brief Links the state space to the graph.
             * @param new_space The space to link.
             */
            void link_space(space_t* new_space)
            {
                if( boost::num_vertices(graph) == 0 || space == NULL )
                {
                    space = new_space;
                }
            }

            /**
             * Sets the weight of an edge in the graph.
             * @brief Sets the weight of an edge in the graph.
             * @param e Edge index to modify.
             * @param weight New edge weight.
             */
            void set_weight(directed_edge_index_t e, double weight)
            {
                weights[e] = weight;
            }

            /**
             * Gets the weight of an edge in the graph.
             * @brief Gets the weight of an edge in the graph.
             * @param e Edge index to retrieve.
             */
            double get_weight(directed_edge_index_t e)
            {
                return weights[e];
            }

            /**
             * Gets an edge.
             * @brief Gets an edge.
             * @param e The edge index to retrieve.
             * @return The edge.
             */
            directed_edge_t* operator[](directed_edge_index_t e) const
            {
                return graph[e].link;
            }

            /**
             * Gets a vertex.
             * @brief Gets a vertex.
             * @param v The vertex index to retrieve.
             * @return The vertex.
             */
            directed_node_t* operator[](directed_vertex_index_t v) const
            {
                return graph[v].node;
            }

            /**
             * Serializes the graph to an output stream
             * @brief Serializes the graph to an output stream
             * @param output_stream The stream to output to.
             * @param point_space The space for state points (passed to the nodes.)
             */
            void serialize(std::ofstream& output_stream, const space_t* point_space)
            {
                //        PRX_LOG_ERROR("Populate the indices map");

                PRX_INFO_S("Serialize graph ...");
                output_stream << boost::num_vertices(graph) << std::endl;

                foreach(directed_vertex_index_t nd, boost::vertices(graph))
                {
                    graph[nd].node->serialize(output_stream, point_space);
                    output_stream << std::endl;
                }
                output_stream << boost::num_edges(graph) << std::endl;

                foreach(directed_edge_index_t ed, boost::edges(graph))
                {
                    graph[ed].link->serialize(output_stream);
                    output_stream << std::endl;
                }


            }

            /**
             * Deserializes the graph from an input stream
             * @brief Deserializes the graph from an input stream
             * @param input_stream The stream to input from.
             * @param point_space The space for state points (passed to the nodes.)
             */
            template<class node_type, class edge_type>
            bool deserialize(const std::string& filename, std::ifstream& input_stream, const space_t* point_space)
            {
                if( !input_stream.is_open() )
                    input_stream.open(filename.c_str());
                if( input_stream.good() )
                {
                    //                    PRX_ERROR_S("Inside graph deserialize!");
                    int num_edges, num_vertices;
                    input_stream >> num_vertices;
                    //                    PRX_DEBUG_COLOR("Number of nodes to read: " << num_vertices, PRX_TEXT_BROWN );

                    for( int i = 0; i < num_vertices; i++ )
                    {
                        directed_vertex_index_t new_vertex = add_vertex<node_type > ();
                        graph[new_vertex].node->index = new_vertex;
                        graph[new_vertex].node->point = point_space->alloc_point();
                        graph[new_vertex].node->deserialize(input_stream, point_space);
                        //                        PRX_DEBUG_COLOR("Node id: " << graph[new_vertex].node->node_id, PRX_TEXT_LIGHTGRAY );
                        id_to_node_map[graph[new_vertex].node->node_id] = new_vertex;
                    }
                    input_stream >> num_edges;
                    PRX_DEBUG_COLOR("Number of edges to read: " << num_edges, PRX_TEXT_BROWN);
                    //                    PRX_WARN_S ("Before edge deserialize");
                    for( int i = 0; i < num_edges; i++ )
                    {
                        int from, to;
                        input_stream >> from >> to;
                        //                        PRX_DEBUG_COLOR("Reading edge from " << from << " to " << to, PRX_TEXT_LIGHTGRAY );
                        double dist = point_space->distance(graph[id_to_node_map[from]].node->point, graph[id_to_node_map[to]].node->point);
                        directed_edge_index_t new_edge = add_edge<edge_type > (id_to_node_map[from], id_to_node_map[to], dist);
                        graph[new_edge].link->deserialize(input_stream);
                    }

                    return true;
                }
                else
                {
                    return false;
                }

            }

            /**
             * @brief The edge weights
             */
            boost::property_map<directed_graph_type, boost::edge_weight_t>::type weights;
            /**
             * @brief Predecessor map. For use in graph searches
             */
            boost::property_map<directed_graph_type, boost::vertex_predecessor_t>::type predecessors;
            /**
             * @brief Stores path costs in graph searches.
             */
            boost::property_map<directed_graph_type, boost::vertex_distance_t>::type distances;
            /**
             * @brief Map to integer indices in graph searches
             */
            boost::property_map<directed_graph_type, boost::vertex_index_t>::type indices;
            /**
             * @brief Color map for graph searches.
             */
            boost::property_map<directed_graph_type, boost::vertex_color_t>::type colors;
            /**
             * @brief Components map for connected components
             */
            boost::property_map<directed_graph_type, component_type_t>::type components;
            /**
             * @brief Edge components map for connected components.
             */
            boost::property_map<directed_graph_type, edge_component_type_t>::type edge_components;

            /**
             * @brief The Boost graph.
             */
            directed_graph_type graph;

            /**
             * @brief Flag for optimizing clearing the graph. Requires no maintenence of indices when clearing.
             */
            bool dont_care_about_indices;

            /** @brief Maps an integer node index to an actual node. Typically used for deserialization */
            hash_t<int, directed_vertex_index_t> id_to_node_map;

          protected:
            /**
             * @brief Number of nodes in the graph.
             */
            int count;



          private:
            /**
             * @brief The state space that nodes are using.
             */
            space_t* space;
        };



    }
}

#endif
