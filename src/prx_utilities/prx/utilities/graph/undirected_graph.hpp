/**
 * @file undirected_graph.hpp 
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

#ifndef PRACSYS_UNDIRECTED_GRAPH_HPP
#define PRACSYS_UNDIRECTED_GRAPH_HPP

#include <iostream>
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"

#include "prx/utilities/graph/abstract_node.hpp"
#include "prx/utilities/graph/undirected_node.hpp"
#include "prx/utilities/graph/abstract_edge.hpp"
#include "prx/utilities/graph/undirected_edge.hpp"

#include <boost/graph/graph_traits.hpp> 
#include <boost/config.hpp>

namespace prx
{
    namespace util
    {

        struct undirected_vertex_wrapper
        {

            undirected_node_t* node;
        };

        struct undirected_edge_wrapper
        {

            undirected_edge_t* link;
        };

        struct undirected_component_type_t
        {

            typedef boost::vertex_property_tag kind;
        };

        struct undirected_edge_component_type_t
        {

            enum
            {

                num = -1
            };
            typedef boost::edge_property_tag kind;
        };

        typedef boost::adjacency_list< boost::listS, boost::listS, boost::undirectedS,
        boost::property<undirected_component_type_t, int,
        boost::property<boost::vertex_index_t, int,
        boost::property<boost::vertex_distance_t, double,
        boost::property<boost::vertex_predecessor_t, undirected_vertex_index_t,
        boost::property<boost::vertex_color_t, boost::default_color_type, undirected_vertex_wrapper> > > > >,
        boost::property<boost::edge_weight_t, double,
        boost::property<undirected_edge_component_type_t, int, undirected_edge_wrapper > > > undirected_graph_type;

        /**
         * A wrapper around the Boost undirected graph.
         * @brief <b> A wrapper around the Boost undirected graph. </b> 
         */
        class undirected_graph_t
        {

          public:

            undirected_graph_t()
            {
                space = NULL;
                weights = boost::get(boost::edge_weight, graph);
                predecessors = boost::get(boost::vertex_predecessor, graph);
                distances = boost::get(boost::vertex_distance, graph);
                indices = boost::get(boost::vertex_index, graph);
                colors = boost::get(boost::vertex_color, graph);
                components = boost::get(undirected_component_type_t(), graph);
                edge_components = boost::get(undirected_edge_component_type_t(), graph);
                count = 0;
            }

            /**
             * @brief Constructor
             * @param inspace The state space to use in this graph.
             */
            undirected_graph_t(space_t* inspace)
            {
                undirected_graph_t();
                space = inspace;
                dont_care_about_indices = false;
            }

            ~undirected_graph_t()
            {
                if( space != NULL )
                {

                    foreach(undirected_edge_index_t ed, boost::edges(graph))
                    {
                        remove_edge(ed);
                    }

                    foreach(undirected_vertex_index_t nd, boost::vertices(graph))
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
            undirected_vertex_index_t add_vertex()
            {
                undirected_vertex_index_t ret = boost::add_vertex(graph);
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
            node_type* get_vertex_as(undirected_vertex_index_t i)
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
            node_type* get_vertex_as(undirected_vertex_index_t i) const
            {
                return (node_type*)graph[i].node;
            }

            /**
             * Removes a vertex from the graph.
             * @brief Removes a vertex from the graph.
             * @param v The index of the vertex to remove.
             */
            void remove_vertex(undirected_vertex_index_t v)
            {
                if( space != NULL )
                    if( graph[v].node->point != NULL )
                        space->free_point(graph[v].node->point);
                graph[v].node->point = NULL;
                graph[v].node->index = NULL;
                if( !dont_care_about_indices )
                {
                    int num = indices[v];

                    foreach(undirected_vertex_index_t u, boost::vertices(graph))
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

                foreach(undirected_edge_index_t e, boost::edges(graph))
                {
                    delete graph[e].link;
                }

                foreach(undirected_vertex_index_t v, boost::vertices(graph))
                {
                    if( space != NULL )
                        if( graph[v].node->point != NULL )
                            space->free_point(graph[v].node->point);
                    graph[v].node->point = NULL;
                    graph[v].node->index = NULL;
                    if( !dont_care_about_indices )
                    {
                        int num = indices[v];

                        foreach(undirected_vertex_index_t u, boost::vertices(graph))
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
            void clear_vertex(undirected_vertex_index_t v)
            {

                foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph))
                {
                    undirected_edge_index_t e = boost::edge(v, u, graph).first;
                    delete graph[e].link;
                }
                boost::clear_vertex(v, graph);
            }

            /**
             * Performs a clear, then removes the vertex from the graph.
             * @brief Performs a clear, then removes the vertex from the graph.
             * @param v The index of the vertex to clear and remove.
             */
            void clear_and_remove_vertex(undirected_vertex_index_t v)
            {

                foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph))
                {
                    undirected_edge_index_t e = boost::edge(v, u, graph).first;
                    bool exists;
                    boost::tie(e, exists) = boost::edge(v, u, graph);
                    if( exists && graph[e].link != NULL )
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
            undirected_edge_index_t add_edge(undirected_vertex_index_t from, undirected_vertex_index_t to, double weight = 0.0001)
            {
                undirected_edge_index_t ret = (boost::add_edge(from, to, graph)).first;
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
            edge_type* get_edge_as(undirected_edge_index_t i)
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
            edge_type* get_edge_as(undirected_edge_index_t i) const
            {
                return (edge_type*)graph[i].link;
            }

            /**
             * Removes and edge from the graph
             * @brief Removes and edge from the graph
             * @param e The index of the edge to remove.
             */
            void remove_edge(undirected_edge_index_t e)
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
            void remove_edge(undirected_vertex_index_t v1, undirected_vertex_index_t v2)
            {
                undirected_edge_index_t e;
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
            void set_weight(undirected_edge_index_t e, double weight)
            {
                weights[e] = weight;
            }

            /**
             * Gets the weight of an edge in the graph.
             * @brief Gets the weight of an edge in the graph.
             * @param e Edge index to retrieve.
             */
            double get_weight(undirected_edge_index_t e)
            {
                return weights[e];
            }

            /**
             * Gets an edge.
             * @brief Gets an edge.
             * @param e The edge index to retrieve.
             * @return The edge.
             */
            undirected_edge_t* operator[](undirected_edge_index_t e) const
            {
                return graph[e].link;
            }

            /**
             * Gets a vertex.
             * @brief Gets a vertex.
             * @param v The vertex index to retrieve.
             * @return The vertex.
             */
            undirected_node_t* operator[](undirected_vertex_index_t v) const
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
                PRX_ERROR_S("Inside graph serialize");
                output_stream << boost::num_vertices(graph) << std::endl;

                foreach(undirected_vertex_index_t nd, boost::vertices(graph))
                {
                    graph[nd].node->serialize(output_stream, point_space);
                    output_stream << std::endl;
                }
                output_stream << boost::num_edges(graph) << std::endl;

                foreach(undirected_edge_index_t ed, boost::edges(graph))
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
                input_stream.open(filename.c_str());
                if( input_stream.good() )
                {
                    int num_edges, num_vertices;
                    input_stream >> num_vertices;
                    hash_t<int, undirected_vertex_index_t> node_map;
                    for( int i = 0; i < num_vertices; i++ )
                    {
                        
                        undirected_vertex_index_t new_vertex = add_vertex<node_type > ();
                        graph[new_vertex].node->index = new_vertex;
                        graph[new_vertex].node->point = point_space->alloc_point();
                        graph[new_vertex].node->deserialize(input_stream, point_space);
                        node_map[graph[new_vertex].node->node_id] = new_vertex;
                    }
                    input_stream >> num_edges;
                    //        PRX_LOG_ERROR("NUM EDGES: %i", num_edges);
                    for( int i = 0; i < num_edges; i++ )
                    {
                        int from, to;
                        input_stream >> from >> to;
                        undirected_edge_index_t new_edge = add_edge<edge_type > (node_map[from], node_map[to]);
                        graph[new_edge].link->deserialize(input_stream);
                    }

                    return true;
                }
                else
                {
                    return false;
                }

            }
            //
            //            
            //            template<class node_type, class edge_type>
            //            void deserialize(std::ifstream& input_stream, const space_t* point_space)
            //            {
            //                PRX_ERROR_S("Inside graph deserialize!");
            //                int num_edges, num_vertices;
            //                input_stream >> num_vertices;
            //                hash_t<int, undirected_vertex_index_t> node_map;
            //                for( int i = 0; i < num_vertices; i++ )
            //                {
            //                    undirected_vertex_index_t new_vertex = add_vertex<node_type > ();
            //                    graph[new_vertex].node->index = new_vertex;
            //                    graph[new_vertex].node->point = point_space->alloc_point();
            //                    graph[new_vertex].node->deserialize(input_stream, point_space);
            //                    node_map[graph[new_vertex].node->node_id] = new_vertex;
            //                }
            //                input_stream >> num_edges;
            //                for( int i = 0; i < num_edges; i++ )
            //                {
            //                    int from, to;
            //                    input_stream >> from >> to;
            //                    std::cout << "From vertex: " << from << ", to vertex: " << to << std::endl;
            //                    undirected_edge_index_t new_edge = add_edge<edge_type > (node_map[from], node_map[to]);
            //                    graph[new_edge].link->deserialize(input_stream);
            //                }
            //
            //            }

            /**
             * @brief The edge weights
             */
            boost::property_map<undirected_graph_type, boost::edge_weight_t>::type weights;

            /**
             * @brief Predecessor map. For use in graph searches
             */
            boost::property_map<undirected_graph_type, boost::vertex_predecessor_t>::type predecessors;

            /**
             * @brief Stores path costs in graph searches.
             */
            boost::property_map<undirected_graph_type, boost::vertex_distance_t>::type distances;

            /**
             * @brief Map to integer indices in graph searches
             */
            boost::property_map<undirected_graph_type, boost::vertex_index_t>::type indices;

            /**
             * @brief Color map for graph searches.
             */
            boost::property_map<undirected_graph_type, boost::vertex_color_t>::type colors;

            /**
             * @brief Components map for connected components
             */
            boost::property_map<undirected_graph_type, undirected_component_type_t>::type components;

            /**
             * @brief Edge components map for connected components.
             */
            boost::property_map<undirected_graph_type, undirected_edge_component_type_t>::type edge_components;

            /**
             * @brief Flag for optimizing clearing the graph. Requires no maintenence of indices when clearing.
             */
            bool dont_care_about_indices;
            
            /**
             * @brief The Boost graph.
             */
            undirected_graph_type graph;

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
