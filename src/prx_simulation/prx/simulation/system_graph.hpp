/**
 * @file system_graph.hpp
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

#ifndef PRX_SYSTEM_GRAPH_HPP
#define	PRX_SYSTEM_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/system_ptr.hpp"
#include "prx/utilities/boost/hash.hpp"

#include <boost/graph/adjacency_list.hpp>   

namespace prx
{
    namespace sim
    {

        class plant_t;

        /**
         * It is a tree data structure that maintains all the systems using as root the system that the \ref system_graph_t called. 
         * In order to build the system graph you need to call \ref system_graph_t::update_system_graph() on the system that you need
         * to get the systems' subtree of this system. After you have the system graph you can request for information such as paths, 
         * subsystems name, systems etc.
         * 
         * @brief <b> A system graph for the representation of the system tree.</b> 
         * 
         * @authors Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield
         * 
         */
        class system_graph_t
        {

          public:

            typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>::vertex_descriptor directed_vertex_t;
            typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>::edge_descriptor directed_edge_t;

            system_graph_t();
            virtual ~system_graph_t();

            /**
             * Clears the system graph.
             * 
             * @brief Clears the system graph.
             */
            void clear();

            /**
             * Adds a new node to the graph. Each node represent a different system.
             * 
             * @brief Adds a new node to the graph.
             * 
             * @param system_path The slash-delimited path indicating the location of the system added in the system_graph. 
             * @param is_a_plant Shows if the system that we are going to add is plant or something else
             * @return A \ref system_graph_t::directed_vertex_t which represent the location of the new system in the graph.
             */
            directed_vertex_t add_node(const std::string& system_path, bool is_a_plant);

            /**
             * Adds a new edge in the graph between the vertex a and vertex b. Each edge represent that the source vertex
             * contains the target vertex as subsystem in the system tree.
             * 
             * @brief Adds a new edge between vertices \c vertex_a and \c vertex_b.
             * 
             * @param vertex_a The parent of the two systems already existed in the system graph.     
             * @param vertex_b The new subsystem of the system in the vertex_a.
             * @param system_name The name of the new subsystem.
             * @param system The pointer to the subsystem. 
             * @return A \ref system_graph_t::directed_edge_t which represent the edge between the vertex a and b in the graph.
             */
            directed_edge_t add_edge(const directed_vertex_t& vertex_a, const directed_vertex_t& vertex_b, const std::string& system_name, system_ptr_t system);

            /**
             * Finds all the plants in the simulation tree and returns their pointers in the argument \c plants.
             * 
             * @brief Finds all the plants in the simulation tree.
             * 
             * @param plants The vector of plant pointers to fill with the pointer of the plants.
             */
            void get_plants(std::vector<plant_t*>& plants);

            /**
             * Finds all the plants in the simulation tree and returns their names (local identifiers)
             * This function is useful for the collision checking operations.
             * 
             * @brief Finds all the plants' names.     
             * 
             * @param plant_names A vector to fill with names from all the plants.
             */
            void get_plant_names(std::vector<std::string>& plant_names);

            void get_plant_vertices(std::vector<directed_vertex_t>& plant_vertices);

            /**
             * Finds all the plants in the simulation tree and returns their full paths.
             * Useful for accessing the plants in the simulation tree.
             * 
             * @brief Finds all the plant paths.
             * 
             * @param plant_paths A vector to be filled with all plant paths.
             */
            void get_plant_paths(std::vector<std::string>& plant_paths);

            /**
             * Constructs a hash map that maps plant names with plant pointers.
             * 
             * @brief Constructs a hash map that maps plant names with plant pointers.
             * 
             * @param plant_hash The hash to populate the map between plant names and plant pointers.
             */
            void get_name_plant_hash(util::hash_t<std::string, plant_t*>& plant_hash);

            /**
             * Constructs a hash map that maps plant paths with plant pointers.
             * 
             * @brief Constructs a hash map that maps plant paths with plant pointers.
             * 
             * @param plant_hash The hash to populate the map between plant paths and plant pointers.
             */
            void get_path_plant_hash(util::hash_t<std::string, plant_t*>& plant_hash);

            /**
             * For all systems in the simulation tree, a list of pointers is created.
             * 
             * @brief Returns a list of all the systems.
             * 
             * @param systems The list of systems that will be filled.
             */
            void get_systems(std::vector<system_ptr_t>& systems);

            /**
             * Populates a list of all the system names in the simulation tree.
             * 
             * @brief Populates a list with all the system names.
             *      
             * @param system_names The list that is populated.
             */
            void get_system_names(std::vector<std::string>& system_names);

            /**
             * Populates a list of all the system paths in the simulation tree. Useful for lookup later.
             * 
             * @brief Populates a list of all the system paths in the simulation tree.
             * 
             * @param system_paths The list of system paths that is populated.
             */
            void get_system_paths(std::vector<std::string>& system_paths);

            /**
             * Creates a hash map of system names to system pointers.
             * 
             * @brief Creates a hash map of system names to system pointers.
             * 
             * @param system_hash The hash that maps system names with system pointers
             */
            void get_name_system_hash(util::hash_t<std::string, system_ptr_t>& system_hash);

            /**
             * Creates a hash map of system paths to system pointers.
             * 
             * @brief Creates a hash map of system paths to system pointers.
             * 
             * @param system_hash The hash that maps system paths to system pointers.
             */
            void get_path_system_hash(util::hash_t<std::string, system_ptr_t>& system_hash);

            /**
             * A container struct for all the information that we need to maintain for the nodes in the graph
             */
            struct node_properties_t
            {

                /** The name of the system*/
                std::string name;
                /** The slash-delimited path that indicates the location of the system*/
                std::string path;
                /** The system pointer*/
                system_ptr_t system;
                /** Stores the vertex of the parent system*/
                directed_vertex_t parent_vertex;
                /** If that system is a plant or not*/
                bool is_plant;

                node_properties_t(){ }

                /**
                 * A constructor that initializes all the variables for the \ref node_properties_t.
                 * @param system_name The name of the system.
                 * @param system_path The slash-delimited path that indicates the location of the system.
                 * @param sys The system pointer.
                 * @param is_a_plant If the system is a plant or not.
                 */
                node_properties_t(const std::string& system_name, const std::string& system_path, system_ptr_t sys, bool is_a_plant = false)
                {
                    name = system_name;
                    path = system_path;
                    system = sys;
                    is_plant = is_a_plant;
                }
            };

            /** 
             * An informative struct for all the information that we need to maintain for the edges in the graph.
             */
            struct edge_properties_t
            {

                /** The name of the parent system that we are attached to.*/
                std::string name;

                edge_properties_t(){ }

                /**
                 * A constructor for initializing the variables of the struct \ref edge_properties_t.
                 * @param sys_name The system name that is the parent of the subsystem. 
                 */
                edge_properties_t(const std::string & sys_name)
                {
                    name = sys_name;
                }
            };


            typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, node_properties_t, edge_properties_t> directed_graph_t;
            /** Is the system_graph that will maintain all the information about the systems in the simulation */
            directed_graph_t system_graph;

        };

    }
}

#endif

