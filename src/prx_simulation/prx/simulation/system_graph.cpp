/**
 * @file system_graph.cpp 
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


#include "prx/simulation/system_graph.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/systems/plants/plant.hpp"


#include <boost/graph/depth_first_search.hpp>

namespace prx
{
    using namespace util;

    namespace sim
    {

        /**
         * This visitor will only report the paths to the plants in the simulation tree.
         * 
         * @brief <b> This visitor will only report the paths to the plants in the simulation tree.</b>
         * 
         * @param plant_paths The vector that will be populated after the visitor is done.
         */
        class dfs_plant_path_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_plant_path_visitor_t(std::vector<std::string>* plant_paths)
            {
                paths = plant_paths;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                PRX_DEBUG_S(" I visit : " << g[u].name << "  with path : " << g[u].path);
                if( g[u].is_plant )
                {
                    PRX_DEBUG_S("is plant  : " << g[u].name);
                    paths->push_back(g[u].path);
                }
            }

            std::vector<std::string>* paths;

        };

        /**
         * This visitor will only report the names of the plants.
         * 
         * @brief <b> This visitor will only report the names of the plants. </b>
         * 
         * @param plant_names The vector that will be populated after the visitor is done.
         */
        class dfs_plant_name_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_plant_name_visitor_t(std::vector<std::string>* plant_names)
            {
                names = plant_names;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                PRX_DEBUG_S(" I visit : " << g[u].name << "  with path : " << g[u].path);
                if( g[u].is_plant )
                {
                    PRX_DEBUG_S("is plant  : " << g[u].name);
                    names->push_back(g[u].name);
                }
            }

            std::vector<std::string>* names;

        };

        /**
         * This visitor creates a hash from plant names to plant pointers.
         * 
         * @brief <b> This visitor creates a hash from plant names to plant pointers. </b>
         *  
         * @param plant_hash The hash to populate
         */
        class dfs_plant_by_name_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_plant_by_name_visitor_t(hash_t<std::string, plant_t*>* plant_hash)
            {
                plants = plant_hash;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                if( g[u].is_plant )
                    ( *plants )[g[u].name] = static_cast<plant_t*>(g[u].system.get());
            }

            hash_t<std::string, plant_t*>* plants;
        };

        /**
         * This visitor creates a hash from plant paths to plant pointers.
         * 
         * @brief <b> This visitor creates a hash from plant paths to plant pointers. </b>
         *  
         * @param plant_hash The hash to populate
         */
        class dfs_plant_by_path_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_plant_by_path_visitor_t(hash_t<std::string, plant_t*>* plant_hash)
            {
                systems = plant_hash;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                if( g[u].is_plant )
                {
                    PRX_DEBUG_S("Found plant at: " << g[u].path);
                    (*systems)[g[u].path] = static_cast<plant_t*>(g[u].system.get());
                }
            }

            hash_t<std::string, plant_t*>* systems;
        };

        /**
         * This visitor will report the plant pointers.
         * 
         * @brief <b> This visitor will report the plant pointers. </b>
         *  
         * @param plant_pointers The vector that will be populated after the visitor is done.
         */
        class dfs_plant_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_plant_visitor_t(std::vector<plant_t*>* plant_pointers)
            {
                plants = plant_pointers;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                PRX_DEBUG_S(" I visit : " << g[u].name << "  with path : " << g[u].path);
                if( g[u].is_plant )
                    plants->push_back(static_cast<plant_t*>(g[u].system.get()));
            }

            std::vector<plant_t*>* plants;

        };

        /**
         * This visitor will report the plant vertices.
         * 
         * @brief <b> This visitor will report the plant vertices. </b>
         *  
         * @param plant_pointers The vector that will be populated after the visitor is done.
         */
        class dfs_plant_vertex_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_plant_vertex_visitor_t(std::vector<system_graph_t::directed_vertex_t>* plant_vertices)
            {
                vertices = plant_vertices;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                PRX_DEBUG_S(" I visit : " << g[u].name << "  with path : " << g[u].path);
                if( g[u].is_plant )
                    vertices->push_back(u);
            }

            std::vector<system_graph_t::directed_vertex_t>* vertices;

        };

        /**
         * This visitor will only report the paths to the systems in the simulation tree.
         * 
         * @brief <b> This visitor will only report the paths to the systems in the simulation tree. </b>
         *  
         * @param system_paths The vector that will be populated after the visitor is done.
         */
        class dfs_system_path_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_system_path_visitor_t(std::vector<std::string>* system_paths)
            {
                paths = system_paths;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                if( g[u].system.get() != NULL )
                    paths->push_back(g[u].path);
            }

            std::vector<std::string>* paths;

        };

        /**
         * This visitor will only report the names of the systems.
         * 
         * @brief <b> This visitor will only report the names of the systems. </b>
         *  
         * @param system_names The vector that will be populated after the visitor is done.
         */
        class dfs_system_name_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_system_name_visitor_t(std::vector<std::string>* system_names)
            {
                names = system_names;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                if( g[u].system.get() != NULL )
                    names->push_back(g[u].name);
            }

            std::vector<std::string>* names;

        };

        /**
         * This visitor creates a hash from system names to system pointers.
         * 
         * @brief <b> This visitor creates a hash from system names to system pointers. </b>
         *  
         * @param system_hash The hash to populate
         */
        class dfs_system_by_name_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_system_by_name_visitor_t(hash_t<std::string, system_ptr_t>* system_hash)
            {
                systems = system_hash;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                if( g[u].system.get() != NULL )
                    ( *systems )[g[u].name] = g[u].system;
            }

            hash_t<std::string, system_ptr_t>* systems;
        };

        /**
         * This visitor creates a hash from plant paths to plant pointers.
         * 
         * @brief <b> This visitor creates a hash from plant paths to plant pointers. </b>
         *  
         * @param system_hash The hash to populate
         */
        class dfs_system_by_path_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_system_by_path_visitor_t(hash_t<std::string, system_ptr_t>* system_hash)
            {
                systems = system_hash;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                if( g[u].system.get() != NULL )
                    ( *systems )[g[u].path] = g[u].system;
            }

            hash_t<std::string, system_ptr_t>* systems;
        };

        /**
         * This visitor will report the system pointers.
         * 
         * @brief <b> This visitor will report the system pointers. </b>
         *  
         * @param system_pointers The vector that will be populated after the visitor is done.
         */
        class dfs_system_visitor_t : public boost::default_dfs_visitor
        {

          public:

            dfs_system_visitor_t(std::vector<system_ptr_t>* system_pointers)
            {
                systems = system_pointers;
            }

            void discover_vertex(system_graph_t::directed_vertex_t u, const system_graph_t::directed_graph_t& g) const
            {
                if( g[u].system.get() != NULL )
                    systems->push_back(g[u].system);
            }

            std::vector<system_ptr_t>* systems;

        };

        system_graph_t::system_graph_t() { }

        system_graph_t::~system_graph_t() { }

        void system_graph_t::clear()
        {

            foreach(directed_edge_t e, boost::edges(system_graph))
            {
                boost::remove_edge(e, system_graph);
            }

            foreach(directed_vertex_t n, boost::vertices(system_graph))
            {
                boost::clear_vertex(n, system_graph);
                boost::remove_vertex(n, system_graph);
            }

        }

        system_graph_t::directed_vertex_t system_graph_t::add_node(const std::string& system_path, bool is_a_plant)
        {
            directed_vertex_t v_new = boost::add_vertex(system_graph);
            system_graph[v_new].path = system_path;
            system_graph[v_new].is_plant = is_a_plant;
            system_graph[v_new].parent_vertex = -1;
            return v_new;
        }

        system_graph_t::directed_edge_t system_graph_t::add_edge(const system_graph_t::directed_vertex_t& vertex_a, const system_graph_t::directed_vertex_t& vertex_b, const std::string& system_name, system_ptr_t system)
        {
            directed_edge_t new_e = (boost::add_edge(vertex_a, vertex_b, system_graph)).first;
            system_graph[new_e].name = system_name;
            system_graph[vertex_b].name = system_name;
            system_graph[vertex_b].system = system;
            system_graph[vertex_b].parent_vertex = vertex_a;

            return new_e;
        }

        void system_graph_t::get_plants(std::vector<plant_t*>& plants)
        {
            dfs_plant_visitor_t vis(&plants);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_plant_names(std::vector<std::string>& plant_names)
        {
            dfs_plant_name_visitor_t vis(&plant_names);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_plant_vertices(std::vector<directed_vertex_t>& plant_vertices)
        {
            dfs_plant_vertex_visitor_t vis(&plant_vertices);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_plant_paths(std::vector<std::string>& plant_paths)
        {
            dfs_plant_path_visitor_t vis(&plant_paths);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_name_plant_hash(hash_t<std::string, plant_t*>& plant_hash)
        {
            dfs_plant_by_name_visitor_t vis(&plant_hash);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_path_plant_hash(hash_t<std::string, plant_t*>& plant_hash)
        {
            dfs_plant_by_path_visitor_t vis(&plant_hash);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_systems(std::vector<system_ptr_t>& systems)
        {
            dfs_system_visitor_t vis(&systems);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_system_names(std::vector<std::string>& system_names)
        {
            dfs_system_name_visitor_t vis(&system_names);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_system_paths(std::vector<std::string>& system_paths)
        {
            dfs_system_path_visitor_t vis(&system_paths);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_name_system_hash(hash_t<std::string, system_ptr_t>& system_hash)
        {
            dfs_system_by_name_visitor_t vis(&system_hash);

            boost::depth_first_search(system_graph, visitor(vis));
        }

        void system_graph_t::get_path_system_hash(hash_t<std::string, system_ptr_t>& system_hash)
        {
            dfs_system_by_path_visitor_t vis(&system_hash);

            boost::depth_first_search(system_graph, visitor(vis));
        }

    }
}

