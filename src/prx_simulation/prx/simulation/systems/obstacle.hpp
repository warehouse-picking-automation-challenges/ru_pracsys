/**
 * @file obstacle.hpp 
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

#ifndef PRX_OBSTACLE_HPP
#define PRX_OBSTACLE_HPP

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/systems/system.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * This is a concrete class that represents the static obstacles. It inherits from 
         * the system class but uses only the state information. The controls cannot apply on 
         * the static obstacles. This class inherits from the \ref system_t class in order to 
         * have compatibility with \ref collision_checker_t and everything else needs to 
         * compare obstacles with real systems.
         * 
         * @brief <b> A concrete class that represents the static obstacles </b>
         * 
         * @authors Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield
         */
        class obstacle_t : public system_t
        {

          public:
            obstacle_t();

            virtual ~obstacle_t();

            /** @copydoc system_t::init() */
            void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * Obstacles do not have state space to return. It will throw a \ref invalid_operation_exception.
             *
             * @brief Obstacles do not have state space to return. It will throw a \ref invalid_operation_exception.
             * 
             */
            virtual const util::space_t* get_state_space() const;

            /**
             * Obstacles do not have control space to return. It will throw a \ref invalid_operation_exception.
             *
             * @brief Obstacles do not have control space to return. It will throw a \ref invalid_operation_exception.
             * 
             */
            virtual const util::space_t* get_control_space() const;

            /**
             * Obstacles cannot propagate. It will throw a \ref invalid_operation_exception.
             *
             * @brief Obstacles cannot propagate. It will throw a \ref invalid_operation_exception.
             * 
             */
            virtual void propagate(const double simulation_step = 0);

            /**
             * Obstacles do not compute a new control. It will throw a \ref invalid_operation_exception.
             *
             * @brief Obstacles do not compute a new control. It will throw a \ref invalid_operation_exception.
             * 
             */
            virtual void compute_control();

            /** @copydoc system_t::update_phys_configs(util::config_list_t &) const */
            virtual void update_phys_configs(util::config_list_t &configs, unsigned& index) const;

            /** @copydoc system_t::update_phys_geoms(util::geom_map_t &) const */
            virtual void update_phys_geoms(util::geom_map_t &geoms) const;
            
            /** @copydoc system_t::get_sensed_geoms(util::geom_map_t &) const */
            virtual void get_sensed_geoms(util::geom_map_t &geoms) const;

            /** @copydoc system_t::update_system_graph(system_graph_t&) */
            system_graph_t::directed_vertex_t update_system_graph(system_graph_t& graph);

            /** @copydoc system_t::set_param(const std::string&, const std::string&, const boost::any&) */
            virtual void set_param(const std::string& system_name, const std::string& parameter_name, const boost::any& value);

            /** @copydoc system_t::verify() */
            virtual void verify() const;

            /**
             * Returns the names of the geometries that the obstacle has.
             * 
             * @brief Returns the names of the geometries that the obstacle has.
             * 
             * @return A vector with all the names of the geometries.
             */
            virtual std::vector<std::string>* get_geometries_names();

            /**
             * Will be thrown if a subsystem path does not reference an existing subsystem.
             * @brief An exception for invalid functions that will be called. 
             * 
             * @param function A string with the name of the function that failed. 
             * 
             */
            class invalid_operation_exception : public std::runtime_error
            {

              public:

                invalid_operation_exception(const std::string& function) : std::runtime_error("Invalid function for this system: \"" + function + "\"."){ };
            };


            void update_root_configuration(const util::config_t& new_root);

            void update_point_cloud(const std::string& parameter_name, const boost::any& value);

          protected:

            /** @copydoc system_t::update_vis_info() */
            virtual void update_vis_info() const;

            /** @copydoc system_t::set_param(const std::string&, const boost::any&) */
            void set_param(const std::string& parameter_name, const boost::any& value);

            /** @brief The map with the geometries of the obstacle.*/
            util::geom_map_t geometries;

            /** @brief The map with all the configurations for each different geometry of the obstacle. Kept for efficiency*/
            util::config_list_t configurations;

            /** @brief The map with all the base configurations for each different geometry of the obstacle.*/
            util::config_list_t relative_configurations;

            util::config_t root_config;

            /** @brief The names of each different geometry in this obstacle. Each obstacle can consist of a 
             combination of different geometries.*/
            std::vector<std::string> geometries_names;
        };

    }
}


#endif