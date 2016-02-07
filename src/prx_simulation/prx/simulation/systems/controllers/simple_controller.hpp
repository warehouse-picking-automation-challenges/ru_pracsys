/**
 * @file simple_controller.hpp 
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

#ifndef SIMPLE_CONTROLLER_HPP
#define	SIMPLE_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/controllers/controller.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/control.hpp"

#include <boost/any.hpp>

namespace prx
{
    namespace util
    {
        class space_t;
        class space_point_t;
    }

    namespace sim
    {

        /**
         * Simple controller is a version of a controller that contains a single subsystem. 
         * Most of the implementations in this class is to speed up the process for a controller
         * that will not have more than one subsystems. In this case the controller does not have 
         * to search throughout all the subsystems. It is know that it will be only one subsystem.
         * 
         * @brief <b> A controller with single subsystem</b>
         * 
         * @author Athanasios Krontiris
         */
        class simple_controller_t : public controller_t
        {

          public:

            simple_controller_t();
            virtual ~simple_controller_t();

            /** 
             * @copydoc system_t::add_system(const std::string&, system_ptr_t)
             *
             * @node Will throw an \ref simple_add_remove_exception in case we try to add a subsystem
             * to a \ref simple_controller_t.     
             */
            virtual void add_system(const std::string& path, system_ptr_t system);

            /** 
             * @copydoc system_t::remove_system()
             *
             * @node Will throw an \ref simple_add_remove_exception in case we try to remove the 
             * only subsystem from a \ref simple_controller_t.
             */
            virtual void remove_system(const std::string& path);

            /**
             * Calls \c propagate to its single subsystem.
             * 
             * @brief Calls \c propagate to its single subsystem.
             */
            virtual void propagate(const double simulation_step = 0);

            /**
             * Calls \c compute_control to its single subsystem.
             * 
             * @brief \c Calls compute_control to its single subsystem.
             */
            virtual void compute_control();

            /** @copydoc controller_t::update_phys_configs(util::config_list_t &,unsigned& index) const*/
            virtual void update_phys_configs(util::config_list_t &configs, unsigned& index) const;
            /** @copydoc controller_t::update_phys_geoms(util::geom_map_t &) const*/
            virtual void update_phys_geoms(util::geom_map_t &geoms) const;

            /** @copydoc controller_t::update_system_graph(system_graph_t&)*/
            virtual system_graph_t::directed_vertex_t update_system_graph(system_graph_t& graph);

            /** @copydoc controller_t::verify() const*/
            virtual void verify() const;

            struct simple_add_remove_exception
            {

            };

          protected:

            /** @copydoc controller_t::update_vis_info() const*/
            virtual void update_vis_info() const;
        };


    }
}


#endif

