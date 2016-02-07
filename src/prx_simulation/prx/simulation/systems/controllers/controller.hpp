/**
 * @file controller.hpp 
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

#ifndef PRX_CONTROLLER_HPP
#define	PRX_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/control.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/sensing/sensing_info.hpp"
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
         * This is the basic class for a controller implementation. Controllers are also systems in 
         * order to be able to stack many controllers about controllers or plants.
         * The controllers are integrated with simulated systems in a manner that allows for 
         * deterministic, and non-deterministic, simulation. A controller will compute the next control
         * that will pass it to the system(s) below it, in the simulation tree, in order to compute 
         * a new control, if the subsystem(s) is(are) other controller(s), or to use this control to 
         * propagate, if the subsystem(s) is(are) plant(s). There are two different type of controllers. \n
         * 1. The \ref simple_controller_t, that deal with one subsystem that can be either another controller 
         * or a plant that the controller is controlling (eg. \ref VO_t, \ref manual_t, etc.).\n
         * 2. The composite controllers that deal with multiple subsystems that can be either other controllers 
         * or plants (eg. \ref router_t, etc.).\n
         * 
         * @brief <b>The basic class for implementing a controller. </b>
         * 
         * @author Athanasios Krontiris, Andrew Kimmel, Zakary Littlefield
         */
        class controller_t : public system_t
        {

          public:


            controller_t();

            virtual ~controller_t();

            /**
             * Initializes a controller from the given parameters. It will also call \c init for all the subsystems.
             * Finally, it will construct the \c input_control_space of the controller, in case it has one.
             * 
             * @brief Initializes a controller from the given parameters. 
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
             * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
             * is not specified in the \c reader then it will be read from the \c template_reader 
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** 
             * @copydoc system_t::add_system(const std::string&, system_ptr_t)
             *
             * @note After the controller adds a new system, it will update
             * the spaces of the controller with the new subsystem's spaces
             */
            virtual void add_system(const std::string& path, system_ptr_t system);

            /** 
             * @copydoc system_t::remove_system(const std::string&)
             *
             * @note After the controller removes the system, it will update
             * its spaces by removing the system's state and control spaces
             * 
             */
            virtual void remove_system(const std::string& path);

            /** 
             * @copydoc system_t::replace_system(const std::string&, system_ptr_t)
             *
             * @note After the controller replaces the system with the new system,
             * it will update the spaces of the controller based on the new system
             * 
             */
            virtual void replace_system(const std::string& path, system_ptr_t system);

            /** @copydoc system_t::get_system(const std::string&) const*/
            virtual system_ptr_t get_system(const std::string& path) const;

            /** 
             * @copydoc system_t::propagate(const double )
             * 
             * @note By default will call propagate to all the sybsystems.
             */
            virtual void propagate(const double simulation_step = 0);

            /**
             * @copydoc system_t::compute_control()
             * 
             * @note By default will call the compute in all the subsystems.
             */
            virtual void compute_control();

            /** @copydoc system_t::get_state_space() const */
            virtual const util::space_t* get_state_space() const;

            /** @copydoc system_t::get_control_space() const */
            virtual const util::space_t* get_control_space() const;

            /** @copydoc system_t::set_param(const std::string&, const std::string& parameter_name, const boost::any&);*/
            virtual void set_param(const std::string& path, const std::string& parameter_name, const boost::any& value);

            /**
             * By default will call \c update_phys_configs to its subsystems. Controllers 
             * most of the time they do not have state to update the configuration. 
             * 
             * @brief By default will call \c update_phys_configs to its subsystems.
             */
            virtual void update_phys_configs(util::config_list_t &configs, unsigned& index) const;

            /**
             * By default will call \c update_phys_geoms to its subsystems. Controllers 
             * most of the time they do not have physical geometries to visualize. 
             * 
             * @brief By default will call \c update_phys_geoms to its subsystems.
             */
            virtual void update_phys_geoms(util::geom_map_t &geoms) const;
            
            /**
             * By default will call \c get_sensed_geoms to its subsystems. Controllers 
             * most of the time they do not have physical geometries to be sensed. 
             * 
             * @brief By default will call \c update_phys_geoms to its subsystems.
             */
            virtual void get_sensed_geoms(util::geom_map_t &geoms) const;

            /** 
             * Makes sure that the controller is valid, after the initialization and the possible changes of the parameters.
             * It will also call \c verify to its subsystems.
             *     
             * @brief Makes sure that the controller is valid.
             * 
             * @exception system_t::invalid_system_exception Throws an \ref system_t::invalid_system_exception if the system
             * is not valid.
             * 
             * @exception controller_t::space_mismatch_exception Throws when the control space of the controller mismatch with 
             * the constructed output control space of the subsystems.
             * 
             */
            virtual void verify() const;

            /**
             * Will construct the memory of the \c output_control_space of the controller, based on the 
             * \c input_control_space of the subsystems. It will also build the state \ref util::space_t of the 
             * controller if the current controller has a state \ref util::space_t.
             * 
             * @brief Will construct the memory of the \c output_control_space of the controller.
             */
            virtual void construct_spaces();

            /**
             * @copydoc system_t::update_system_graph(system_graph_t& )
             * 
             * @note Recursively will call \c update_system_graph to all of its subsystems.
             */
            virtual system_graph_t::directed_vertex_t update_system_graph(system_graph_t& graph);

            /** @copydoc system_t::set_active(bool, const std::string&)*/
            virtual void set_active(bool in_active, const std::string& path);

            /**
             * Returns the controller's state space, if the controller has one.
             * 
             * @brief Returns the controller's state space, if the controller has one.
             * 
             * @return The controller's state space, if the controller has one.
             */
            util::space_t* get_controller_state_space() const;
            
            std::vector<sensing_info_t*> get_sensing_info();

            /**
             * 
             * @copydoc system_t::append_contingency(plan_t& ,double);
             * 
             * @note It will also call \c append_contingency for all of its subsystems.
             */
            virtual void append_contingency(plan_t& result_plan, double duration);

            /**
             * Controllers will throw this exception, in order to request from the controller above, to also 
             * delete the current controller from the tree.
             */
            struct removal_exception
            {

            };

            /** 
             * This exception is when the read \c output_control_space mismatch with the constructed 
             * \c output_control_space based on the \c input_control_space of the subsystems.
             */
            struct space_mismatch_exception
            {

            };

          protected:

            /** The state \ref stace_t of the subsystems below this controller.*/
            util::space_t* state_space;
            /** Controller's state \ref stace_t, if has one.*/
            util::space_t* controller_state_space;
            /** The control \ref util::space_t that the controller expect from its parent in the tree.*/
            util::space_t* input_control_space;
            /** The control that the controller will build for the subsystems.*/
            util::space_t* output_control_space;

            /** The computed control from the controller that will pass to the subsystems.*/
            control_t* computed_control;

            /** The hash map of all the subsystems.*/
            util::hash_t<std::string, system_ptr_t > subsystems;
            /** A list of all associated sensing infos for this controller */
            std::vector<sensing_info_t*> sensing_info;
            /** A list with all the subsystems names.*/
            std::vector<std::string> subsystem_names;
            /** A helper iterator for the hash map of the subsystems.*/
            util::hash_t<std::string, system_ptr_t >::iterator subsystems_iter;

            /**
             * @copydoc system_t::update_vis_info() const;
             * 
             * @note The default implementation for the controller, will call the \c update_vis_info 
             * for all the systems.
             */
            virtual void update_vis_info() const;

            /** @copydoc system_t::set_param(const std::string&, const boost::any&)*/
            virtual void set_param(const std::string& parameter_name, const boost::any& value);

            /**
             * Creates and initializes the subsystems of the controller. 
             * 
             * @param path The slash-delimited path indicating the location that the system will be stored.
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
             * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
             * is not specified in the \c reader then it will be read from the \c template_reader 
             * 
             * @return The pointer of the initialized system.
             * 
             * @brief Creates and initializes the subsystems of the controller.
             */
            system_ptr_t create_subsystem(const std::string& path, const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);


        };


    }
}

#endif

