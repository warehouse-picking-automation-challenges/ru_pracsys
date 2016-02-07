/**
 * @file system.hpp 
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

#ifndef PRACSYS_SYSTEM_HPP
#define PRACSYS_SYSTEM_HPP

#include "prx/simulation/system_ptr.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"
#include "prx/simulation/system_graph.hpp"

#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"
#include "prx/simulation/plan.hpp"

#include <boost/range.hpp>
#include <boost/any.hpp>
#include <boost/range/any_range.hpp>
#include <pluginlib/class_loader.h>


namespace prx
{

    namespace util
    {
        class parameter_reader_t;
        class space_t;
    }

    namespace sim
    {

        /**
         * The lowest level, and perhaps most important, is the system  class - all controllers 
         * and plants are systems in PRACSYS . This functionality allows for other nodes, such
         * as planning, to reason over one or more systems  without the loss of generality (i.e.
         * planning over both controllers and plants).
         * 
         * @brief <b> The lowest level class in PRACSYS.</b>
         * 
         * @authors Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield
         */
        class system_t
        {

          public:
            /**
             * A generic iterator over a range of systems.
             *
             * Use this when you want to pass multiple systems to be iterated over,
             * but you don't want to expose the underlying container that is used.
             * 
             * To create a \ref system_range_t over the elements of an \c std::vector or some other container:
             * @code
             * std::vector<system_t*> systems;
             * // Put stuff in system_map.
             * system_range_t system_range = std::make_pair(systems.begin(), systems.end());
             * @endcode
             *
             * To create a \ref system_range_t over the values of an \c std::map or \c boost::unordered_map:
             * @code
             * #include <boost/range/adaptor/map.hpp>
             * std::map<std::string, system_t*> system_map;
             * // Put stuff in system_map.
             * system_range_t system_range = system_map | boost::adaptors::map_values;
             * @endcode
             *
             * You can iterate over the range using Boost foreach:
             * @code
             * foreach(system_t* system, system_range)
             * {
             *    PRX_INFO_S(system->pull_state_space()->print_point(system->pull_state()));
             * }
             * @endcode
             *
             * @sa http://www.boost.org/doc/libs/release/libs/range/index.html
             */
            typedef boost::any_range<system_ptr_t, boost::single_pass_traversal_tag, system_ptr_t, std::ptrdiff_t> system_range_t;


            system_t();

            virtual ~system_t(){ }

            /**
             * Initializes from the given parameters.
             * 
             * @brief Initializes from the given parameters.
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
             * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
             * is not specified in the \c reader then it will be read from the \c template_reader 
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL) = 0;

            /** 
             * A system's state space describes the topology and bounds of the system's state space.
             * If this system simply wraps another system as a subsystem, then this function may just
             * return the subsystem's space. If this system is a composition of subsystems or if it
             * has any state of its own, then it will return a pointer to its \ref util::space_t.
             * 
             * @brief Gets the state space of the system.
             * 
             * @return A pointer to this system's state space.
             */
            virtual const util::space_t* get_state_space() const = 0;

            /** 
             * A system's control space describes the topology and bounds of the system's control space.
             * A system may return a pointer to a subsystem's control space, a \ref util::space_t
             * that composes many subsystem's control spaces, or an entirely new control space.
             *
             * @brief Gets the control space of the system.
             * 
             * @return A pointer to this system's state space.
             */
            virtual const util::space_t* get_control_space() const = 0;

            /** 
             * Adds an initialized system to the subsystem tree.
             *
             * If the \c path references a location that is not a direct child of this system, then
             * this function will call its children recursively until this is the case.
             * 
             * @brief Add an initialized system to the subsystem tree.
             * 
             * @param path A slash-delimited path indicating the location to insert the system.
             * @param system The system to add to the tree.
             *
             * @exception controller_t::invalid_path_exception
             *            Will be thrown if the given subsystem path does not reference an existing subsystem.
             *
             * @exception controller_t::invalid_subsystem_tree_exception
             *            Will be thrown if adding a system to that location will result in an invalid
             *            subsystem tree. For example, adding a subsystem to a system that already has
             *            the maximum number of subsystems.
             *      
             */
            virtual void add_system(const std::string& path, system_ptr_t system);

            /** 
             * Removes a system branch from the subsystem tree and free its memory recursively.
             *
             * If the \c path references a location that is not a direct child of this system, then
             * this function will call its children recursively until this is the case.
             * 
             * @brief Removes a system branch from the subsystem tree and free its memory recursively.
             * 
             * @param path A slash-delimited path indicating the location of the system to remove.
             *
             * @exception controller_t::invalid_path_exception
             *            Will be thrown if the given subsystem path does not reference an existing subsystem.
             *
             * @exception controller_t::invalid_subsystem_tree_exception
             *            Will be thrown if removing a system from that location will result in an invalid
             *            subsystem tree. For example, removing the only subsystem from a system that requires
             *            exactly one subsystem.
             *      
             */
            virtual void remove_system(const std::string& path);


            /** 
             * Replaces a system in the subsystem tree with the new system.
             *
             * If the \c path references a location that is not a direct child of this system, then
             * this function will call its children recursively until this is the case.
             *
             * @brief Replaces a system in the subsystem tree with the new system.
             * 
             * @param path A slash-delimited path indicating the location to replace the system.
             * @param system The system to add to the tree after delete the previous system.
             *      
             */
            virtual void replace_system(const std::string& path, system_ptr_t system);

            /**
             * Given the current state and control, propagate the system for one step and update the current state.
             * 
             * @brief Given the current state and control, propagate the system for one step and update the current state.
             * @param simulation_step For how long we want the system to be propagated. By default is 0, but \ref simulator_t 
             * will give by default the simulation time that will read from the parameters.  
             */
            virtual void propagate(const double simulation_step = 0) = 0;

            /**
             *  Computes the control to pass down.
             * 
             * @brief Computes the control to pass down.
             */
            virtual void compute_control() = 0;

            /**
             * Performs a BVP solving calculation. Default implementation throws an error. 
             * In the event that an exact connection is impossible, this function should return
             * an empty plan in \ref result_plan.
             * 
             * @param start Where the problem starts
             * @param goal Where the problem ends
             * @param plan The sequence of controls and durations that achieves this connection if possible.
             * 
             * @brief Performs a BVP solving calculation.
             */
            virtual void steering_function(const state_t* start, const state_t* goal, plan_t& result_plan);


            /**
             * Appends contingency controls into this plan in order to match the given amount of time.
             * 
             * @param plan The sequence of controls and durations to modify.
             * @param duration The target duration of the plan.
             * 
             * @brief Appends contingency controls into this plan in order to match the given amount of time.
             */
            virtual void append_contingency(plan_t& result_plan, double duration);


            /**
             * Adds or updates configurations of a system's rigid bodies to match whatever the system's
             * current state is. This function will be called in every simulation step in order to update the
             * current configuration of all the visible systems and also it is needed for collision checking purposes. 
             * 
             * @param configs The configuration map that will be filled with the new configurations and will
             * be sent to the \ref visualization_t, \ref collision_checker_t, or anywhere that you need the new configurations.
             * 
             * @param index A counter that determines the location in the list for the system to update configurations. It also
             * serves as a flag for the function to create the configuration if necessary.
             * 
             * @brief Adds or updates configurations of a system's rigid bodies.
             */
            virtual void update_phys_configs(util::config_list_t &configs, unsigned& index) const = 0;


            /** 
             * Adds or updates extra informative configurations and geometries of a system to match whatever the system's
             * current state is. For example a goal configuration for a controller.
             * 
             * @brief Adds or updates extra informative configurations and geometries.
             * 
             */
            void update_visualization_info() const;


            /**
             * Adds or updates geometries of a system's rigid bodies. Mostly for visualizations 
             * purposes. \ref plant_t has a default implementation in order to fill the geometries 
             * of the plant. This function will be called only when there is a change in the geometries
             * of the systems. 
             * 
             * @brief Adds or updates geometries of a system's rigid bodies.
             * 
             * @param geoms The map with the updated geometries. 
             */
            virtual void update_phys_geoms(util::geom_map_t &geoms) const = 0;

            /**
             * Updates the system graph with all the systems that included in the
             * tree of the simulation. 
             * 
             * @param graph Is the graph that its going to be updated at the end of the
             * function call.
             * 
             * @brief Updates the system graph.
             */
            virtual system_graph_t::directed_vertex_t update_system_graph(system_graph_t& graph) = 0;
            
            
            /**
             * This function determines which rigid bodies are returned to the sensor
             * 
             * @brief Returns the set of geometries that can be sensed from the plant
             * @param geoms A set of sensed geometries for the plant
             */
            virtual void get_sensed_geoms(util::geom_map_t &geoms) const = 0;

            /**
             * Changes or overwrite the parameters, usually initialized in \ref system_t::init().
             * 
             * @brief Changes or overwrite parameters of the systems.
             * 
             * @param path A slash-delimited path indicating the location of the parameter that we want to change.
             * @param parameter_name The name of the parameter that we want to change.
             * @param value The value that we want to assign to the parameter. This value is assigned as boost::any 
             * so as we can give any type of values. An example how this boost::any can be used is followed.
             * 
             * In this example we change the x parameter of the system car1 to be 100.
             * 
             * @code
             * system->set_param("car1","state_space/x", boost::any(100));
             * @endcode
             */
            virtual void set_param(const std::string& path, const std::string& parameter_name, const boost::any& value) = 0;

            /**
             * Sets the pathname of the systems. Mostly used from the parents to set the correct
             * path to the children in the system tree.
             * 
             * @brief Sets the pathname of the systems.
             * 
             * @param path The slash-delimited path indicating the location of the system.
             */
            void set_pathname(const std::string& path)
            {
                pathname = path;
            }

            /**
             * Returns the pathname of the system.
             * 
             * @brief Returns the pathname of the system.
             */
            std::string get_pathname()
            {
                return pathname;
            }

            /**
             * Checks if the system is active. It is for visualization and collision checking purposes. 
             * If the system is active visualizer will visualize it and the collision checker will check 
             * for collisions with this system. If you want to have something visualized but do not care
             * for collisions you can place this system in a black list for the collision checker. 
             * 
             * @brief Returns if the system is active or not.
             * 
             * @return True if the system is active, else False
             */
            virtual bool is_active() const;

            /**
             * Activate/inactivate the system. Set the \c in_active parameter to the 
             * \c active variable.
             * 
             * @brief Activate/inactivate the system.
             * 
             * @param in_active The value to activate the system. True to activate
             * the system or False to deactivate it. 
             * @path path The slash-delimited path indicating the location of the system that we want to activate/deactivate.
             */
            virtual void set_active(bool in_active, const std::string& path = "");


            /** 
             * Makes sure that the system is valid, after the initialization and the possible changes of the parameters
             * of the system. 
             * 
             * @brief Makes sure that the system is valid.
             * 
             * @exception system_t::invalid_system_exception Throws an \ref system_t::invalid_system_exception if the system
             * is not valid.
             */
            virtual void verify() const = 0;

            /**
             * Returns the system under the location that is specified in the \c systems_path variable.
             * 
             * @brief Returns the system under the location that is specified in the \c systems_path variable.
             * 
             * @param system_path The slash-delimited path indicating the location of the system.
             * @return The system that is under the \c system_path.
             */
            virtual system_ptr_t get_system(const std::string& system_path) const;

            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<system_t>& get_loader();

            /**
             * Will be thrown if a subsystem path does not reference an existing subsystem.
             * 
             * @brief Will be thrown if a subsystem path does not reference an existing subsystem.
             * 
             * @param path The slash-delimited path indicating the wrong path.
             */
            class invalid_path_exception : public std::runtime_error
            {

              public:

                invalid_path_exception(const std::string& path) : std::runtime_error("Invalid subsystem path: \"" + path + "\"."){ };
            };

            /**
             * Will be thrown if adding or removing a system  will result in an invalid
             * subsystem tree. For example, removing the only subsystem from a system that requires
             * exactly one subsystem.
             * 
             * @brief Will be thrown if there is an invalid subsystem tree.
             * 
             * 
             */
            class invalid_subsystem_tree_exception : public std::runtime_error
            {

              public:

                invalid_subsystem_tree_exception(const std::string& reason) : std::runtime_error("Invalid subsystem tree: " + reason){ };
            };

            /**
             * Will be thrown if a system verification failed.
             * 
             * @brief Will be thrown if a system verification failed.
             * 
             * @param system_name The name of the system that made the system tree invalid. 
             */
            class invalid_system_exception : public std::runtime_error
            {

              public:

                invalid_system_exception(const std::string& system_name) : std::runtime_error("Invalid system: \"" + system_name + "\"."){ };
            };

            /**
             * Will be thrown if a subsystem path does not reference an existing subsystem.
             * 
             * @brief Will be thrown if a subsystem path does not reference an existing subsystem.
             * 
             * @param msg A message to explane the reason of the exception.
             */
            class invalid_element_exception : public std::runtime_error
            {

              public:

                invalid_element_exception(const std::string& msg) : std::runtime_error("Invalid element: \"" + msg + "\"."){ };
            };

            friend class system_graph;

          protected:

            /**
             * Update the visualization info that each system wants to visualize. If the 
             * system can be visualized, this function will be called through the \ref 
             * system_t::update_visualization_info(). This is the function that the other 
             * classes have to implement in order to visualize the extra information. 
             * 
             * @brief Update the visualization geometries.
             */
            virtual void update_vis_info() const = 0;

            /**
             * Set the value to the parameter with name \c parameter_name. The function \ref system_t::set_param()
             * has to be called at the end of the current \c set_param. Is checking if
             * the system is_active, else throws an \ref invalid_element_exception.
             * 
             * @param parameter_name Is the name of the parameter that we want to change.
             * @param value Is the value that we want to give to the parameter.
             * 
             * @brief Helper function for setting the value to a parameter.
             */
            virtual void set_param(const std::string& parameter_name, const boost::any& value);

            /**
             * The slash-delimited path indicating the location of the system. 
             * 
             * @brief The slash-delimited path indicating the location of the system. 
             */
            std::string pathname;

            /** 
             * The variable that checks if the system is active or inactive.
             *  
             * @brief Checks if the system is active or inactive. 
             */
            bool active;

            /** 
             * Flag for the visualization.
             * 
             * @brief Flag for the visualization.     
             */
            bool visualizable;

            /**
             * Memory for the state of the system. Its also the memory that is passed in the space for the state of the system.
             * 
             * @brief Memory for the state of the system.
             */
            std::vector<double*> state_memory;

            /**
             * Memory for the control of the system. Its also the memory that is passed in the space for the control of the system.
             * 
             * @brief Memory for the control of the system.
             */
            std::vector<double*> control_memory;

          private:
            /** 
             * The pluginlib loader for the \ref system_t class.
             * 
             * @brief The pluginlib loader for the \ref system_t class.
             */
            static pluginlib::ClassLoader<system_t> loader;
        };

        /**
         * A global namespace where you can find the \c simulation_step for the simulator.
         * 
         * @brief <b> A global namespace where you can find the \c simulation_step for the simulator. </b>
         */
        namespace simulation
        {
            /**
             * The simulation_step for the current execution.
             * 
             * @brief The simulation_step for the current execution.
             */
            extern double simulation_step;

            extern bool update_all_configs;
        }

    }
}

#endif












