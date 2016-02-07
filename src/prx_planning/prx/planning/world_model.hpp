/**
 * @file world_model.hpp
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

#ifndef PRACSYS_WORLD_MODEL_HPP
#define PRACSYS_WORLD_MODEL_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/simulators/simulator.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {
        class space_t;
        class embedded_space_t;
    }

    namespace sim
    {
        class collision_list_t;
        class vector_collision_list_t;
    }

    namespace plan
    {

        struct context_flags
        {

            context_flags(bool active = false, bool plannable = false)
            {
                this->active = active;
                this->plannable = plannable;
            }

            void set(bool active = false, bool plannable = false)
            {
                this->active = active;
                this->plannable = plannable;
            }

            bool active;
            bool plannable;
        };

        /**
         * @anchor space_descriptor_t
         *
         * @ingroup planning
         *
         * This support class helps describe the space in which a planner operates, and
         * contains information about the state and control spaces as well as an
         * activation list.  These space descriptors can be hot-swapped in the world
         * model to change planning contexts.
         *
         * @brief <b> World model space descriptor context class. <\b>
         *
         * @author Zakary Littlefield
         */
        class planning_context_t
        {

          public:
            /** @brief List of system activations, representing which systems are planned for. */
            util::hash_t< std::string, bool > activation_list;

            /** @brief State space corresponding to the planned systems. */
            util::space_t* planning_state_space;

            /** @brief Control space corresponding to the planned systems. */
            util::space_t* planning_control_space;

            /** @brief Temporary state for intermediate operations. */
            sim::state_t* temp_state;

            /** @brief State space for task specific operations where collisions with these systems are considered. */
            util::space_t* active_space;

            /** @brief State space for all systems that are not considered in planning or collision. */
            util::space_t* inactive_space;

            planning_context_t()
            {
                planning_state_space = NULL;
                planning_control_space = NULL;
                active_space = NULL;
                inactive_space = NULL;
            }
        };

        /**
         * @anchor world_model_t
         *
         * @ingroup planning
         *
         * This class represents a planner's view of the world, and is responsible for
         * reporting state information.The world model works closely with planning
         * modules for proper functioning.
         *
         * @brief <b> Abstract world representation class for planners. </b>
         *
         * @author Zakary Littlefield
         */
        class world_model_t
        {

          public:

            ~world_model_t(){ }

            /**
             * Initialize the world_model from the given parameters. Also performs a massive initialization
             * procedure that involved subdividing the world state space into an embedded space for planning.
             * This embedded space can perform the task of hiding certain systems from the planner for dynamic
             * obstacle planning, or for planning for multiple systems in a decentralized way. This can also
             * be used to create a task space, a typically lower dimensional space that captures the task that
             * is being achieved.
             *
             * These things are accomplished through the use of the embedded space class.
             *
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             * @param template_reader A secondary \ref util::parameter_reader_t with a dictionary of default parameters.
             */
            void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @brief Retrieval function for the current state space being planned over.
             *
             * @return The space that the world model is currently using
             */
            util::space_t* get_state_space() const;

            /**
             * @brief Retrieval function for the current full state space being planned over.
             *
             * @return The full space that the world model is currently using
             */
            const util::space_t* get_full_state_space() const;

            /**
             * @brief Retrieval function for the current control space being planned over.
             *
             * @return The space of controls the world model is currently using
             */
            util::space_t* get_control_space() const;
            /**
             * @brief Retrieval function for the current full control space being planned over.
             *
             * @return The full space of controls the world model is currently using
             */
            const util::space_t* get_full_control_space() const;



            /**
             * @brief Retrieval function for space of systems that are active but not being planned.
             *
             * @return The space for active parts that are not being planned.
             */
            util::space_t* get_active_space() const;

            /**
             * @brief Retrieval function for space of systems that are inactive.
             *
             * @return The space for inactive parts.
             */
            util::space_t* get_inactive_space() const;

            /**
             * @brief Retrieves the current state of the internal simulator of the world model.
             *
             * @return The current state of the simulator.
             */
            sim::state_t* pull_state();

            /**
             * Copy the given state over the world current state.
             *
             * The state must be compatible with the system's state space.
             * All the implementations of systems have to enforce the bounds
             * of the state space on the state that they just receive \c source.
             *
             * @param source A point from the simulator's state space.
             */
            void push_state(const sim::state_t * const source);

            /**
             * @brief Retrieval of system configurations for visualization and other purposes.
             *
             * When planning for individual systems, there may be a configuration
             * in SE3 that represents the state of that system in the world.
             *
             * The default implementation throws an exception.
             *
             * @param state The state to set the simulator to for the configuration retrieval.
             * @param system_names The names of the systems we wish to get configurations for.
             *
             * @return A projection of the world state in SE3.
             */
            const std::vector<util::config_t> get_configs(const sim::state_t * const state, std::vector<std::string> system_names);

            /**
             * @brief Resets the initial state.
             *
             * @param initial_state to start planning from.
             */
            void reset(const sim::state_t * const initial_state);

            // /**
            //  * The default implementation that returns the resulting state. It cuts the time parameter into simulation steps.
            //  *
            //  * @param state The starting state to simulate from.
            //  * @param control The control to apply for the entire simulation.
            //  * @param time The amount of time to apply the control.
            //  * @param result The resulting state of the world after propagation.
            //  */
            // void propagate(const sim::state_t * const state, const sim::control_t * const control,
            //                const double time, sim::state_t* result);

            // /**
            //  * Does the same operations as propagate above, but provides storage as a parameter
            //  *
            //  * @param state The starting state to simulate from.
            //  * @param control The control to apply for the entire simulation.
            //  * @param time The amount of time to apply the control.
            //  * @param traj A set of points already allocated.
            //  */
            // void propagate(const sim::state_t * const state, const sim::control_t * const control,
            //                const double time, sim::trajectory_t& traj);


            /**
             * @brief Performs a single propagation step.
             *
             * @param state The starting state to simulate from.
             * @param control The control to apply for the entire simulation.
             * @param time The amount of time to apply the control. Is not split into simulation steps. Assumed to be a simulation step.
             * @param result The resulting state of the world after propagation.
             */
            void propagate_once(const sim::state_t * const state, const sim::control_t * const control,
                                const double time, sim::state_t * const result);

            /**
             * @brief Propagate the simulator given an entire plan of controls.
             *
             * @param state The starting state to simulate from.
             * @param plan The plan to apply.
             * @param traj The trajectory resulting from the plan.
             */
            void propagate_plan(const sim::state_t * const state, const sim::plan_t& plan, sim::trajectory_t& traj, bool collision_in_traj = false, bool print_death = false );

            /**
             * @brief Propagate the simulator given an entire plan of controls.
             *
             * @param state The starting state to simulate from.
             * @param plan The plan to apply.
             * @param result The resulting state of the propagation.
             */
            void propagate_plan(const sim::state_t * const state, const sim::plan_t& plan, sim::state_t* result);

            /** @copydoc sim::simulator_t::get_colliding_bodies( const sim::state_t* const ) */
            sim::collision_list_t* get_colliding_bodies(const sim::state_t * const state);

            /** @copydoc sim::simulator_t::get_colliding_bodies( const sim::state_t* const, sim::collision_list_t* ) */
            sim::collision_list_t* get_colliding_bodies(const sim::state_t * const state, sim::collision_list_t* collision_list);

            /**
             * Given an input clearance threshold, this function determines which bodies are
             * within clearance distance from each other, and returns a list of all such paris.
             *
             * @brief Get a list of near-colliding bodies.
             *
             * @param state The state to use for near-collision checking.
             * @param eps The clearance value used to determine if bodies are near collision.
             *
             * @return A list of all the nearly-colliding bodies in the world model.
             */
            sim::collision_list_t* get_near_colliding_bodies(const sim::state_t * const state, double eps);

            /**
             * Given an input clearance threshold, this function determines which bodies are
             * within clearance distance from each other, and returns a list of all such paris.
             *
             * @brief Get a list of near-colliding bodies.
             *
             * @param state The state to use for near-collision checking.
             * @param collision_list The collision list which says which collisions are important to test.
             * @param eps The clearance value used to determine if bodies are near collision.
             *
             * @return A list of all the nearly-colliding bodies in the world model.
             */
            sim::collision_list_t* get_near_colliding_bodies(const sim::state_t * const state, sim::collision_list_t* collision_list, double eps);

            /**
             * Check if a state is invalid (in collision or outside of bounds). Should only be called from validity_checker
             *
             * @param state the state to check for validity
             * @return true if the given state is valid
             */
            bool valid_state(const sim::state_t * const state);

            /**
             * Check if a state near invalid. Should only be called from validity_checker
             *
             * @param eps The clearance for which to do the validity checking
             * @param state the state to check for validity
             * @return true if the given state is valid
             */
            bool near_valid_state(double eps, const sim::state_t* state);

            /**
             * Get the clearance from obstacles for this state.
             *
             * @param state the state to check.
             * @return The minimum clearance.
             */
            double state_clearance(const sim::state_t* state);

            /**
             * @brief Use a steering function to drive toward a desired state.
             *
             * This function attempts to solve the boundary value problem, where, given
             * an initial and goal state, the world model attempts to drive the simulation
             * from the start state to the goal state as exactly as possible.
             *
             * @param start The state from which to start propagating.
             * @param goal The desired state of the simulation.
             * @param result The resulting plan which drives the system toward goal.
             */
            void steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result);

            /**
             * @copydoc sim::simulator_t::get_colliding_bodies()
             */
            sim::collision_list_t* get_colliding_bodies();

            /**
             * Given an input clearance threshold, this function determines which bodies are
             * within clearance distance from each other, and returns a list of all such paris.
             *
             * @brief Get a list of near-colliding bodies.
             *
             * @param eps The clearance value used to determine if bodies are near collision.
             *
             * @return A list of all the nearly-colliding bodies in the world model.
             */
            sim::collision_list_t* get_near_colliding_bodies(double eps);

            /**
             * @brief Return a contingency, or last resort control.
             *
             * Get a control that is the start of a contingency plan for the current
             * state. The default implementation returns the zero control.
             *
             * @param curr_state The state from which to compute the contingency control.
             *
             * @return The contingency for the system.
             */
            sim::control_t* compute_contingency_control(const sim::state_t* curr_state);

            /**
             * @brief Copy the control to the simulator.
             *
             * @param source The control to pass
             */
            void push_control(const sim::control_t*);

            sim::simulator_t* get_simulator() const;

            /**
             * @brief Get a system from the simulator
             *
             * Retrieves a system belonging to the system tree under the world model.
             * Recursively searches for a system at the given path.
             *
             * @param source The path to the system to retrieve.
             *
             * @return The system pointer for the requested system.
             */
            sim::system_ptr_t get_system(const std::string& path) const;

            /**
             * @brief This function initializes the simulator's collision list.
             *
             * @param reader A \ref util::parameter_reader_t from which to read the collision list.
             */
            void init_collision_list(const util::parameter_reader_t * const reader);

            /**
             * @brief This function updates the world model with ground truth information.
             *
             * From time to time, the world model will receive ground truth information from
             * the simulation node or from sensor data.  This function will update the world
             * model with this new state information.
             *
             * @param time Time stamp for the newly acquired information.
             * @param elements The actual ground truth information received.
             */
            void update_ground_truth(const double time, const std::vector<double>& elements);

            /**
             * @brief Adds a system-system pair to a collision list.
             *
             * @param system1 The name of the first system of the pair to add.
             * @param system2 The name of the second system of the pair to add.
             * @param black_list The collision list to augment.
             * @param plants The list of plants which contains system1 and system2.
             */
            void add_system_system_in_collision_list(const std::string& system1, const std::string& system2, const sim::vector_collision_list_t* black_list, util::hash_t<std::string, sim::plant_t*>& plants);

            /**
             * @brief Adds a system-obstacle pair to a collision list.
             *
             * @param system1 The name of the system half of the pair to add.
             * @param obstacle The name of the obstacle half of the pair to add.
             * @param black_list The collision list to augment.
             * @param plants The list of plants which contains system1.
             * @param obstacles The list of obstacles which contains obstacle.
             */
            void add_system_obstacle_in_collision_list(const std::string& system, const std::string& obstacle, const sim::vector_collision_list_t* black_list, util::hash_t<std::string, sim::plant_t*>& plants, util::hash_t<std::string, sim::system_ptr_t>& obstacles);

            /**
             * @brief Fills a collision list with all possible collision pairs.
             *
             * @param plants A map containing the plants in the simulation.
             * @param obstacles A map containing the obstacles in the simulation.
             * @param black_list The collision list to populate.
             */
            void collision_list_all_vs_all(util::hash_t<std::string, sim::plant_t*>& plants, util::hash_t<std::string, sim::system_ptr_t>& obstacles, const sim::vector_collision_list_t* black_list);

            /**
             * @brief Space context switch function.
             *
             * This function switches the context of the planning to work over a new
             * space.  This function loads a new \ref space_descriptor_t for planning.
             *
             * @param name The name of the space which we want to plan for.
             */
            void use_context(std::string name);

            /**
             * @brief Used to check if a space has an entry in the world model's space name map
             *
             * @param name The name of the space we are checking for existence.
             *
             * @return A flag indicating whether the specified space exists in the world model.
             */
            bool check_context_name(std::string name);

            void update_sensing();



            /**
             * @brief Returns the currently applied context.
             *
             * @return The context name that is currently applied.
             */
            std::string get_current_context();

            /**
             * @brief Ask for a system pointer given a path.
             *
             * Given a path from the world model (i.e. world_model/system1/system2), this function will cut off the world
             * model name and ask the simulator for a system.
             *
             * @param sys_path The path to the system.
             *
             * @return The system requested.
             */
            sim::system_ptr_t get_split_system(const std::string& sys_path) const;

            /**
             * @brief Gets the system graph.
             *
             * Returns a reference to the system graph for direct access to plants etc.
             *
             * @returns A reference to the system graph.
             */
            sim::system_graph_t& get_system_graph();

            /**
             * Sets the respond variable. Does not reset it (be sure to reset it to whatever value
             * you need if necessary).
             *
             * @brief Sets respond
             * @param resp The value of the respond variable
             */
            void set_propagate_response(bool resp);

            /**
             * @brief Create a new planning context at runtime.
             * @details [long description]
             *
             * @param new_context_name [description]
             * @param mappings [description]
             * @param default_flags [description]
             */
            void create_new_planning_context(std::string new_context_name, util::hash_t<std::string, context_flags>& mappings, context_flags default_flags = context_flags());


            util::hash_t<std::string, sim::system_ptr_t>& get_obstacles();

          protected:
            /**
             * @brief Fast, internal single step propagate function.
             *
             * This function is the same as propagate_once, just doesn't call push state or push control.
             * This helps to minimize the calls to push_state and push_control.  This should only be
             * called in propagate.
             *
             * @param state The starting state to simulate from.
             * @param control The control to apply for the entire simulation.
             * @param time The amount of time to apply the control.
             * @param result The resulting state of the world after propagation.
             */
            void propagate_optimized(const sim::state_t * const state, const sim::control_t * const control,
                                     const double time, sim::state_t * const result);

            /** @brief The unit of time for a single atomic step of the simulation. */
            double simulation_step;

            /** @brief A flag indicating whether the underlying simulator is physics-based. */
            bool phys_based_sim;

            /** @brief A flag indicating whether the optimized propagations should be used.*/
            bool use_optimized_propagate;

            /** @brief The currently used space name by the world model. */
            std::string context_name;

            /** @brief A pointer to the currently used state space. */
            util::space_t* selected_state_space;

            /** @brief A pointer to the currently used control space. */
            util::space_t* selected_control_space;

            /** @brief The internal simulator used for simulating system propagation. */
            sim::simulator_t* simulator;

            /** @brief The collision list of pairs to consider for collision. */
            sim::collision_list_t* collision_list;

            /** @brief The mapping of all configuration the world model needs to update. */
            util::config_list_t config_map;

            /** @brief The system graph containing the system hierarchy of the simulation. */
            sim::system_graph_t sys_graph;

            /** @brief A map of all created spaces for this world model. */
            util::hash_t< std::string, planning_context_t > contexts;

            /** @brief A list of all of the low-level plants in the internal simulator. */
            util::hash_t<std::string, sim::plant_t*> plants;

            /** @brief A list of all of the systems in the internal simulator. */
            util::hash_t<std::string, sim::system_ptr_t> systems;

            /** @brief Storage of trajectories for each system. */
            util::hash_t< std::string, sim::trajectory_t > stored_trajectories;

            /** @brief Storage of plans for each system. */
            util::hash_t< std::string, sim::plan_t > stored_plans;

            /** @brief The current full state of the underlying simulation. */
            sim::state_t* full_state;

            /** @brief The current full control of the underlying simulation. */
            sim::control_t* full_control;

            /** @brief Determines if world model uses propagate and respond */
            bool respond;
        };

        /**
         * @brief Colliding bodies call for a given state.
         *
         * @param state The state to check for colliding bodies.
         *
         * @return A list of all pairs of collisions which occur at the given state.
         */
        inline sim::collision_list_t* world_model_t::get_colliding_bodies(const sim::state_t * const state)
        {
            push_state(state);
            return get_colliding_bodies();
        }

        /**
         * @brief Colliding bodies call for a given state, looking at specific systems.
         *
         * @param state The state to check for colliding bodies.
         * @param collision_list The list of pairs to check for collision.
         *
         * @return A list of all pairs of collisions which occur at the given state.
         */
        inline sim::collision_list_t* world_model_t::get_colliding_bodies(const sim::state_t * const state, sim::collision_list_t* collision_list)
        {
            push_state(state);
            simulator->link_collision_list(collision_list);
            return get_colliding_bodies();
        }

        inline sim::collision_list_t* world_model_t::get_near_colliding_bodies(const sim::state_t * const state, double eps)
        {
            push_state(state);
            return get_near_colliding_bodies(eps);
        }

        inline sim::collision_list_t* world_model_t::get_near_colliding_bodies(const sim::state_t * const state, sim::collision_list_t* collision_list, double eps)
        {
            push_state(state);
            simulator->link_collision_list(collision_list);
            return get_near_colliding_bodies(eps);
        }


    }
}

#endif
