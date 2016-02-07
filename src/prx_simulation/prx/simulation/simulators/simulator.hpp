/**
 * @file simulator.hpp 
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

#ifndef PRACSYS_SIMULATOR_HPP
#define PRACSYS_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"
#include "prx/simulation/systems/controllers/router.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/simulation/sensing/sensing_model.hpp"


namespace prx
{
    namespace sim
    {

        class collision_list_t;
        class collision_checker_t;

        /**
         * The simulator is responsible for propagating systems, is a \ref router_t controller containing all 
         * controllers and plants. The simulator contains several classes which are interfaces 
         * used for the development of controllers. It, also, provides plugin capability for 
         * different physics engines, as well as different collision checkers. The currently 
         * supported physics engine is the ODE. Adding a new physics-simulation engine as a 
         * plugin would simply involve extending the \ref simulator_t, \ref plant_t, 
         * \ref obstacle_t, and \ref collision_checker_t class. If a user does not require the 
         * use of a physics-simulator, they can simply disable this by omitting it from the input file. 
         * 
         * @brief <b> The simulator is responsible for propagating systems. </b>
         * 
         * @autors Athanasios Krontiris
         */
        class simulator_t : public router_t
        {

          public:

            simulator_t();
            ~simulator_t();

            /** 
             * Intitialize the simulator from the given parameters.
             *
             * @brief Intitialize the simulator from the given parameters.
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this simulator.
             * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
             * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
             * is not specified in the \c reader then it will be read from the \c template_reader 
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);


            /**
             * Checks if there is at least one collision between systems
             * in the \c collision_list. All the systems must be in the updated 
             * current position. 
             * 
             * @brief Checks if there is at least one collision.
             * 
             * @return True If there is at least a collision. False If there is no
             * collision between systems
             */
            virtual bool in_collision();


            /**
             * Checks the minimum distance among all collisions
             * in the \c collision_list. All the systems must be in the updated 
             * current position. 
             * 
             * @brief Finds the minimum distance between objects.
             * 
             * @return Distance between objects.
             */
            virtual double collision_distance();


            /**
             * Checks if there is at least one near-collision between systems
             * in the \c collision_list. All the systems must be in the updated 
             * current position. 
             * 
             * @brief Checks if there is at least one near-collision.
             * 
             * @return True If there is a near-collision. False If there is no
             * such near-collisions.
             */
            virtual bool near_collision( double eps );

            /**
             * Links the list \c collision_list of the systems to the \ref collision_checker_t
             * that we want to check for collisions.
             * 
             * @brief Links a collision list to the \ref collision_checker_t
             * 
             * @param collision_list Is the white list of the systems that we are 
             * interested to have collision check. 
             */
            virtual void link_collision_list(collision_list_t* collision_list);

            /**
             * It returns a list of pairs with all the bodies of the systems that have 
             * been collided, during this simulation step.     
             * 
             * @brief It returns a list of pairs with all the bodies of the systems.
             * 
             * @return A list with the pairs of the bodies of the systems in collision. 
             */
            virtual collision_list_t* get_colliding_bodies();

            /**
             * It returns a list of pairs with all the bodies of the systems that have 
             * been collided, during this simulation step.     
             * 
             * @brief It returns a list of pairs with all the bodies of the systems.
             * 
             * @return A list with the pairs of the bodies of the systems in collision. 
             */
            virtual collision_list_t* get_near_colliding_bodies( double eps );

            /**
             * The function responsible for the propagation of the systems. All the active
             * systems will use the computed control from the controller that have been controlled 
             * by, in order to propagate their state. 
             * 
             * @brief The function responsible for the propagation of the systems.
             * 
             * @param simulation_step The time that simulator will simulate the systems.
             */
            virtual void propagate(const double simulation_step = 0);

            /**
             * The function responsible for the propagation of the systems and responding to 
             * any collision. All the active systems will use the computed control from the 
             * controller that have been controlled by, in order to propagate their state. At
             * the end of the simulation step a response for all the systems in collision will 
             * be executed. 
             * 
             * @brief The function responsible for the propagation of the systems and responding to any collision.     
             */
            virtual void propagate_and_respond() = 0;


            /**
             * Checks if there is at least one collision between the systems
             * in the already assigned \c collision_list after sets the new \c state 
             * for all the systems.
             * 
             * @brief Checks if there is at least one collision based on the given state.
             * 
             * @param state The new state that the systems are. 
             * 
             * @return True If there is at least one collision. False If there is no
             * collision between the systems in the \c collision_list
             */
            bool in_collision(const state_t * const state);

            /**
             * First copies the given state to the systems. Links to the collision_checker
             * the new collision_list and then checks for at least one collision between
             * the systems in the \c collision_list.
             * 
             * @brief Checks for collisions given a state and a collision list.
             * 
             * @param state The new state that the systems are. 
             * @param collision_list Is the list with the pairs of systems that we are interested in 
             * checking for collisions.
             * 
             * @return True If there is at least one collision. False If there is no
             * collision between the systems in the \c collision_list
             */
            bool in_collision(const state_t * const state, collision_list_t* collision_list);

            /**
             * It returns a list of pairs with all the systems that have been collided, during
             * this simulation step and belongs in the already assigned \c collision_list. 
             * First sets the new state to the systems and then call the \ref collision_checker_t::get_colliding_bodies()
             * of the collision checker.
             * 
             * @brief Returns a list with all the systems that collided in this simulation step.
             *    
             * @param state The new state that the systems are.  
             * 
             * @return A list with the pairs of the systems in collision. 
             */
            virtual collision_list_t* get_colliding_bodies(const state_t * const state);


            /**
             * First copy to the systems the given \c state. Links to the \ref collision_checker_t
             * the new \c collision_list and then checks for collisions between the systems
             * in the \c collision_list. It returns a list of pairs with all the systems that have 
             * been collided, during this simulation step.
             * 
             * @brief Returns a list with all the colliding systems given a \c state and a \c collision_list
             *    
             * @param state The new state that the systems are.  
             * @param collision_list Is the list with the pairs of systems that we are interested to 
             * check for collisions.
             * 
             * @return A list with the pairs of the systems in collision. 
             */
            virtual collision_list_t* get_colliding_bodies(const state_t * const state, collision_list_t* collision_list);
            
            virtual collision_checker_t* get_collision_checker();

            /**
             * An implementation for the system_t::add_system().     
             * The simulator has also to add the system in the collision list and in the collision
             * checker. 
             * 
             * @copydoc router_t::add_system()
             *      
             */
            virtual void add_system(const std::string& path, system_ptr_t system);

            /** 
             * An implementation for the system_t::remove_system(). 
             * The system has to be removed from the collision list and the collision
             * checker.
             * 
             * @copydoc router_t::remove_system()
             */
            virtual void remove_system(const std::string& path);


            /** 
             * An implementation for the system_t::replace_system(). 
             * The system has to be replaced in the collision list and in collision
             * checker.
             * 
             * @copydoc router_t::replace_system(const std::string&, system_ptr_t)
             * 
             */
            virtual void replace_system(const std::string& path, system_ptr_t system);

            /**
             * Updates and stores the new configurations of the obstacles into the \map. 
             * The obstacles are not in the same list as the other systems, for speed up 
             * purposes, so the \ref system_t::update_phys_configs() will not update the 
             * configurations of the obstacles.
             * 
             * @brief Updates the new configurations of the obstacles.
             * 
             * @param map The map that will store the new configurations of the obstacles.
             */
            virtual void update_obstacles_configs(util::config_list_t& map, unsigned& index);

            /**
             * This function adds an obstacle to the simulation and to the 
             * collision checker. Obstacles are stored in different lists in order to 
             * avoid spending time to call the propagation function on them.
             *      
             * @brief Adds an obstacle into the simulation. 
             * 
             * @param name The full path name of the obstacle that we want to add. (eg. simulator/obstacles/box)
             * @param obstacle Is the system pointer for the obstacle that we want to add in 
             * the simulation.
             */
            virtual void add_obstacle(const std::string& name, system_ptr_t obstacle);

            /**
             * This function removes an obstacle from the simulation and the 
             * collision checker. Obstacles are stored in different lists in order to 
             * avoid spending time to call the propagation function on them.
             * 
             * @brief Removes an obstacles from the simulation.
             * 
             * @param name Is the name of the obstacle that we want to remove from
             * the simulator.
             */
            virtual void remove_obstacle(const std::string& name);

            /**
             * Returns a hash with all the obstacles. 
             * 
             * @brief Returns a hash with all the obstacles. 
             * 
             * @return A hash with all the obstacles.
             */
            virtual util::hash_t<std::string, system_ptr_t>& get_obstacles();

            /**
             * It propagates all the systems and responses to the collisions that 
             * happened during the last simulation step. First, sets the \c states and the
             * \c controls for the system and the propagates them. At the end of the 
             * simulation it will response for the collisions.  
             * 
             * @brief Propagates the systems given states and controls. Returns the state of 
             * the systems after the response of the \ref collision_checker_t
             * 
             * @param initial_state The new state for all the systems.
             * @param control The new control for all the systems.
             *      
             * @return The state for the systems after the response of the \ref collision_checker_t.
             */
            const state_t* propagate_and_respond(const state_t * const initial_state, const control_t * const control);

            /**
             * Makes sure that it is a valid simulator after the initialization and 
             * the possible changes. 
             * 
             * @brief Checks if the simulator is valid. 
             * 
             * @exception system_t::invalid_system_exception
             */
            virtual void verify() const;

            /**
             * Function to return a system pointer given a path to that system.
             * 
             * @brief Returns the pointer of the system that we request.
             * 
             * @param system_path A slash-delimited path indicating the location of the system.
             * @return The system pointer.
             */
            virtual system_ptr_t get_system(const std::string& system_path) const;

            /**
             * 
             * @copydoc system_t::get_loader();
             */
            static pluginlib::ClassLoader<simulator_t>& get_loader();

            /**
             * Copies the given \c state to the simulator state. 
             * 
             * @brief Copies the given \c state to the simulator state. 
             * 
             * @param state The state that we want to set the simulation. 
             */
            virtual void push_state(const state_t * const state);

            /** @copydoc route_t::compute_control() */
            virtual void push_control(const control_t * const control);

            /**
             * Calls compute_controls to all the subsystems. 
             * 
             * @brief Calls compute_controls to all the subsystems. 
             */
            virtual void push_control();

            /**
             * Returns the current state of the simulation. 
             * 
             * @brief Returns the current state of the simulation. 
             * 
             * @return The current state of the simulation.  
             */
            virtual state_t* pull_state();
            
            virtual sensing_model_t* get_sensing_model();
            
            /**
             * Called at the end of init, guarantees everything has been set in the simulation, and 
             * can be overwritten for use by special sensing models.
             * 
             * @brief Initializes sensing
             */
            virtual void initialize_sensing();

            /**
             * In planning, the simulator may require more complex state computation when the simulator state is reset (i.e. manipulation).
             * This function helps the world_model know if this is the case or not.
             * 
             * @brief This function helps the world_model know if the simulator is complex.
             * @return [description]
             */
            virtual bool internal_state_push() 
            {
                  return false;
            }
            
            /**
             * Simulator may require to update the collision configurations of obstacles, if their configurations are updated in runtime. 
             * 
             * @brief Update the configurations of obstacles in the collision checker
             * @return void
             */
            virtual void update_obstacles_in_collision_checker();

            void update_obstacle_geoms(util::geom_map_t &geoms) const;

          protected:

            util::config_list_t config_list;

            /** The collision checker that checks for collisions between systems and obstacles*/
            collision_checker_t* collision_checker;

            /** The hash map with all the obstacles in the simulation*/
            util::hash_t<std::string, system_ptr_t> obstacles;

            //This is for resetting state for plants that are in collision.
            system_graph_t sys_graph;

            /** A helping variable that gets the plants from the system graph.*/
            util::hash_t<std::string, plant_t*> plants;

            /** 
             * Correspond to the intervals for each system.
             * For example if we have a system with 3 dimension space, and our simulation
             * has 3 systems then this hash map will be {[0,2],[3,5],[6,8]}. 
             */
            util::hash_t<system_t*, std::pair<unsigned, unsigned> > system_intervals;

            /** 
             * The memory for the current state of the simulator. This includes the states from all
             * the systems, controllers and plants.
             */
            state_t* simulator_state;
            
            /**
             * The a pointer to the application's sensing 
             * 
             * @brief The application's sensing mode
             * 
             */
            sensing_model_t* sensing;
            util::hash_t<std::string, sim::plant_t*> sensed_plants;
            util::geom_map_t sensed_geoms;

            /** @copydoc router_t::set_param(const std::string&, const boost::any&) */
            virtual void set_param(const std::string& parameter_name, const boost::any& value);
            


          private:
            /** The pluginlib loader for the simulator class.*/
            static pluginlib::ClassLoader<simulator_t> loader;

            /** A system clock to mesure the time.*/
            util::sys_clock_t propagation_timer;


        };

        /**
         * Helper function that first sets the given \c state, then calls \ref simulator_t::in_collision().
         * 
         * @brief First sets the given \c state, then calls \ref simulator_t::in_collision().
         * @param state A given state for the systems.
         * @return True if there is at least one collision, False if nothing collided. 
         */
        inline bool simulator_t::in_collision(const state_t * const state)
        {
            push_state(state);
            return in_collision();
        }

        /**
         * Helper function that first sets the given \c state and a specific \c collision_list, then 
         * calls \ref simulator_t::in_collision().
         * 
         * @brief First sets the given \c state and a specific \c collision_list, then calls \ref simulator_t::in_collision().
         * @param state A given state for the systems.
         * @param collision_list A collision list for systems that we want to check for collisions.
         * @return True if there is at least one collision, False if nothing collided. 
         */
        inline bool simulator_t::in_collision(const state_t * const state, collision_list_t* collision_list)
        {
            push_state(state);
            link_collision_list(collision_list);
            return in_collision();
        }

        /**
         * Helper function that first sets the given \c state, then calls \ref simulator_t::get_colliding_bodies().
         * 
         * @brief First sets the given \c state, then calls \ref simulator_t::get_colliding_bodies().
         * @param state A given state for the systems.
         * @return A list with the pairs of the bodies of the systems in collision. 
         */
        inline collision_list_t* simulator_t::get_colliding_bodies(const state_t * const state)
        {
            push_state(state);
            return get_colliding_bodies();
        }

        /**
         * Helper function that first sets the given \c state and a specific \c collision_list, then calls 
         * \ref simulator_t::get_colliding_bodies().
         * 
         * @param state A given state for the systems.
         * @param collision_list A collision list for systems that we want to check for collisions.
         * @return A list with the pairs of the bodies of the systems in collision. 
         */
        inline collision_list_t* simulator_t::get_colliding_bodies(const state_t * const state, collision_list_t* collision_list)
        {
            push_state(state);
            link_collision_list(collision_list);
            return get_colliding_bodies();
        }

        /**
         * A helper function that first sets the given \c state and the given \c control, to the systems, then 
         * calls \ref simulator_t::propagate_and_respond().
         * 
         * @param initial_state A given state for the systems.
         * @param control Given controls for the systems.
         * @return The state for the systems after the response of the \ref collision_checker_t.
         */
        inline const state_t* simulator_t::propagate_and_respond(const state_t * const initial_state, const control_t * const control)
        {
            push_state(initial_state);
            push_control(control);
            propagate_and_respond();
            return pull_state();
        }

    }
}

#endif
