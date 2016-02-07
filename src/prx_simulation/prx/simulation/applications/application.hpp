/**
 * @file application.hpp
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

#ifndef PRX_APPLICATION_HPP
#define	PRX_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/system_graph.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/simulation/sensing/sensing_model.hpp"

#include <ros/ros.h>

namespace prx
{
    namespace sim
    {

        class vector_collision_list_t;

        /**
         * An organizer class for our application. It contains the simulator and keeps all the extra 
         * informations that we may need for our application.
         * 
         * @brief <b> An organizer class for our applications. </b>
         * 
         * @authors Andrew Kimmel, Athanasios Krontiris 
         */
        class application_t
        {

          public:
            application_t();
            virtual ~application_t();

            /**
             * Initializes from the given parameters.
             * 
             * @brief Initializes from the given parameters.
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             */
            virtual void init(const util::parameter_reader_t * const reader);
            
            /**
             * Called at the end of init, guarantees everything has been set in the simulation, and 
             * can be overwritten for use by special sensing models.
             * 
             * @brief Initializes sensing
             */
            virtual void initialize_sensing();

            /**
             * Returns True as long as the application is working and stops whenever the user decides to 
             * stop the application.
             * 
             * @brief Executes what the application has to do till the user decides to stop the application.
             * @return True while the application is active.
             *         False when the application stop running.
             */
            virtual bool running();

            /**
             * This function will be called every simulation step. Will execute the main functionality of
             * the application. It will be triggered by the \ref ros::TimerEvent every simulation step. 
             * 
             * @brief This function will execute the main functionality of the application.
             * 
             * @param event A \ref ros::TimerEvent that will be occurred every simulation step.
             */
            virtual void frame(const ros::TimerEvent& event);

            /**
             * A broadcaster for the \ref ros::tf. This function will broadcast the configurations of
             * all the geometries to the tf so the visualization will be able to update the models on 
             * based on their new position. It will be triggered by the \ref ros::TimerEvent every 
             * simulation step. 
             * 
             * @brief This function will broadcast the configurations of all the geometries.
             * 
             * @param event A \ref ros::TimerEvent that will be occurred every simulation step.
             */
            //virtual void tf_broadcasting(const ros::TimerEvent& event);
            
            void tf_broadcasting();

            /**
             * Broadcasts information for the extra geometries that are not the main systems.
             * Most of the time these extra configurations are for debugging/helper geometries that controllers
             * need to visualize. It will be triggered by the \ref ros::TimerEvent every 0.05 seconds.
             * 
             * @brief Broadcasts information for the extra geometries.
             * 
             * @param event A \ref ros::TimerEvent that will be occurred every 0.05 secods.
             */
            virtual void info_broadcasting(const ros::TimerEvent& event);

            /**
             * Broadcasts the geometries information for the extra geometries. These extra geometries
             * are not regular systems. Most of the time these extra geometries are for debugging/helper 
             * geometries that controllers need to visualize.
             * 
             * @brief Broadcasts the geometries information for the extra geometries.
             * 
             * @param event A \ref ros::TimerEvent that will be occurred every 0.05 secods.
             */
            virtual void geom_broadcasting(const ros::TimerEvent& event);

            /**
             * Sets the simulator active/inactive.
             * 
             * @brief Sets the simulator active/inactive.
             * 
             * @param running True if we want to set the simulator active. \n
             *                False if we want to set the simulator inactive.
             */
            void set_running_simulator(bool running);

            /**
             * Handles the input keys from the keyboard. 
             * 
             * @brief Handles the input keys from the keyboard. 
             */
            virtual void handle_key();

            /**
             * Sets the pressed key to the helper variable \c key.
             * 
             * @brief Sets the pressed key to the helper variable \c key.
             * 
             * @param inkey The last pressed key.
             */
            virtual void set_pressed_key(int inkey);

            /**
             * Sets to the helper variable \c selected_path the slash-delimited path for the selected system
             * 
             * @brief Sets to the helper variable \c selected_path the slash-delimited path for the selected system
             * 
             * @param path The full slash-delimited path for the selected system. 
             */
            virtual void set_selected_path(const std::string& path) = 0;

            /**
             * Sets to the helper variable \c selected_point the selected point on the screen. Gives the 
             * arguments as (x,y,z) coordinates. 
             * 
             * @brief Sets to the helper variable \c selected_point the selected point on the screen.
             * 
             * @param x The x coordinate of the selected point.
             * @param y The y coordinate of the selected point.
             * @param z The z coordinate of the selected point.
             */
            virtual void set_selected_point(double x, double y, double z);

            /**
             * Sets the position and the eye of the camera to be in the given points. 
             * 
             * @brief Sets the position and the eye of the camera to be in the given points.
             * 
             * @param new_center The new position of the camera.
             * @param new_eye The new position of the eye of the camera. This is where the 
             * camera will be looking. Towards which point will be recording.
             */
            virtual void set_camera(const util::vector_t& new_center, const util::vector_t& new_eye);
            
            virtual void set_sim_state(const std::vector<double>& new_sim_state);

            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<application_t>& get_loader();

            /**
             * Returns the simulator that this application is using to simulate the systems.
             * 
             * @brief Returns the simulator that this application is using.
             * 
             * @return The simulator that this application is using.
             */
            simulator_t* get_simulator();
            
            /**
             * Informs the user if the application is capable of storing states.
             * 
             * @brief Informs user if application is storing states
             * @return True if the application stores, false otherwise
             */
            bool is_storing_states();
            
            /**
             * Informs the user if the application is capable of replaying states.
             * 
             * @brief Informs user if application is replaying states
             * @return True if the application replays, false otherwise
             */
            bool is_replaying_states();
            
            /**
             * Informs the user if the simulation is visualizing.
             * 
             * @brief Returns visualizing boolean
             * @return True if simulation visualizes, false otherwise
             */
            bool is_visualizing();
            
            
            /**
             * This function gets called when the application is shutdown
             * 
             * @brief Gets called when the application is shutdown
             */
            virtual void shutdown();

          protected:

            /**
             * Read in from input, with a default value of true,
             * this variable determines if the visualization node is
             * running and should be set to false if visualization
             * is not necessary for the simulation to run.
             * 
             * 
             * @brief Determines if visualization is necessary.
             */
            bool visualize;

            /**
             * The simulator that this application is using.
             * 
             * @brief The simulator that this application is using.
             */
            simulator_t* simulator;
            
            /**
             * The a pointer to the simulator's sensing 
             * 
             * @brief The application's sensing mode
             * 
             */
            const sensing_model_t* sensing;

            /**
             * A helper variable to maintain the system graph when any function needs 
             * to compute the system graph.
             * 
             * @brief A helper variable to maintain the system graph.
             */
            system_graph_t sys_graph;

            /**
             * The collision list that this application is using for pairs of systems
             * that need to be checked for collisions.
             * 
             * @brief the current collision list of this application.
             */
            collision_list_t* collision_list;

            /**
             * The whole simulation state. Its a composite state with all the states from
             * all the systems of this application.
             * 
             * @brief A composite state with all the states from all the systems.
             */
            state_t* simulation_state;

            /**
             * The composite simulation state space. Contains all the state spaces from all 
             * the systems.
             * 
             * @brief The composite simulation state space.
             */
            const util::space_t* simulation_state_space;

            /**
             * The whole simulation control. Its a composite control with all the controls from
             * all the systems of this application.
             * 
             * @brief A composite control with all the controls from all the systems.
             */
            control_t* simulation_control;

            /**
             * The composite simulation control space. Contains all the control spaces from all 
             * the systems.
             * 
             * @brief The composite simulation control space.
             */
            const util::space_t* simulation_control_space;

            /**
             * True if the simulator runs. \n
             * False if the simulator is inactive.
             * 
             * @brief True if the simulator runs. False if the simulator is inactive.
             */
            bool simulator_running;

            /**
             * Helper variables for how the simulator is running.
             * @brief Helper variables for how the simulator is running.
             */
            int simulator_counter, simulator_mode;

            /**
             * The last pressed key.
             * @brief The last pressed key.
             */
            int key;

            /**
             * The name of the pressed key.
             * @brief The name of the pressed key.
             */
            std::string sim_key_name;

            /**
             * True if there are new geometries information, False otherwise.
             * @brief True if there are new geometries information, False otherwise.
             */
            bool new_geometries_info;

            /**
             * The full slash-delimited path for the selected system. 
             * @brief The full slash-delimited path for the selected system.
             */
            std::string selected_path;

            /**
             * Helper variable that keeps the previous selected point.
             * @brief Helper variable that keeps the previous selected point.
             */
            util::vector_t previous_selected_point;

            /**
             * Keeps the current selected point from the user input. 
             * @brief Keeps the current selected point from the user input. 
             */
            util::vector_t selected_point;

            /** 
             * Variables for the camera. 
             * @brief Variables for the camera.
             */
            util::vector_t eye, center;

            /**
             * A list with the registered keys that the user can use to control basic stuff during the 
             * simulation (eg. camera, activate/deactivate simulator, etc).
             * 
             * @brief A list with the registered keys for the application.
             */
            std::vector<int> important_keys;

            /**
             * The full slash-delimited paths from all the plants.
             * @brief The full slash-delimited paths from all the plants.
             */
            std::vector<std::string> plant_paths;

            /**
             * A clock to time the loops during the simulation. 
             * @brief A clock to time the loops during the simulation. 
             */
            util::sys_clock_t loop_timer;

            /**
             * Statistical variable to count the total time during the simulation.
             * @brief Statistical variable to count the total time during the simulation.
             */
            double loop_total;

            /**
             * Statistical variable to count the average of the total time during each 
             * simulation step.
             * @brief Statistical variable to count the average of the total time during 
             * each simulation step.
             */
            double loop_avg;

            /**
             * Statistical variable to count how many simulation steps have been executed.
             * @brief Statistical variable to count how many simulation steps have been executed.
             */
            int loop_counter;

            //TODO: Do we want ground truth controlled by application?
            /** @brief The amount of time before another ground truth message is published */
            double ground_truth_time_limit;
            /** @brief Keeps track of time for ground truth publishing*/
            util::sys_clock_t ground_truth_timer;

            /**
             * Initializes the collision list from the parameter reader. It needs a
             * parameter reader, where it will find all the informations that a 
             * collision list needs. 
             * In the input file a black list or a white list can be specified. If none
             * of them is specified then all the systems can collide each other as well 
             * with all the obstacles. 
             * 
             * White list specifies the pairs of the systems (plants or obstacles) that
             * we want to check for collisions. First the system that we want to check 
             * has to be provided and then a list with all the systems that we want to 
             * check for collisions. 
             * (e.g.
             * white_list:
             *  - [car1, [car2, building1, building2]]
             *  - [car2, [building1, building2]]
             * )
             * 
             * Black list specifies the pairs of the systems (plants or obstacles) that
             * we are not interested of checking for collisions. First the system that
             * we want to avoid checking for collisions and then a list with all the systems
             * that we want the systems to collide with. If a pair of systems has been specified 
             * in both white and black list then, the black list overwrite the white list and the 
             * pair will not be in the collision list. An example: 
             * 
             * @code
             * white_list:
             *  - [car1, [car2, building1, building2]]
             *  - [car2, [building1, building2]]
             * black_list:
             *  - [car1, [car2]]
             *  - [car2, [airplane1, airplane2]]
             * @endcode
             * 
             * in this case the car1 will not be checked for collisions with car2
             *      
             * @brief Initializes the collision list from the parameter reader.
             * 
             * @param reader  A \ref util::parameter_reader_t with a dictionary of parameters for this application.
             */
            void init_collision_list(const util::parameter_reader_t * const reader);

            /** 
             * The pluginlib loader for the \ref application_t class.
             * 
             * @brief The pluginlib loader for the \ref application_t class.
             */
            static pluginlib::ClassLoader<application_t> loader;
            
            /**
             * @brief Serializes stored simulation states 
             * @param filename The name of the file that will contain stored simulation states
             */
            void serialize_simulation_states(const std::string& filename);

            /**
             * @brief Deserializes simulation states from a file
             * @param filename The name of the file that contains the simulation states
             * @param stored_sim_states The container structure for the deserialized simulation states
             */
            void deserialize_simulation_states(const std::string& filename, trajectory_t& stored_sim_states);
            
            trajectory_t store_simulation_states;
            trajectory_t replay_simulation_states;
            
            bool screenshot_every_simstep;
            
            bool stores_states, replays_states;
            std::string serialize_sim_file, deserialize_sim_file;

            /**
             * TODO: Document this
             */
            void add_system_self_collision_list( const std::string& system, const vector_collision_list_t* black_list, util::hash_t<std::string, plant_t*>& plants );
        
            /**
             * Adds a pair of systems in the collision list, to be checked for collisions.
             * 
             * @brief Adds a pair of systems in the collision list.
             * 
             * @param system1 The full slash-delimited path for the first system.
             * @param system2 The full slash-delimited path for the second system.
             * @param black_list A black list for potential parts of the systems that have to be out of the list 
             * in order not to be checked for collisions.
             * @param plants A map between the names and the pointers of the plants.
             */
            void add_system_system_in_collision_list(const std::string& system1, const std::string& system2, const vector_collision_list_t* black_list, util::hash_t<std::string, plant_t*>& plants);

            /**
             * Adds a pair of system with obstacle in the collision list to be checked for collisions.
             * 
             * @param system The full slash-delimited path for the system.
             * @param obstacle The full slash-delimited path for the obstacle.
             * @param black_list A black list for potential parts of the systems that have to be out of the list 
             * in order not to be checked for collisions.
             * @param plants A map between the names and the pointers of the plants.
             * @param obstacles A map between the names and the obstalces. 
             */
            void add_system_obstacle_in_collision_list(const std::string& system, const std::string& obstacle, const vector_collision_list_t* black_list, util::hash_t<std::string, plant_t*>& plants, util::hash_t<std::string, system_ptr_t>& obstacles);

            /**
             * If neither white nor black list have been assigned through the parameter reader, during the initialization
             * then in the collision list will be all the plants versus all other plants, and obstacles.
             * 
             * @brief Will build a collision list will all the pairs plant-plant, plant-obstacle.
             * 
             * @param plants The hash map with all the plants.
             * @param obstacles The hash map with all the obstacles.
             * @param black_list A black list for potential parts of the systems that have to be out of the list 
             * in order not to be checked for collisions.
             */
            void collision_list_all_vs_all(util::hash_t<std::string, plant_t*>& plants, util::hash_t<std::string, system_ptr_t>& obstacles, const vector_collision_list_t* black_list);
        };


    }
}

#endif

