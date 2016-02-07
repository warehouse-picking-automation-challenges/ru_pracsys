/**
 * @file planning_application.hpp 
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
#ifndef PRX_PLANNING_APPLICATION_HPP
#define	PRX_PLANNING_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/planning/planner.hpp"


#include "prx_simulation/query_msg.h"
#include "prx_simulation/graph_msg.h"
#include "prx_simulation/manipulation_acknowledgement.h"
#include "prx_simulation/plant_locations_msg.h"

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace plan
    {

        class world_model_t;
        class task_planner_t;
        class query_msg;
        class state_msg;

        /** @brief Defines the lower and upper intervals in simulator and planning mappings */
        typedef std::pair<unsigned, unsigned> interval_t;

        /** @brief A simulator interval, planning interval pair */
        typedef std::pair<interval_t, interval_t> interval_pair_t;

        /**
         * @anchor planning_application_t
         * 
         * The top-level class which manages communication and the task planners running
         * on a planning node.  The application encapsulates the entire execution of a
         * single planning node.
         * 
         * @brief <b> Top-level planning manager class. </b>
         */
        class planning_application_t
        {

          public:

            planning_application_t();
            virtual ~planning_application_t();

            /** 
             * @brief Initialize the task planner from the given parameters.
             *
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters.
             */
            virtual void init(const util::parameter_reader_t* reader);

            /**
             * @brief Top-level execution function which starts the application loop.
             */
            virtual void execute() = 0;

            /**
             * @brief Converts a simulator state to the planning state.
             * @param state The simulator state.
             * @param c_path The path to the consumer on simulation side.
             * @return The planning state.
             */
            std::vector<double> sim_to_plan_state(const std::vector<double>& state, std::string c_path = "");

            /**
             * @brief Converts a planning state to the simulator state.
             * @param state The planning state.
             * @param c_path The path to the consumer on simulation side.
             * @return The simulator state.
             */
            std::vector<double> plan_to_sim_state(sim::state_t* state, std::string c_path = "");

            /**
             * @brief Converts a simulator control to the planning control.
             * @param state The simulator control.
             * @param c_path The path to the consumer on simulation side.
             * @return The planning control.
             */
            std::vector<double> sim_to_plan_control(const std::vector<double>& control, std::string c_path = "");

            /**
             * @brief Converts a planning control to the simulator control.
             * @param state The planning control.
             * @param c_path The path to the consumer on simulation side.
             * @return The simulator control.
             */
            std::vector<double> plan_to_sim_control(sim::control_t* control, std::string c_path = "");

            /**
             * @brief Call the query processing callback on the provided message.
             * 
             * @param msg The message containing the query to process.
             */
            void process_query(const prx_simulation::query_msg& msg);
            /**
             * @brief Call the ground truth processing callback on the provided message.
             *
             * @param msg The message containing the ground truth information.
             */
            void process_ground_truth(const prx_simulation::state_msg& msg);

            /**
             * @brief Callback which handles new queries to the planning node.
             *
             * @param msg The message containing query information.
             */
            virtual void process_query_callback(const prx_simulation::query_msg& msg);
            /**
             * @brief The callback which updates the planning's ground truth information.
             *
             * @param msg The message containing the ground truth simulation state.
             */
            virtual void process_ground_truth_callback(const prx_simulation::state_msg& msg);
            /**
             * @brief Callback which locates plants.
             *
             * @param msg A message containing what plants to locate.
             */
            virtual void process_plant_locations_callback(const prx_simulation::plant_locations_msg& msg);

            /**
             * @brief Callback which deals with the graph messages.
             *
             * @param msg A message containing a graph that sent from another node.
             */
            virtual void process_graph_callback(const prx_simulation::graph_msg& msg);
            
            /**
             * @brief Callback which deals with the manipulation acknowledgement messages.
             *
             * @param msg A message containing information needed for the manipulation acknowledgement.
             */
            virtual void process_ack_callback(const prx_simulation::manipulation_acknowledgement& msg);

            /**
             * @brief Get the pluginlib class loader for planning node applications.
             *
             * @return The pluginlib class loader for planning applications.
             */
            static pluginlib::ClassLoader<planning_application_t>& get_loader();

            std::string get_simulation_node_name() const;

            bool is_visualizing() const;

            /** @brief A mapping from planning to simulation to correllate systems. */
            util::hash_t<std::string, std::string > planning_to_simulator_sys_mapping; // from input file

          protected:
            /**
             * @brief Internal function for initializing spaces for the given consumer.
             *
             * @param consumer_path The system path to the consumer controller we are building the spaces for.
             */
            virtual void initialize_spaces(std::string consumer_path);

            /** @brief Path to the consumer controller this planning node is responsible for. */
            std::string consumer_path;
            /** @brief The set of specifications to the root task */
            std::vector<specification_t* > root_specifications;
            /** @brief The set of queries to the root task */
            std::vector<query_t* > root_queries;
            /** @brief The world model in which the planning is taking place. */
            world_model_t* model;

            /** @brief The root task to complete */
            task_planner_t* root_task;

            /** @brief A map from consumer paths to that consumer's corresponding state space index interval. */
            util::hash_t< std::string, std::vector<interval_pair_t> > consumer_path_to_state_intervals;
            /** @brief A map from consumer paths to that consumer's corresponding control space index interval. */
            util::hash_t< std::string, std::vector<interval_pair_t> > consumer_path_to_control_intervals;

            /* The following maps allow to go from simulation to planning state, and vice versa.
             * 
             * Normally, it is sufficient to just define a 1-to-1 system mapping, however, in
             * certain cases it is necessary to have different parts of system's state and control map
             * to separate objects in planning
             *  Example:
                   state_mapping:
                   - 
                     pair: [simulator/consumer/vo_controller/disk, world_model/simulator/disk]
                   control_mapping:
                   - 
                     pair: [simulator/consumer/vo_controller, world_model/simulator/disk]

             */

            /** @brief A mapping from simulator to planning to correllate systems. */
            util::hash_t<std::string, std::string > simulator_to_planning_sys_mapping; // from input file

            /** @brief A mapping from simulator to planning to correllate state space intervals. */
            util::hash_t<std::string, std::string > simulator_to_planning_sys_state_mapping; // from input file
            /** @brief A mapping from simulator to planning to correllate control space intervals. */
            util::hash_t<std::string, std::string > simulator_to_planning_sys_control_mapping; // from input file
            /** @brief A mapping from consumer name to space name. */
            util::hash_t<std::string, std::string > consumer_to_space_name_mapping; // from planner_structure input file

            /** @brief Flag for displaying debug information such as planner structures. */
            bool debug;
            /** @brief Flag for sending information over to simulation (false for running planning headless.) */
            bool simulate;
            /** @brief The name of the simulation node that this particular planning node communicates to (if simulate is true) */
            std::string simulation_node_name;
            /** @brief A flag indicating whether the planning should be publishing visualization information. */
            bool visualize;
            /** @brief Use the mapping functions to convert states and controls. Otherwise its an element-wise copy. */
            bool use_sys_mapping;

          private:
            /** @brief The pluginlib class loader for planning applications. */
            static pluginlib::ClassLoader<planning_application_t> loader;

        };

    }
}

#endif	

