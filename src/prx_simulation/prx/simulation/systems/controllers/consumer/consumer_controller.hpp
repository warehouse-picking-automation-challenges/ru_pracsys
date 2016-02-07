/**
 * @file consumer_controller.hpp 
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

#ifndef PRX_CONSUMER_CONTROLLER
#define PRX_CONSUMER_CONTROLLER

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"



namespace prx
{
    namespace util
    {
        class radial_goal_region_t;
    }

    namespace sim
    {

        /**
         * The default controller used to interact with the planning node. This controller
         * is responsible for either receiving (passive) or actively querying (active)
         * plans from the planning node.
         * 
         * Consumer controllers are tied to a single planning node. However, planning nodes
         * are capable of operating over one or many consumer controllers.
         * 
         * For active consumers, the controller operates over one or many goals. The consumer
         * will query for a single goal and then consume the plan to reach that goal, before
         * querying for the next goal.
         * 
         * For passive consumers, the controller waits until it has received a plan from the
         * planning node, after which it will begin to consume controls from the plan. 
         * 
         * Once the plan has been consumed, the controller will enter its contingency
         * plan. Depending on whether or not \c keep_last_control has been set to true,
         * the consumer will either use the most recently consumed control or revert to
         * a default behavior (such as stop moving).
         * 
         * @brief <b> An organizer class for our applications. </b>
         * 
         * @authors Andrew Kimmel
         */
        class consumer_controller_t : public simple_controller_t
        {

          public:
            /**
             * Initializes the state space of the controller with the parameter time.
             * 
             * @brief Initializes the state space of the controller with the parameter time.
             */
            consumer_controller_t();

            /** @copydoc system_t::~system_t() */
            virtual ~consumer_controller_t();

            /**
             * Defines which planning node the consumer is paired to, creates new query
             * publisher and new plan subscriber, and if a plan is saved to a file,
             * loads the plan.
             * 
             * Also does standard controller_t::init()
             * 
             * @brief Initializes the consumer_controller_t 
             * @param reader Used to initialize the controller
             * @param template_reader Used to initialize the controller from template files
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** 
             * @copydoc controller_t::propagate()
             */
            virtual void propagate(const double simulation_step = 0);

            /** 
             * @copydoc simple_controller_t::verify()
             *
             * @note Checks to make sure the controller state space is 1 dimension
             *
             */
            virtual void verify() const;

            /**
             * This function is responsible for computing the control for the controller's
             * subsystem. This is accomplished by consuming a control from the plan. There are
             * three cases that can happen: the controller uses the newly consumed control, or
             * the controller uses the previously consumed control, or the controller uses
             * a control from the contingency plan.
             * 
             * @brief Consumes a control from the plan and gives this control to the subsystem
             */
            virtual void compute_control();

            /**
             * Copies the given plan for consumption. Typically called by planning_comm after
             * communication has received a plan from planning.
             * 
             * @brief  Copies the given plan for consumption
             */
            virtual void copy_plan(const plan_t& inplan);

            /**
             * Retrieves a pointer to the output control space
             * 
             * @brief Retrieves a pointer to the output control space
             * @return A pointer to the output control space
             */
            virtual const util::space_t* get_output_control_space() const;

            /**
             * Returns the controller's subsystem map
             * 
             * @brief Returns the controller's subsystem map
             * @return The controller's subsystem map 
             */
            virtual const util::hash_t<std::string, system_ptr_t >* get_subsystems() const;

          protected:

            /** @brief The memory for the controller's state space member: time*/
            double _time;

            /** @brief The simulation step of the simulator */
            double sim_step;

            /** @brief The plan the controller is consuming */
            plan_t plan;

            /** @brief Contingency plan in case consumer receives no plan (or reach end of plan) */
            plan_t contingency_plan;

            /** @brief The initial state of the consumer's subsystem */
            state_t* init_state;

            /** @brief The name of the file of a serialized plan */
            std::string deserialize_plan;

            /** @brief The most recently consumed control from the plan */
            control_t* last_control;

            /** @brief Determines whether the consumer will continue applying 
             *  the most recent control after consumption of the plan. */
            bool keep_last_control;
            bool keep_last_state;

            /** @brief Determines if planning queries are over homogeneous setups*/
            bool homogeneous_setup;

            /** @brief The name of the planning node this consumer is paired to. */
            std::string planning_node;

            /** @brief Determines whether the consumer actively queries planning and is by default false. */
            bool active_queries;

            bool smooth;

            /** @brief The list of goals the consumer will query plans for (active mode) */
            std::vector<util::radial_goal_region_t*> goal_states;

            /** @brief The index of the currently planned goal */
            int goal_index;
            const util::space_t* child_state_space;
            state_t* get_state, *current_goal_state;
            bool queried_planner;

            virtual void query_planner();

          private:

            void read_in_goals(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader);
        };

    }
}

#endif