#pragma once
/**
 * @file replanning_controller.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_REPLANNING_CONTROLLER
#define PRX_REPLANNING_CONTROLLER

#include "prx/simulation/systems/controllers/consumer/consumer_controller.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include <list>

namespace prx
{
    namespace util
    {
        class radial_goal_region_t;
    }

    namespace sim
    {

        class replanning_controller_t : public consumer_controller_t
        {

          public:
            /**
             * Initializes the state space of the controller with the parameter time.
             */
            replanning_controller_t();

            /** @copydoc system_t::~system_t() */
            virtual ~replanning_controller_t();

            /** @copydoc system_t::init(const util::parameter_reader_t * const) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** 
             * @copydoc stateful_controller_t::propagate()
             *
             * This function will increment internal state value of time based on
             * the set simulation step for the consumer controller.
             * 
             */
            virtual void propagate(const double simulation_step = 0);

            /** 
             * @copydoc stateful_controller_t::verify()
             *
             * Checks to make sure the simulation step is set to something valid
             *
             */
            virtual void verify() const;

            virtual void compute_control();

            /**
             * Copies the given plan for consumption
             */
            virtual void copy_plan(const plan_t& inplan);


          protected:

            virtual void query_planner();

            // This queue stores the plans computed from all future queries
            plan_t zero_plan;
            std::list <plan_t> queued_plans;

            // Counters to determine which waypoint we are on
            int plan_index;
            state_t* future_end_state;
            double goal_radius;

            // Various flags
            bool got_plan, sent_query;

            util::sys_clock_t query_timer;


        };

    }
}


#endif