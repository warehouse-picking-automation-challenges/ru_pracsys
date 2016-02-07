/**
 * @file time_varying_local_planner.hpp
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

#ifndef PRX_TIME_VARYING_LOCAL_PLANNER_HPP
#define	PRX_TIME_VARYING_LOCAL_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @anchor time_varying_local_planner_t
         *
         * Performs local planning, but uses bounds to search for paths within trajectory
         * length windows, producing a variety of paths.
         *
         * @brief <b> Performs local planning with the ability to change the trajectory length. </b>
         *
         * @author Zakary Littlefield
         */
        class time_varying_local_planner_t : public local_planner_t
        {

          public:

            time_varying_local_planner_t() : local_planner_t(){ }
            /**
             * @copydoc local_planner_t::init()
             */
            virtual void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t * const template_reader);

            virtual ~time_varying_local_planner_t(){ }
            /**
             * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::trajectory_t& , bool connect)
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::trajectory_t& traj, bool connect = true);
            /**
             * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::state_t*, bool connect )
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::state_t* result, bool connect = true);

          protected:
            /** @brief Storage for computed trajectories. */
            sim::trajectory_t* trajs;
            /** @brief Storage for computed plans. */
            sim::plan_t* plans;
            /** @brief The number of controls to try when producing new trajectories. */
            int num_controls;
            /** @brief Lower bound on simulation steps to take when generating a trajectory. */
            int lower_multiple;
            /** @brief Upper bound on simulation steps to take when generating a trajectory. */
            int upper_multiple;
        };

    }
}

#endif
