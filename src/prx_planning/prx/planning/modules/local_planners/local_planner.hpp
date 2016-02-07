/**
 * @file local_planner.hpp 
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

#ifndef PRX_LOCAL_PLANNER_HPP
#define PRX_LOCAL_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {
        class space_t;
        class distance_metric_t;        
    }
    
    namespace plan
    {

        class sampler_t;
        class world_model_t;
        

        /**
         * @anchor local_planner_t
         *
         * Abstract class which aims to create local optimal trajectories between two
         * states.  
         *
         * @brief <b> Local optimal trajectory planner. </b>
         *
         * @author Zakary Littlefield
         */
        class local_planner_t
        {

          public:

            local_planner_t() : world_model(NULL){ }

            virtual ~local_planner_t(){ }

            /**
             * @brief Local planner initialization method, which loads appropriate parameters.
             *
             * @param reader A \ref util::parameter_reader_t containing the parameters for this local planner.
             * @param template_reader A \ref util::parameter_reader_t containing default parameters for local planners.
             */
            virtual void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t * const template_reader = NULL);

            /**
             * This propagate step will only return the result of the plan's controls.
             * 
             * @param start The start state of the propagation.
             * @param plan A plan containing the to-be-simulated controls
             * @param result The resulting state. 
             */
            virtual void propagate_step(const sim::state_t* start, const sim::plan_t& plan, sim::state_t* state);

            /**
             * Tries to connect the start and goal.
             * 
             * @param start The start state of the propagation
             * @param goal The goal state that the planner will try to reach
             * @param plan The resulting plan that achieved that.
             * @param traj The trajectory produced by the plan.
             * @param connect Flag determining if a connection should be attempted (default true)
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::trajectory_t& traj, bool connect = true) = 0;

            /**
             * Tries to connect the start and goal.
             * 
             * @param start The start state of the propagation
             * @param goal The goal state that the planner will try to reach
             * @param plan The resulting plan that achieved that.
             * @param result The state produced by the plan.
             * @param connect Flag determining if a connection should be attempted (default true)
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::state_t* result, bool connect = true) = 0;

            /**
             * This propagate will return the trajectory that the plan creates.
             * 
             * @param start The start state of the propagation.
             * @param plan A plan containing the to-be-simulated controls
             * @return The resulting trajectory.
             */
            virtual void propagate(const sim::state_t* start, const sim::plan_t& plan, sim::trajectory_t& traj);

            /**
             * @param step The new step to use.
             */
            void set_duration_step(double step);

            /**
             * @param in_sampler The sampler to link to this local planner.
             */
            void link_sampler(sampler_t* in_sampler);

            /**
             * @param dist_metric The distance metric to link to this local planner.
             */
            void link_metric(util::distance_metric_t* dist_metric);

            /**
             * @param model The world model to link to this local planner.
             */
            virtual void link_model(world_model_t* model);

            /**
             * @param space The state space to link to this local planner.
             */
            virtual void link_state_space(const util::space_t* space);

            /**
             * @param space The control space to link to this local planner.
             */
            virtual void link_control_space(const util::space_t* space);

            /**
             * @brief Retrieves the local planner class loader.
             *
             * @return The local planner class loader.
             */
            static pluginlib::ClassLoader<local_planner_t>& get_loader();

          protected:
            /** @brief The state space over which the local planner is operating. */
            const util::space_t* state_space;

            /** @brief The control space over which the local planner is operating. */
            const util::space_t* control_space;

            /** @brief The world model the local planner is operating in. */
            world_model_t* world_model;

            /** @brief The sampler used by the local planner. */
            sampler_t* sampler;

            /** @brief The distance metric used by the local planner. */
            util::distance_metric_t* metric;

            /** @brief The duration step the local planner uses for creating local plans. */
            double duration_step;

          private:
            /** @brief The pluginlib class loader for local planner objects. */
            static pluginlib::ClassLoader<local_planner_t> loader;

        };

    }
}

#endif