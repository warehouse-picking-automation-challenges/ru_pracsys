/**
 * @file apc_local_planner.hpp
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

#ifndef PRX_APC_LOCAL_PLANNER_HPP
#define	PRX_APC_LOCAL_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"
#include "simulation/plants/manipulator.hpp"

namespace prx
{
    namespace plan
    {
        class validity_checker_t;

        /**
         * @anchor apc_local_planner_t
         *
         * This class exactly connects samples within the state space.  It does so by
         * employing solutions to the boundary value problem, which must be provided
         * by the underlying world model.
         *
         * @brief <b> Local planner which solves the boundary value problem. </b>
         *
         * @author Athanasios Krontiris
         */
        class apc_local_planner_t : public bvp_local_planner_t
        {

          public:
            apc_local_planner_t();
            virtual ~apc_local_planner_t();

            /**
             * @copydoc local_planner_t::init()
             */
            virtual void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t * const template_reader = NULL);
            
            void link_info(prx::packages::baxter::manipulator_plant_t* manip, const util::vector_t& min_bounds, const util::vector_t& max_bounds);
            
            /**
             * Checks 
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::trajectory_t& traj, bool connect = true);  
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan,sim::state_t* result, bool connect = true);
            virtual bool safe_steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::trajectory_t& traj);
            virtual bool IK_steering_general( util::config_t& start_config, util::config_t& end_config, sim::trajectory_t& path, sim::plan_t& plan,  validity_checker_t* validity_checker, bool grasping, bool camera_link );
            
          protected:

            // The pointers to the actual manipulator plant
            prx::packages::baxter::manipulator_plant_t* _manipulator;
            util::vector_t min_bounds;
            util::vector_t max_bounds;
            
            util::space_point_t* prev_st;
            util::space_point_t* inter_st;

            sim::plan_t temp_plan;
            sim::trajectory_t temp_traj;
        };

    }
}

#endif	

