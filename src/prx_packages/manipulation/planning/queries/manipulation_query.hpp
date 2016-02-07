/**
 * @file manipulation_query.hpp
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

#ifndef PRX_MANIPULATION_QUERY_HPP
#define	PRX_MANIPULATION_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "planning/modules/pose.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace plan
    {
        class stopping_criteria_t;
    }

    namespace packages
    {
        namespace manipulation
        {

            /**
             * @anchor manipulation_query_t
             *
             * This class represents a problem instance to be solved by a manipulator task planner.
             * It is a self-contained class which holds the starts/goals poses for the objects.
             *
             * @brief <b> General query for manipulation task planners. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulation_query_t : public plan::motion_planning_query_t
            {

              public:

                manipulation_query_t();
                virtual ~manipulation_query_t();

                enum path_mode_t
                {

                    PRX_FULL_PATH, PRX_REACH_PATH, PRX_RETRACT_PATH, PRX_TRANSFER_PATH, PRX_TRANSIT, PRX_TRANSFER
                };

                enum path_quality_t
                {

                    PRX_FIRST_PATH, PRX_BEST_PATH, PRX_ALL_PATHS
                };

                /**
                 * @brief Initialize the planing query from input parameters.
                 *
                 * @param reader The reader for the parameters.
                 * @param template_reader A template reader for reading template parameters 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /** 
                 * @copydoc motion_planning_query_t::clear() 
                 * 
                 * Its also clears the plans that the query has stored.
                 */
                virtual void clear();

                /**
                 * @brief Returns the initial poses for the objects.
                 *
                 * @return The initial poses for the objects.
                 */
                virtual const std::vector<double>& get_initial_pose() const;

                /**
                 * @brief Returns the target poses for the objects.
                 *
                 * @return The target poses for the objects.
                 */
                virtual const std::vector<double>& get_target_pose() const;

                virtual void build_initial_pose(const util::space_t* space);
                virtual void build_target_pose(const util::space_t* space);

                /** @brief Declares the mode for the manipulation planning. Request for a full,reach,retract or transfer path between the poses.*/
                path_mode_t mode;
                /** @brief Declares the quality of the path, */
                path_quality_t path_quality;
                /** @brief The starting pose.*/
                pose_t start_pose;
                /** @brief The final pose.*/
                pose_t target_pose;

                std::vector<sim::plan_t*> plans;
                
                /** @brief Stores the state that the manipulation motion planner will return as goal. */
                sim::state_t* found_goal_state;
                /** @brief Stores the state that the manipulation motion planner will return as start. */
                sim::state_t* found_start_state;      

                /** @brief True if a path is found, False otherwise.*/
                bool found_path;
                

              private:
                std::vector<double> vec_initial_pose;
                std::vector<double> vec_target_pose;
            };
        }
    }
}

#endif

