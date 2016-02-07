/**
 * @file manipulation_specification.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors:Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_REARRANGEMENT_MANIPULATION_SPECIFICATION_HPP
#define PRX_REARRANGEMENT_MANIPULATION_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class distance_metric_t;
    }

    namespace plan
    {
        class world_model_t;
        class sampler_t;
        class validity_checker_t;
        class local_planner_t;
    }

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * @anchor rearrangement_manipulation_specification_t
             *
             * 
             * @Author Athanasios Krontiris
             */
            class rearrangement_manipulation_specification_t : public plan::specification_t
            {

              public:

                rearrangement_manipulation_specification_t();

                virtual ~rearrangement_manipulation_specification_t();

                /**
                 * @brief Initialize the specification from input parameters.
                 *
                 * @param reader The reader for the parameters.
                 * @param template_reader A template reader for reading template parameters 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * @brief Clear the plan and trajectory of this query.
                 *
                 * Clears the plan and the trajectory of the planning query in order to 
                 * reuse the planning query with the same start and stopping criterion. 
                 */
                virtual void clear();

                /**
                 * Prepare the planning specification to be linked to the children
                 * 
                 * @brief Prepare the planning specification to be linked to the children
                 */
                virtual void setup(plan::world_model_t * const model);

                /**
                 * @brief Links the state and control spaces that the planning query is working over.
                 *
                 * Links the state and control spaces that the planning query is working over.
                 * 
                 * @param state_space The state space.
                 * @param control_space The control space.
                 */
                virtual void link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space);

                /**
                 * @brief Returns the initial poses for the objects.
                 *
                 * @return The initial poses for the objects.
                 */
                virtual const std::vector< std::vector<double> >* get_initial_poses() const;

                /**
                 * @brief Returns the target poses for the objects.
                 *
                 * @return The target poses for the objects.
                 */
                virtual const std::vector< std::vector<double> >* get_target_poses() const;

                /**
                 * @brief Returns the extra poses for the objects.
                 *
                 * @return The extra poses for the objects.
                 */
                virtual const std::vector< std::vector<double> >* get_extra_poses() const;

                /**
                 * Returns the total number of all the poses in this query.
                 * 
                 * @return The total number of all the poses in this query.
                 */
                virtual unsigned total_poses() const;

                /**
                 * Returns the total number of the initial, the goal and the extra poses. 
                 * 
                 * @return The total number of the initial, the goal and the extra poses. 
                 */
                virtual unsigned query_poses() const;

                /**
                 * Returns the total number of both the initial and goal poses. 
                 * 
                 * @return The total number of both the initial and goal poses. 
                 */
                virtual unsigned important_poses() const;


                util::distance_metric_t* transfer_metric;
                /** @brief The z coordinate that the object will be on a surface. */
                double z_on_table;
                /** @brief The max tries to grasp a pose*/
                int max_tries;
                /** @brief The different ways to grasp a pose*/
                unsigned max_different_grasps;
                /** @brief The distance that the manipulator will move after leaving the cup on the ground */
                double retract_distance;
                /** @brief The number poses that will be added in the graph*/
                unsigned num_poses;
                /** @brief The safe position where the manipulator will return without colliding with anything */
                std::vector<double> safe_position;
                /** @brief k cups in the problem.*/
                unsigned _k;
                /** @brief b extra poses in the RPG.*/
                unsigned _b;
                /** @brief The failures that are allowed for constructing an MPG.*/
                int max_random_failures;
                /** @brief The bias towards the goal poses of the MPG.*/
                int goal_biasing;
                /** @brief The time limit for the application to find a solution.*/
                double time_limit;
                /** @brief The file where the poses are stored.*/
                std::string poses_file;
                /** @brief the file where the informed transit graph is stored.*/
                std::string transit_graph_file;
                /** @brief the file where the informed transfer graph is stored.*/
                std::string transfer_graph_file;
                /** @brief A flag if we want the task planner to read information from a file or build them by itself.*/
                bool deserialize_flag;

                /** @brief A flag that will tell us if we want to smooth the path or not.*/
                bool apply_smoothing;

              protected:
                /** @brief Vector containing all possible initial poses in double form */
                std::vector< std::vector<double> > initial_poses;
                /** @brief Vector containing all possible target poses in double form */
                std::vector< std::vector<double> > target_poses;
                /** @brief Vector containing all extra poses in double form */
                std::vector< std::vector<double> > extra_poses;
            };
        }
    }
}

#endif
