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

#ifndef PRX_MANIPULATION_SPECIFICATION_HPP
#define	PRX_MANIPULATION_SPECIFICATION_HPP

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
        class astar_module_t;
        class motion_planning_specification_t;
    }

    namespace packages
    {
        namespace baxter
        {
            class manipulator_plant_t;
        }

        namespace manipulation
        {
            class manip_sampler_t;
            class movable_body_plant_t;

            /**
             * @anchor manipulation_specification_t
             *
             * 
             * @Author Athanasios Krontiris
             */
            class manipulation_specification_t : public plan::specification_t
            {

              public:

                manipulation_specification_t();

                virtual ~manipulation_specification_t();

                /**
                 * @brief Initialize the specification from input parameters.
                 *
                 * @param reader The reader for the parameters.
                 * @param template_reader A template reader for reading template parameters 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * Links the state and control spaces for the transfer points. The spaces for the transit graph
                 * are linked through the main \c link_spaces function.
                 *
                 * @brief Links the state and control spaces for the transfer points.
                 * 
                 * @param state_space The state space for the transfer graph.
                 * @param control_space The control space for the transfer graph.
                 */
                virtual void link_extra_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space);

                /**
                 * Clears the seeds from the specification. 
                 * 
                 * @brief Clears the seeds from the specification. 
                 *                  
                 */
                void clear_seeds();

                /**
                 * Add seeds for the transit and transfer graphs.
                 * 
                 * @brief Add seeds for the transit and transfer graphs.
                 * 
                 * @param transit_seed The seed for the transit graph.
                 * @param transfer_seed The seed for the transfer graph.
                 */
                void add_seeds(const sim::state_t* transit_seed, const sim::state_t* transfer_seed);

                /**
                 * Adds a seed for the transit graph.
                 * 
                 * @brief Adds a seed for the transit graph.
                 * 
                 * @param state The seed for the transit graph.
                 */
                void add_transit_seed(const sim::state_t* state);

                /**
                 * Adds a seed for the transfer graph.
                 * 
                 * @brief Adds a seed for the transfer graph.
                 * 
                 * @param state The seed for the transfer graph.
                 */
                void add_transfer_seed(const sim::state_t* state);

                /**
                 * Gets the seeds for the transit graph.
                 * 
                 * @brief Gets the seeds for the transit graph.
                 * 
                 * @return The vector with the seeds for the transit graph.
                 */
                std::vector<sim::state_t*>& get_transit_seeds();

                /**
                 * Gets the seeds for the transfer graph.
                 * 
                 * @brief Gets the seeds for the transfer graph.
                 * 
                 * @return The vector with the seeds for the transfer graph.
                 */
                std::vector<sim::state_t*>& get_transfer_seeds();

                /** @brief The max tries to grasp a pose*/
                int max_tries;
                /** @brief The different ways to grasp a pose*/
                unsigned max_different_grasps;
                /** @brief The distance that the manipulator will move after leaving the cup on the ground */
                double retract_distance;
                /** @brief The safe position where the manipulator will return without colliding with anything */
                std::vector<double> safe_position;

                /** The sampler in case we need to sample grasps for a pose.*/
                manip_sampler_t* manip_sampler;
                
                /** @brief The extra astar module for the second motion planner.*/
                plan::astar_module_t* transfer_astar;

              private:
                
                /** @brief State space over which the transfer graph is operating. */
                const util::space_t* transfer_state_space;
                /** @brief Control space over which the transfer graph is operating. */
                const util::space_t* transfer_control_space;

                std::vector<sim::state_t*> transit_seeds;
                std::vector<sim::state_t*> transfer_seeds;
            };
        }
    }
}

#endif
