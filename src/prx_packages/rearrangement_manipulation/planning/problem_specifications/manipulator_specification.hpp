/**
 * @file manipulator_specification.hpp
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

#ifndef PRX_MANIPULATOR_SPECIFICATION_HPP
#define	PRX_MANIPULATOR_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "../../../manipulation/planning/problem_specifications/manipulation_specification.hpp"

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
        }

        namespace rearrangement_manipulation
        {

            /**
             * @anchor manipulator_specification_t
             *
             * 
             * @Author Athanasios Krontiris
             */
            class manipulator_specification_t : public manipulation::manipulation_specification_t
            {

              public:

                manipulator_specification_t();

                virtual ~manipulator_specification_t();

                /**
                 * Prepare the planning specification to be linked to the children
                 * 
                 * @brief Prepare the planning specification to be linked to the children
                 */
                virtual void setup(plan::world_model_t * const model);

                /** @brief Planning context name for the manipulator only.*/
                std::string pc_name_manipulator_only;
                /** @brief Planning context for the object.*/
                std::string pc_name_object_only;
                /** @brief Planning context for the manipulator and the object.*/
                std::string pc_name_manipulator_with_object;
                /** @brief Planning context for the manipulator and an active for collision object.*/
                std::string pc_name_manipulator_with_active_object;
                /** @brief The planning context that is for the transit state (The manipulator and active object).*/
                std::string pc_name_transit_inform;
                /** @brief The planning context that is for the transfer state (The manipulator with plannable object and one more active object.*/
                std::string pc_name_transfer_inform;
                /** @brief Planning context for the transit planning. All objects are active.*/
                std::string pc_name_transit_planning;
                /** @brief Planning context for the transfer planning. All the objects are active and one is plannable inside the manipulator.*/
                std::string pc_name_transfer_planning;

                baxter::manipulator_plant_t* manipulator;
                manipulation::movable_body_plant_t* object;

                /** @brief All the poses that will inform the graph. Id for the pose and the state*/
                std::vector< std::pair<unsigned, sim::state_t*> >* seed_poses;
                /** @brief The new poses that correspond to the initial and final positions.*/
                std::vector< std::pair<unsigned, sim::state_t*> >* query_poses;


                plan::motion_planning_specification_t* transit_graph_specification;
                plan::motion_planning_specification_t* transfer_graph_specification;

              protected:
                /** 
                 * If the specification is informative we need to check if all the data are in place!
                 */
                virtual bool valid();
            };
        }
    }
}

#endif
