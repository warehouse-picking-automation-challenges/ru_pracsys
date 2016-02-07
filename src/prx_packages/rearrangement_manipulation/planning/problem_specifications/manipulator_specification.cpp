/**
 * @file manipulation_specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/problem_specifications/manipulator_specification.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

#include "../../../manipulation/simulation/plants/movable_body_plant.hpp"
#include "../../../baxter/simulation/plants/manipulator.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::manipulator_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace plan;

    namespace packages
    {
        using namespace manipulation;

        namespace rearrangement_manipulation
        {

            manipulator_specification_t::manipulator_specification_t()
            {
                pc_name_manipulator_only = "";
                pc_name_object_only = "";
                pc_name_manipulator_with_object = "";
                pc_name_manipulator_with_active_object = "";
                pc_name_transit_planning = "";
                pc_name_transfer_planning = "";

                manipulator = NULL;
                object = NULL;

                transit_graph_specification = NULL;
                transfer_graph_specification = NULL;

                safe_position.clear();
            }

            manipulator_specification_t::~manipulator_specification_t() { }

            void manipulator_specification_t::setup(world_model_t * const model)
            {
                specification_t::setup(model);

                if( !valid() )
                    PRX_FATAL_S("The specification for manipulation is informative but does not contains all the information!");

            }

            bool manipulator_specification_t::valid()
            {
                bool is_valid = true;
                if( manipulator == NULL )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the manipulator plan!");
                }

                if( object == NULL )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the object plan!");
                }

                if( safe_position.size() == 0 )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the safe position!");
                }

                if( pc_name_manipulator_only == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the pc_name_manipulator_only!");
                }

                if( pc_name_object_only == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the pc_name_object_only!");
                }

                if( pc_name_manipulator_with_object == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the pc_name_manipulator_with_object!");
                }

                if( pc_name_manipulator_with_active_object == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the pc_name_manipulator_with_active_object!");
                }

                if( pc_name_transit_planning == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the pc_name_transit_planning!");
                }

                if( pc_name_transfer_planning == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The manipulation specification is informative but does not include the pc_name_transfer_planning!");
                }

                if( max_tries == 0 )
                {
                    is_valid = false;
                    PRX_WARN_S("The max_tries variable is not specified in manipulation specification !");
                }

                if( max_different_grasps == 0 )
                {
                    is_valid = false;
                    PRX_WARN_S("The max_different_grasps variable is not specified in manipulation specification !");
                }

                if( retract_distance == 0 )
                {
                    PRX_WARN_S("The retract_distance variable is 0 in manipulation specification !");
                }

                return is_valid;
            }
        }
    }
}