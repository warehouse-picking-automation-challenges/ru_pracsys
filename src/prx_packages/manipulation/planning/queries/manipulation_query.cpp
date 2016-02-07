/**
 * @file manipulation_query.cpp
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

#include "planning/queries/manipulation_query.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {

            manipulation_query_t::manipulation_query_t()
            {
                state_space = NULL;
                control_space = NULL;
                mode = PRX_FULL_PATH;
                path_quality = PRX_FIRST_PATH;
            }

            manipulation_query_t::~manipulation_query_t()
            {
                vec_initial_pose.clear();
                vec_target_pose.clear();
                clear();
            }

            void manipulation_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                motion_planning_query_t::init(reader, template_reader);
                std::string mode_str = parameters::get_attribute("mode", reader, template_reader, "");

                if( mode_str == "full_path" )
                    mode = PRX_FULL_PATH;
                else if( mode_str == "reach_path" )
                    mode = PRX_REACH_PATH;
                else if( mode_str == "transfer_path" )
                    mode = PRX_TRANSFER_PATH;
                else if( mode_str == "retract_path" )
                    mode = PRX_RETRACT_PATH;
                else if( mode_str == "transit" )
                    mode = PRX_TRANSIT;
                else if( mode_str == "transfer" )
                    mode = PRX_TRANSFER;
                else
                {
                    PRX_WARN_S("Wrong Input for path mode! Full path is selected!");
                    mode = PRX_FULL_PATH;
                }

                std::string quality_str = parameters::get_attribute("path_quality", reader, template_reader, "");

                if( quality_str == "first_path" )
                    path_quality = PRX_FIRST_PATH;
                else if( quality_str == "best_path" )
                    path_quality = PRX_BEST_PATH;
                else if( quality_str == "all_paths" )
                    path_quality = PRX_ALL_PATHS;
                else
                {
                    PRX_WARN_S("Wrong Input for path quality! First path is selected!");
                    path_quality = PRX_FIRST_PATH;
                }


                if( parameters::has_attribute("initial_pose", reader, template_reader) )
                {
                    vec_initial_pose = parameters::get_attribute_as<std::vector<double> >("initial_pose", reader, template_reader);
                }
                else
                {
                    PRX_WARN_S("No initial pose for the object is given to the manipulation query!");
                }

                if( parameters::has_attribute("target_pose", reader, template_reader) )
                {
                    vec_target_pose = parameters::get_attribute_as<std::vector<double> >("target_pose", reader, template_reader);
                }
                else
                {

                    PRX_WARN_S("No target pose for the object is given to the manipulation query!");
                }
            }

            void manipulation_query_t::clear()
            {
                motion_planning_query_t::clear();

                foreach(plan_t* plan, plans)
                {
                    if( plan != NULL )
                    {
                        plan->clear();
                        delete plan;
                    }
                }
                plans.clear();
            }

            const std::vector<double>& manipulation_query_t::get_initial_pose() const
            {

                return vec_initial_pose;
            }

            const std::vector<double>& manipulation_query_t::get_target_pose() const
            {

                return vec_target_pose;
            }

            void manipulation_query_t::build_initial_pose(const util::space_t* space)
            {
                start_pose.state = space->alloc_point();
                space->copy_vector_to_point(vec_initial_pose, start_pose.state);
            }

            void manipulation_query_t::build_target_pose(const util::space_t* space)
            {
                target_pose.state = space->alloc_point();
                space->copy_vector_to_point(vec_target_pose, target_pose.state);
            }
        }
    }
}
