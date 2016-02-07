/**
 * @file manipulation_specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/problem_specifications/rearrangement_manipulation_specification.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/world_model.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::rearrangement_manipulation_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace plan;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            rearrangement_manipulation_specification_t::rearrangement_manipulation_specification_t() { }

            rearrangement_manipulation_specification_t::~rearrangement_manipulation_specification_t() { }

            void rearrangement_manipulation_specification_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                specification_t::init(reader, template_reader);

                if( parameters::has_attribute("transfer_metric", reader, template_reader) )
                {
                    transfer_metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "transfer_metric", template_reader, "transfer_metric");
                }
                else
                {
                    PRX_WARN_S("Missing transfer_metric attribute in planning specification!");
                }

                z_on_table = parameters::get_attribute_as<double>("z_on_table", reader, template_reader, 0);
                max_tries = parameters::get_attribute_as<unsigned>("max_tries", reader, template_reader, 20);
                max_different_grasps = parameters::get_attribute_as<unsigned>("max_different_grasps", reader, template_reader, 3);
                retract_distance = parameters::get_attribute_as<double>("retract_distance", reader, template_reader, 0.1);
                num_poses = parameters::get_attribute_as<unsigned>("num_poses", reader, template_reader, 10);
                max_random_failures = parameters::get_attribute_as<int>("max_random_failures", reader, template_reader, 10);
                goal_biasing = parameters::get_attribute_as<int>("goal_biasing", reader, template_reader, 5);
                time_limit = parameters::get_attribute_as<double>("time_limit", reader, template_reader, 1800);

                poses_file = parameters::get_attribute("poses_file", reader, template_reader, "poses.txt");
                transit_graph_file = parameters::get_attribute("transit_graph_file", reader, template_reader, "transit_graph.txt");
                transfer_graph_file = parameters::get_attribute("transfer_graph_file", reader, template_reader, "transfer_graph.txt");
                deserialize_flag = parameters::get_attribute_as<bool>("deserialize", reader, template_reader, true);
                apply_smoothing = parameters::get_attribute_as<bool>("apply_smoothing", reader, template_reader, true);

                if( parameters::has_attribute("k", reader, template_reader) )
                    _k = parameters::get_attribute_as<unsigned>("k", reader, template_reader);
                else
                    PRX_FATAL_S("The number of cups in an RPG (k) is not specified!");

                if( parameters::has_attribute("b", reader, template_reader) )
                    _b = parameters::get_attribute_as<unsigned>("b", reader, template_reader);
                else
                    PRX_FATAL_S("The number of extra poses in an RPG (b) is not specified!");

                if( parameters::has_attribute("safe_position", reader, template_reader) )
                    safe_position = parameters::get_attribute_as<std::vector<double> >("safe_position", reader, template_reader);
                else
                    PRX_FATAL_S("No safe position is specified for the problem!");

                if( parameters::has_attribute("initial_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("initial_poses", reader, template_reader))
                    {

                        initial_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }
                else
                {
                    PRX_WARN_S("Missing initial poses for the objects from rearrangement manipulation specification!");
                }

                if( parameters::has_attribute("target_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("target_poses", reader, template_reader))
                    {

                        target_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }
                else
                {
                    PRX_WARN_S("Missing target poses for the objects from rearrangement manipulation specification!");
                }

                if( parameters::has_attribute("extra_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("extra_poses", reader, template_reader))
                    {

                        extra_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }
            }

            void rearrangement_manipulation_specification_t::setup(world_model_t * const model)
            {
                specification_t::setup(model);
            }

            void rearrangement_manipulation_specification_t::clear()
            {
                specification_t::clear();
            }

            void rearrangement_manipulation_specification_t::link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space)
            {
                this->state_space = new_state_space;
                this->control_space = new_control_space;
            }

            const std::vector< std::vector<double> >* rearrangement_manipulation_specification_t::get_initial_poses() const
            {
                return &initial_poses;
            }

            const std::vector< std::vector<double> >* rearrangement_manipulation_specification_t::get_target_poses() const
            {
                return &target_poses;
            }

            const std::vector< std::vector<double> >* rearrangement_manipulation_specification_t::get_extra_poses() const
            {
                return &extra_poses;
            }

            unsigned rearrangement_manipulation_specification_t::total_poses() const
            {
                return initial_poses.size() + target_poses.size() + extra_poses.size() + num_poses;
            }

            unsigned rearrangement_manipulation_specification_t::query_poses() const
            {
                return initial_poses.size() + target_poses.size() + extra_poses.size();
            }

            unsigned rearrangement_manipulation_specification_t::important_poses() const
            {
                return initial_poses.size() + target_poses.size();
            }
        }
    }
}