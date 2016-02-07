/**
 * @file manipulation_mp_specification.cpp
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

#include "planning/problem_specifications/manipulation_mp_specification.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/world_model.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::manipulation_mp_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace plan;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            manipulation_mp_specification_t::manipulation_mp_specification_t()
            {
                object_space = NULL;
                collision_object_space = NULL;

                graph_deserialization_file = "";
                deserialization_file = "";
                serialization_file = "";
            }

            manipulation_mp_specification_t::~manipulation_mp_specification_t() { }

            void manipulation_mp_specification_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                specification_t::init(reader, template_reader);
            }

            void manipulation_mp_specification_t::link_poses(std::vector< std::pair<unsigned, sim::state_t*> >* poses)
            {
                this->poses = poses;
            }
            
            void manipulation_mp_specification_t::link_query_poses(std::vector< std::pair<unsigned, sim::state_t*> >* poses)
            {
                this->query_poses = poses;
            }                        

            const std::vector< std::pair<unsigned, sim::state_t*> >* manipulation_mp_specification_t::get_poses() const
            {
                return poses;
            }
            
            const std::vector< std::pair<unsigned, sim::state_t*> >* manipulation_mp_specification_t::get_query_poses() const
            {
                return query_poses;
            }
        }
    }
}