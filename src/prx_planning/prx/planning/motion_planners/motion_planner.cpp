/**
 * @file motion_planner.hpp 
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

#include "prx/planning/motion_planners/motion_planner.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/simulation/control.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;

    using namespace util::parameters;

    namespace plan
    {

        motion_planner_t::motion_planner_t()
        {
            input_specification = NULL;
            input_query = NULL;
            statistics = NULL;
            deserialize_flag = serialize_flag = false;
        }

        motion_planner_t::~motion_planner_t() { }

        void motion_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            planner_t::init(reader, template_reader);
            if( has_attribute("serialize_file", reader, template_reader) )
            {
                PRX_DEBUG_S("Serialization variable set");
                serialization_file = get_attribute_as<std::string > ("serialize_file", reader, template_reader);
                serialize_flag = true;
            }
            else
            {
                PRX_DEBUG_S("Serialization set to false");
                serialize_flag = false;
            }

            if( has_attribute("deserialize_file", reader, template_reader) )
            {
                PRX_DEBUG_S("Deerialization variable set");
                deserialization_file = get_attribute_as<std::string > ("deserialize_file", reader, template_reader);
                deserialize_flag = true;
            }
            else
            {
                PRX_DEBUG_S("Deserialization set to false");
                deserialize_flag = false;
            }
        }

        void motion_planner_t::link_world_model(world_model_t * const model) { }

        void motion_planner_t::link_specification(specification_t* new_spec)
        {
            input_specification = (motion_planning_specification_t*)new_spec;
            state_space = input_specification->state_space;
            control_space = input_specification->control_space;

            if( input_specification->local_planner != NULL )
                local_planner = input_specification->local_planner;
            else
                PRX_FATAL_S("No local planner has been specified in motion planner " << path);

            if( input_specification->validity_checker != NULL )
                validity_checker = input_specification->validity_checker;
            else
                PRX_FATAL_S("No validity checker has been specified in motion planner " << path);

            if( input_specification->sampler != NULL )
                sampler = input_specification->sampler;
            else
                PRX_FATAL_S("No sampler has been specified in motion planner " << path);

            if( input_specification->metric != NULL )
                metric = input_specification->metric;
            else
                PRX_FATAL_S("No distance metric has been specified in motion planner " << path);
        }

        void motion_planner_t::link_query(query_t* new_query)
        {
            input_query = (motion_planning_query_t*)new_query;
            PRX_ASSERT(input_specification != NULL);
            input_specification->get_stopping_criterion()->link_goal(input_query->get_goal());
        }

        double motion_planner_t::compute_cost()
        {
            resolve_query();
            return validity_checker->trajectory_cost(input_query->path);
        }

        bool motion_planner_t::execute()
        {
            PRX_ASSERT(input_specification != NULL);
            do
            {
                // PRX_INFO_S("EXECUTE FROM MOTION PLANNER");
                step();
            }
            while( !input_specification->get_stopping_criterion()->satisfied() );
                        
            return succeeded();
        }

        bool motion_planner_t::serialize()
        {
            PRX_ERROR_S("Serialize called on motion planner: Empty implementation!");
            return false;
        }

        bool motion_planner_t::can_serialize()
        {
            return serialize_flag;
        }

        bool motion_planner_t::deserialize()
        {
            PRX_ERROR_S("Deserialize called on motion planner: Empty implementation!");
            return false;
        }

        bool motion_planner_t::can_deserialize()
        {
            return deserialize_flag;
        }

        const statistics_t* motion_planner_t::get_statistics()
        {
            return statistics;
        }

        void motion_planner_t::set_param(const std::string& path, const std::string& parameter_name, const boost::any& value)
        {
            if( !path.empty() )
                PRX_FATAL_S("A path leading to motion planner is invalid. Passed path is " << path << " while this motion planner has path " << this->path);
            this->set_param(parameter_name, value);
        }

        void motion_planner_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            PRX_DEBUG_COLOR("Motion Planner set_param : " << parameter_name, PRX_TEXT_MAGENTA);
            if( parameter_name == "serialize_flag" )
                serialize_flag = boost::any_cast<bool>(value);
            else if( parameter_name == "deserialize_flag" )
                deserialize_flag = boost::any_cast<bool>(value);
            else if( parameter_name == "serialization_file" )
                serialization_file = boost::any_cast<std::string>(value);
            else if( parameter_name == "deserialization_file" )
                deserialization_file = boost::any_cast<std::string>(value);
            else
                planner_t::set_param(parameter_name, value);
        }
    }
}
