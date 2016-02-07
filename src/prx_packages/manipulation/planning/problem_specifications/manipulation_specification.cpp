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

#include "planning/problem_specifications/manipulation_specification.hpp"
#include "planning/modules/samplers/manip_sampler.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

#include "simulation/plants/movable_body_plant.hpp"
#include "../../../baxter/simulation/plants/manipulator.hpp"

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            manipulation_specification_t::manipulation_specification_t()
            {
                safe_position.clear();
            }

            manipulation_specification_t::~manipulation_specification_t()
            {
                clear();
            }

            void manipulation_specification_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                specification_t::init(reader, template_reader);
                max_tries = parameters::get_attribute_as<unsigned>("max_tries", reader, template_reader, 0);
                max_different_grasps = parameters::get_attribute_as<unsigned>("max_different_grasps", reader, template_reader, 0);
                retract_distance = parameters::get_attribute_as<double>("retract_distance", reader, template_reader, 0);

                if( parameters::has_attribute("manipulation_sampler", reader, template_reader) )
                {
                    sampler_t* tmp_sampler = parameters::initialize_from_loader< sampler_t > ("prx_planning", reader, "manipulation_sampler", template_reader, "manipulation_sampler");
                    manip_sampler = dynamic_cast<manip_sampler_t*>( tmp_sampler );
                    PRX_DEBUG_COLOR("Initializing manip spec, sampler: " << tmp_sampler << " : " << manip_sampler, PRX_TEXT_GREEN );
                }
                else
                {
                    PRX_WARN_S("No manipulation_sampler attribute in manipulation specification!");
                }

                if( parameters::has_attribute("safe_position", reader, template_reader) )
                    safe_position = parameters::get_attribute_as<std::vector<double> >("safe_position", reader, template_reader);
                else
                    PRX_WARN_S("No safe position is specified for the manipulation problem!");
            }

            void manipulation_specification_t::link_extra_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space)
            {
                transfer_state_space = new_state_space;
                transfer_control_space = new_control_space;
            }

            void manipulation_specification_t::add_transit_seed(const sim::state_t* state)
            {
                transit_seeds.push_back(state_space->clone_point(state));
            }

            void manipulation_specification_t::add_transfer_seed(const sim::state_t* state)
            {
                transfer_seeds.push_back(transfer_state_space->clone_point(state));
            }

            void manipulation_specification_t::clear_seeds()
            {
                for( unsigned i = 0; i < transit_seeds.size(); ++i )
                    state_space->free_point(transit_seeds[i]);
                transit_seeds.clear();

                for( unsigned i = 0; i < transfer_seeds.size(); ++i )
                    transfer_state_space->free_point(transfer_seeds[i]);
                transfer_seeds.clear();
            }

            void manipulation_specification_t::add_seeds(const sim::state_t* transit_seed, const sim::state_t* transfer_seed)
            {
                add_transit_seed(transit_seed);
                add_transfer_seed(transfer_seed);
            }

            std::vector<sim::state_t*>& manipulation_specification_t::get_transit_seeds()
            {
                return transit_seeds;
            }

            std::vector<sim::state_t*>& manipulation_specification_t::get_transfer_seeds()
            {
                return transfer_seeds;
            }
        }
    }
}
