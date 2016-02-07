/**
 * @file rigid_body_plant.cpp
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

#include "movable_body_plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/plan.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::movable_body_plant_t, prx::sim::system_t)

namespace prx
{
    namespace packages
    {
        using namespace util;
        using namespace sim;

        namespace manipulation
        {
            movable_body_plant_t::movable_body_plant_t() : plant_t()
            {
                state_memory = boost::assign::list_of(&_x)(&_y)(&_z)(&_qx)(&_qy)(&_qz)(&_qw);

                state_space = new space_t("SE3", state_memory);
                input_control_space = new space_t("EMPTY", control_memory);
            }

            void movable_body_plant_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                plant_t::init(reader, template_reader);
            }

            void movable_body_plant_t::push_state(const state_t * const state)
            {
                state_space->copy_from_point(state);
            }

            void movable_body_plant_t::move_object( const config_t& end_effector_config, const config_t& relative_config )
            {
                //Transform the configuration based on the manipulator configuration
                root_config = relative_config;
                root_config.relative_to_global( end_effector_config );
                //Store the result into the state
                root_config.get_position(_x, _y, _z);
                root_config.get_orientation().get(_qx, _qy, _qz, _qw);
            }

            void movable_body_plant_t::relative_config_with_manipulator(const util::config_t& end_effector_config, util::config_t& relative_config )
            {
                relative_config.set_position(_x, _y, _z);
                relative_config.set_xyzw_orientation(_qx, _qy, _qz, _qw);
                relative_config.global_to_relative( end_effector_config );
            }


            void movable_body_plant_t::propagate(const double simulation_step)
            {
            }

            void movable_body_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                root_config.set_position(_x, _y, _z);
                root_config.set_xyzw_orientation(_qx, _qy, _qz, _qw);
                plant_t::update_phys_configs(configs, index);
            }

            void movable_body_plant_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
            {
            }

            void movable_body_plant_t::get_configuration( config_t& body_config ) const
            {
                body_config.set_position(_x, _y, _z);
                body_config.set_xyzw_orientation(_qx, _qy, _qz, _qw);
            }

            void movable_body_plant_t::set_configuration( const config_t& body_config )
            {
                root_config = body_config;
            }

            void movable_body_plant_t::print_configuration( ) const
            {
                PRX_PRINT( "Object configuration: " << root_config, PRX_TEXT_GREEN );
            }

        }
    }
}

