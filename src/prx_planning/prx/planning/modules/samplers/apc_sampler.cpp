/**
 * @file apc_sampler.cpp
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

#include "prx/planning/modules/samplers/apc_sampler.hpp"

#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::apc_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        apc_sampler_t::apc_sampler_t()
        {
            manip_space = NULL;
            manip_point = NULL;
            IK_base = NULL;
        }

        apc_sampler_t::~apc_sampler_t() 
        {
            manip_space->free_point(manip_point);
        }

        void apc_sampler_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            theta_error = parameters::get_attribute_as<int>("theta_error", reader, template_reader, 10);
            max_tries = parameters::get_attribute_as<int>("max_tries", reader, template_reader, 10);
            neighs = parameters::get_attribute_as<int>("neighbors", reader, template_reader, 50);

            std::string mode = parameters::get_attribute("mode", reader, template_reader, "transit");
            if( mode == "transit" )
            {
                gripper_mode = GRIPPER_OPEN;
            }
            else if( mode == "transfer" )
            {
                gripper_mode = GRIPPER_CLOSED;
            }
            else
            {
                PRX_WARN_S("Invalid sampling mode for apc_sampler_t!\n Transfer mode will be used.");
                gripper_mode = GRIPPER_CLOSED;
            }

            //Shouldn't we read this in from input or something maybe?
            relative_configuration.set_position(0, 0, 0);
            relative_configuration.set_orientation(0, 0.707106, 0, 0.707106);
        }

        void apc_sampler_t::link_info(prx::packages::baxter::manipulator_plant_t* manip, const util::vector_t& min_bounds, const util::vector_t& max_bounds, IK_data_base_t* IK_data_base)
        {
            PRX_PRINT("Linking info to apc sampler < " << this << " >", PRX_TEXT_BROWN);
            _manipulator = manip;
            manip_space = manip->get_state_space();
            IK_base = IK_data_base;
            this->min_bounds = min_bounds;
            this->max_bounds = max_bounds;
            if( manip_point == NULL )
                manip_point = manip_space->alloc_point();
        }

        void apc_sampler_t::sample(const space_t* space, space_point_t* point)
        {
            //Sample a point
            space->uniform_sample(point);
            //Set the correct gripper mode
            point->memory[15] = 0;
            point->memory.back() = gripper_mode;
        }

        bool apc_sampler_t::safe_sample(const space_t* space, space_point_t* point)
        {
            //Sample a point
            space->uniform_sample(point);
            //Set the correct gripper mode
            point->memory[15] = 0;
            point->memory.back() = gripper_mode;
            // PRX_PRINT("space: " << space->get_space_name() << "        manip_space:" << manip_space->get_space_name(), PRX_TEXT_BROWN);
            // if( space == manip_space )
            // {
            //     PRX_PRINT("I am in to check for trusted points", PRX_TEXT_BROWN);
            manip_space->copy_from_point(point);
            _manipulator->get_end_effector_configuration(effector_config);
            vector_t effector_pos = effector_config.get_position();
            if( effector_pos > min_bounds && effector_pos < max_bounds )
                return false;
            return true;
            // }
            // return false;
        }

        bool apc_sampler_t::sample_inside(state_t* result_point, const util::vector_t& min_bounds, const util::vector_t& max_bounds)
        {
            config_t grasp_config;

            for( unsigned i = 0; i < max_tries; ++i )
            {
                grasp_config.set_position_at(0, uniform_random(min_bounds[0], max_bounds[0]));
                grasp_config.set_position_at(1, uniform_random(min_bounds[1], max_bounds[1]));
                grasp_config.set_position_at(2, uniform_random(min_bounds[2], max_bounds[2]));

                double angle = uniform_random(-theta_error, theta_error);
                double qx = sin(angle);
                double qy = cos(angle);
                double qz = 0;

                angle = uniform_random(1.57 - theta_error, 1.57 + theta_error) / 2.0;
                qx = qx * sin(angle);
                qy = qy * sin(angle);
                qz = qz * sin(angle);
                double qw = cos(angle);

                grasp_config.set_orientation(qx, qy, qz, qw);

                if( IK_base != NULL )
                {
                    if(IK_base->has_data())
                    {
                        std::vector< sim::state_t* > states;
                        IK_base->get_near_neighbors( states , grasp_config, 1);
                        manip_space->copy_point( manip_point, states[0] );
                    }
                    else
                    {
                        // PRX_WARN_S("Linked IK database has no data.");
                        manip_space->uniform_sample(manip_point);
                    }
                }
                else
                    manip_space->uniform_sample(manip_point);

                if( _manipulator->IK_solver(grasp_config, result_point, gripper_mode, manip_point) )
                {
                    result_point->memory[15] = 0;
                    return true;
                }
            }
            return false;
        }

        void apc_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& bounds, space_point_t* point)
        {
            PRX_FATAL_S("sample_near for manipulation sampler is not implemented!");
        }

        bool apc_sampler_t::sample_near_object(state_t* result_point, const state_t* target_point)
        {
            PRX_FATAL_S("sample_near object for apc sampler is not implemented YET!");
            
            return false;
        }

        void apc_sampler_t::set_transit_mode()
        {
            gripper_mode = GRIPPER_OPEN;
        }

        void apc_sampler_t::set_transfer_mode()
        {
            gripper_mode = GRIPPER_CLOSED;
        }

        bool apc_sampler_t::is_transfer_mode()
        {
            return gripper_mode == GRIPPER_CLOSED;
        }


    }
}
