/**
 * @file manipulation_sampler.cpp
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

#include "planning/modules/samplers/manip_sampler.hpp"

#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manip_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            manip_sampler_t::manip_sampler_t()
            {
                grasp_z = 0;
                manip_point = NULL;
            }

            manip_sampler_t::~manip_sampler_t()
            {
            }

            void manip_sampler_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                min_theta = parameters::get_attribute_as<double>("min_theta", reader, template_reader, 1);
                max_theta = parameters::get_attribute_as<double>("max_theta", reader, template_reader, 2);
                grasp_z = parameters::get_attribute_as<double>("grasp_z", reader, template_reader, 0);
                max_tries = parameters::get_attribute_as<int>("max_tries", reader, template_reader, 10);

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
                    PRX_WARN_S("Invalid sampling mode for manip_sampler_t!\n Transfer mode will be used.");
                    gripper_mode = GRIPPER_CLOSED;
                }

                //Shouldn't we read this in from input or something maybe?
                relative_configuration.set_position(0, 0, 0);
                relative_configuration.set_orientation(0, -0.707106, 0, 0.707106);
            }

            void manip_sampler_t::link_info(manipulator_plant_t* manip, const space_t* manipulator_state_space, const space_t* object_state_space, const space_t* combined_state_space)
            {
                _manipulator = manip;
                manip_space = manipulator_state_space;
                object_space = object_state_space;
                combined_space = combined_state_space;
                if(manip_point==NULL)
                    manip_point = manip_space->alloc_point();
            }

            void manip_sampler_t::sample(const space_t* space, space_point_t* point)
            {
                //Local configurations FOR APC: not needed
                // config_t tmp_config;
                // config_t effector_config;
                // std::vector< double > config_vector(8);

                //Sample a point
                space->uniform_sample(point);
                //Set the correct gripper mode
                point->memory.back() = gripper_mode;

                //If we are transferring, we also need to set the state of the object for this sample FOR APC: this is not relevant ever. We are going to build roadmaps with no objects to begin with.
                // if( gripper_mode == GRIPPER_CLOSED )
                // {
                //     _manipulator->get_end_effector_configuration(effector_config);

                //     tmp_config = relative_configuration;
                //     tmp_config.relative_to_global( effector_config );

                //     tmp_config.copy_to_vector(config_vector);
                //     object_space->set_from_vector(config_vector);
                // }
                // space->copy_to_point(point);
            }

            void manip_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& bounds, space_point_t* point)
            {
                PRX_FATAL_S("sample_near for manipulation sampler is not implemented!");
            }

            bool manip_sampler_t::sample_near_object(state_t* result_point, const state_t* target_point)
            {
                config_t relative_grasp;
                config_t object_config;
                //Set the object config to the state data
                object_config.set_position( target_point->memory[0], target_point->memory[1], target_point->memory[2] );
                object_config.set_orientation( target_point->memory[3], target_point->memory[4], target_point->memory[5], target_point->memory[6] );

                bool good_grasp = false;
                for( unsigned i=0; i<max_tries && !good_grasp; ++i )
                {
                    //Sample a random angle to grasp the cylinder from
                    double theta = uniform_random(min_theta, max_theta) / 2;
                    // PRX_DEBUG_COLOR("Try: [" << tries << "]   at Angle: " << theta*2 << "  with grasp_z: " << grasp_z, PRX_TEXT_CYAN);

                    //Set the relative grasp to be a rotation about z, as this is just a cylinder grasper
                    relative_grasp.set_position( 0, 0, grasp_z );
                    relative_grasp.set_orientation( 0, 0, sin(theta), cos(theta) );
                    //Going from the object's orientation to the end-effector's orientation
                    quaternion_t relative_orienation = relative_configuration.get_orientation();
                    relative_grasp.set_orientation( relative_grasp.get_orientation() * -relative_orienation );

                    //Compute the global end-effector position with the relative angle
                    relative_grasp.relative_to_global( object_config );

                    good_grasp = _manipulator->IK_solver(relative_grasp, manip_point, true);
                }

                //If we ended with a successful manipulator configuration
                if( good_grasp )
                {
                    //Copy the data from the object and manipulator point
                    object_space->copy_from_point( target_point );
                    manip_space->copy_from_point( manip_point );
                    //Need to set the manipulator state
                    combined_space->copy_to_point( result_point );

                    //DEBUG
                    //DEBUG: get the end-effector configuration
                    //DEBUG
                    config_t ee_config;
                    _manipulator->get_end_effector_configuration( ee_config );

                    //And setup the object config
                    config_t obj_config;
                    obj_config.set_position( target_point->memory[0], target_point->memory[1], target_point->memory[2] );

                    //Then, assert that the difference in pose isn't hueg
                    double rel_dist = obj_config.get_position().distance( relative_grasp.get_position() );
                    double true_dist = ee_config.get_position().distance( relative_grasp.get_position() );

                    if( true_dist > PRX_DISTANCE_CHECK || rel_dist > fabs(grasp_z) + PRX_DISTANCE_CHECK )
                    {
                        //So... some bad things happened here.. let's dig
                        PRX_DEBUG_COLOR("Manipulator at: " << manip_space->print_point( manip_point, 3 ), PRX_TEXT_CYAN );
                        PRX_DEBUG_COLOR("Grasp Z : " << grasp_z, PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("Gives ee_config: " << ee_config.print(), PRX_TEXT_GREEN );
                        PRX_DEBUG_COLOR("For grasping at: " << obj_config.print(), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("Relative grasp: " << relative_grasp.print(), PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("True Dist: " << true_dist << "   Relative Dist: " << rel_dist, PRX_TEXT_LIGHTGRAY);
                        PRX_FATAL_S("DEBUG this stuff.");
                    }
                    // << DEBUG

                    return true;
                }
                return false;
            }

            void manip_sampler_t::set_transit_mode()
            {
                gripper_mode = GRIPPER_OPEN;
            }

            void manip_sampler_t::set_transfer_mode()
            {
                gripper_mode = GRIPPER_CLOSED;
            }

            bool manip_sampler_t::is_transfer_mode()
            {
                return gripper_mode == GRIPPER_CLOSED;
            }

            void manip_sampler_t::set_grasp_z(double in_gz)
            {
                grasp_z = in_gz;
            }
        }
    }
}
