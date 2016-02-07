/**
 * @file urdf_plant.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Rahul Shome, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/plants/baxter_arm.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include "prx/utilities/definitions/string_manip.hpp"

#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/param.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::baxter::baxter_arm_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        namespace baxter
        {

            baxter_arm_t::baxter_arm_t() : manipulator_plant_t()
            {
                // PRX_DEBUG_COLOR("Building the general Baxter Arm.", PRX_TEXT_RED);
                joints_ = new double[7];
                controls_ = new double[7];
                state_memory = boost::assign::list_of(&joints_[0]) (&joints_[1]) (&joints_[2]) (&joints_[3]) (&joints_[4]) (&joints_[5]) (&joints_[6]) (&gripper_);
                control_memory = boost::assign::list_of(&controls_[0]) (&controls_[1]) (&controls_[2]) (&controls_[3]) (&controls_[4]) (&controls_[5]) (&controls_[6]) (&gripper_control);
                state_space = new space_t("Baxter_Arm_Left", state_memory);
                input_control_space = new space_t("Baxter_Arm_Left", control_memory);

                //For now, this is hard-coded, but we should be able to parameterize this based on what gripper we are using.
                wrist_to_end_effector_distance = 0.05;
                _is_left_handed = true;
            }

            baxter_arm_t::~baxter_arm_t()
            {
                delete[] joints_;
                delete[] controls_;

                state_space->free_point(prev_st);
                state_space->free_point(inter_st);
            }

            void baxter_arm_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                prev_st = state_space->alloc_point();
                inter_st = state_space->alloc_point();

                manipulator_plant_t::init(reader, template_reader);

                MAX_IK_STEP = parameters::get_attribute_as<double>("max_ik_step", reader, template_reader, 0.01);

                colliding_bodies.push_back("left_upper_shoulder");
                colliding_bodies.push_back("left_lower_shoulder");
                colliding_bodies.push_back("left_upper_elbow");
                colliding_bodies.push_back("left_lower_elbow");
                colliding_bodies.push_back("left_upper_forearm");
                colliding_bodies.push_back("left_lower_forearm");
                colliding_bodies.push_back("left_wrist");
                colliding_bodies.push_back("left_hand");
                colliding_bodies.push_back("left_gripper");
                colliding_bodies.push_back("left_gripper_left_finger");
                colliding_bodies.push_back("left_gripper_right_finger");
                colliding_bodies.push_back("end_effector");
                max_step = parameters::get_attribute_as<double>("max_step", reader, template_reader, 0.05);

                double forward_transform_rotation[9];
                double forward_transform_translation[3];
                double inverse_transform_rotation[9];
                double inverse_transform_translation[3];

                if( parameters::has_attribute("forward_transform_rotation", reader, template_reader) && parameters::has_attribute("forward_transform_translation", reader, template_reader) )
                {
                    std::vector<double> fwd_tform = parameters::get_attribute_as< std::vector<double> >("forward_transform_rotation", reader, template_reader);

                    if( fwd_tform.size() < 9 )
                    {
                        PRX_FATAL_S("Rotation matrix needs to have 9 entries");
                    }
                    for( unsigned i = 0; i < 9; ++i )
                    {
                        forward_transform_rotation[i] = fwd_tform[i];
                    }

                    if( parameters::has_attribute("inverse_transform_rotation", reader, template_reader) )
                    {
                        fwd_tform = parameters::get_attribute_as< std::vector<double> >("inverse_transform_rotation", reader, template_reader);
                        if( fwd_tform.size() < 9 )
                        {
                            PRX_FATAL_S("Rotation matrix needs to have 9 entries");
                        }
                        for( unsigned i = 0; i < 9; ++i )
                        {
                            inverse_transform_rotation[i] = fwd_tform[i];
                        }
                    }
                    else
                    {
                        inverse_transform_rotation[0]=forward_transform_rotation[0];
                        inverse_transform_rotation[1]=forward_transform_rotation[3];
                        inverse_transform_rotation[2]=forward_transform_rotation[6];
                        inverse_transform_rotation[3]=forward_transform_rotation[1];
                        inverse_transform_rotation[4]=forward_transform_rotation[4];
                        inverse_transform_rotation[5]=forward_transform_rotation[7];
                        inverse_transform_rotation[6]=forward_transform_rotation[2];
                        inverse_transform_rotation[7]=forward_transform_rotation[5];
                        inverse_transform_rotation[8]=forward_transform_rotation[8];
                    }

                    fwd_tform = parameters::get_attribute_as< std::vector<double> >("forward_transform_translation", reader, template_reader);
                    if( fwd_tform.size() < 3 )
                    {
                        PRX_FATAL_S("Translation vector needs to have 3 entries");
                    }
                    for( unsigned i = 0; i < 3; ++i )
                    {
                        forward_transform_translation[i] = fwd_tform[i];
                    }

                    if( parameters::has_attribute("inverse_transform_translation", reader, template_reader) )
                    {
                        fwd_tform = parameters::get_attribute_as< std::vector<double> >("inverse_transform_translation", reader, template_reader);
                        if( fwd_tform.size() < 3 )
                        {
                            PRX_FATAL_S("Translation vector needs to have 3 entries");
                        }
                        for( unsigned i = 0; i < 3; ++i )
                        {
                            inverse_transform_translation[i] = fwd_tform[i];
                        }
                    }
                    else
                    {
                        inverse_transform_translation[0]=(inverse_transform_rotation[0]*forward_transform_translation[0]*(-1))+(inverse_transform_rotation[1]*forward_transform_translation[1]*(-1))+(inverse_transform_rotation[2]*forward_transform_translation[2]*(-1));
                        inverse_transform_translation[1]=(inverse_transform_rotation[3]*forward_transform_translation[0]*(-1))+(inverse_transform_rotation[4]*forward_transform_translation[1]*(-1))+(inverse_transform_rotation[5]*forward_transform_translation[2]*(-1));
                        inverse_transform_translation[2]=(inverse_transform_rotation[6]*forward_transform_translation[0]*(-1))+(inverse_transform_rotation[7]*forward_transform_translation[1]*(-1))+(inverse_transform_rotation[8]*forward_transform_translation[2]*(-1));
                    }
                    kinematic_model.set_static_transform(forward_transform_translation[0], forward_transform_translation[1], forward_transform_translation[2], forward_transform_rotation);
                    kinematic_model.set_static_inverse_transform(inverse_transform_translation[0], inverse_transform_translation[1], inverse_transform_translation[2], inverse_transform_rotation);
                }
                else
                {
                    PRX_FATAL_S("No transforms given for Baxter system!");
                }

                std::string _hand = parameters::get_attribute_as<string>("hand", reader, template_reader, "unset");
                if( _hand == "left" )
                {
                    kinematic_model.set_hand_state( kinematic_model.LEFT );
                    _is_left_handed = true;
                }
                else if( _hand == "right" )
                {
                    kinematic_model.set_hand_state( kinematic_model.RIGHT );
                    _is_left_handed = false;
                }
                else
                {
                    PRX_FATAL_S("Input gave an erroneous hand.");
                }

                effector_name = pathname + "/end_effector";
                grasped_ignored.push_back(pathname + "/left_gripper_left_finger");
                grasped_ignored.push_back(pathname + "/left_gripper_right_finger");
            }

            void baxter_arm_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                config_t linkcfg;
                double result[7];

                //Then, for each visible geometry (shouldn't we care about collidable too?)
                for( unsigned i = 0; i < colliding_bodies.size(); i++ )
                {
                    //Do the forward kinematics for that link
                    kinematic_model.fk_for_link(colliding_bodies[i], joints_, gripper_, result);

                    //Set the position and orientation for it based on the FK solution and the global offset
                    linkcfg.set_position( result[0], result[1], result[2] );
                    linkcfg.set_orientation( result[3], result[4], result[5], result[6] );

                    //Ensure there is an entry in the list for this configuration
                    augment_config_list(configs,index);
                    //Assign the values
                    configs[index].first = pathname + "/" + colliding_bodies[i];
                    configs[index].second = linkcfg;
                    ++index;
                }
            }

            void baxter_arm_t::get_end_effector_configuration(config_t& effector_config)
            {
                double pose[7];
                kinematic_model.fk_for_link("end_effector", joints_, gripper_, pose);

                //Why does the FK handle orientation but not position...?
                effector_config.set_position(pose[0], pose[1], pose[2]);
                effector_config.set_orientation(pose[3], pose[4], pose[5], pose[6]);
            }


            bool baxter_arm_t::IK_solver(const config_t& effector_config, space_point_t* computed_state, bool set_grasping, const space_point_t* seed_state, bool do_min)
            {
                config_t wrist_configuration;
                quaternion_t tmp_orient;
                config_t transformed_config = effector_config;
                double soln[7];

                //Baxter arm should be applying end-effector offset here as well, correct?
                wrist_configuration.set_position( 0, 0, -wrist_to_end_effector_distance );
                wrist_configuration.set_orientation( 0, 0, 0, 1 );
                wrist_configuration.relative_to_global( transformed_config );

                //Get the robot state in vector form
                std::vector< double > baxter_state;
                if( seed_state != NULL )
                    state_space->copy_point_to_vector( seed_state, baxter_state );
                else
                    state_space->copy_to_vector( baxter_state );

                bool success = false;
                if( do_min )
                    success = kinematic_model.min_ik_for_position(wrist_configuration, baxter_state, soln);
                else
                    success = kinematic_model.ik_for_position(wrist_configuration, baxter_state, soln);

                //DO DEBUGGING HERE THEN?
                // config_t wrist_center;
                // wrist_center.set_position( 0, 0, -0.025 );
                // wrist_center.set_orientation( 0, 0, 0, 1 );
                // wrist_center.relative_to_global( wrist_configuration );
                // PRX_DEBUG_COLOR("IK for wrist center: " << wrist_center.print(), PRX_TEXT_BLUE);

                //Check for failure
                if( !success )
                    return false;

                //Copy over the IK solution
                std::vector<double> state_vec = boost::assign::list_of(soln[0])(soln[1])(soln[2])(soln[3])(soln[4])(soln[5])(soln[6]);
                state_vec.push_back( set_grasping ? (double)GRIPPER_CLOSED : GRIPPER_OPEN );
                state_space->copy_vector_to_point( state_vec, computed_state );

                return true;
            }


            bool baxter_arm_t::is_grasping() const
            {
                return (gripper_ == 1);
            }


            bool baxter_arm_t::IK_steering( const config_t& start_config, const config_t& goal_config, sim::plan_t& result_plan, bool set_grasping )
            {
                //Begin by determining a distance between the start and goal configurations
                double distance = start_config.get_position().distance( goal_config.get_position() );
                //Set the number of interpolation steps
                double steps = std::ceil(distance / MAX_IK_STEP);
                //Clear out the plan we were given, we will be filling it in ourselves
                result_plan.clear();
                //Keep around an intermediate plan for systematically building up the solution
                plan_t intermediate_plan = result_plan;
                //A storage vector for resulting states
                std::vector< double > result_state_vec(8);

                //Store the arm's current state as the previous state
                state_space->copy_to_point(prev_st);
                //So first, let's do the IK for the start configuration
                if( !IK_solver( start_config, prev_st, set_grasping, prev_st, true ) )
                {
                    PRX_DEBUG_COLOR("IK failed in Steering for the start config.", PRX_TEXT_RED);
                    return false;
                }

                //Storage for the interpolated configuration
                config_t interpolated_config;

                //Then, for each step
                for( double i=1; i<=steps; ++i )
                {
                    //Compute the next interpolated configuration
                    double t = PRX_MINIMUM( (i/steps), 1.0 );
                    interpolated_config.interpolate( start_config, goal_config, t );

                    //Compute Minimum IK for interpolated configuration (Pose, seed state, soln)
                    if( IK_solver( interpolated_config, inter_st, set_grasping, prev_st, true ) )
                    {
                        //Create a plan by steering between states and add it to the resulting plan
                        intermediate_plan.clear();
                        steering_function(prev_st, inter_st, intermediate_plan);
                        result_plan += intermediate_plan;
                        //And make sure we store this state as our prior state
                        state_space->copy_point(prev_st, inter_st);
                    }
                    //If the MinIK fails, then the steering is a failure, abort
                    else
                    {
                        //Clear out whatever plan we had started making
                        PRX_DEBUG_COLOR("IK failed in Steering", PRX_TEXT_BROWN);
                        result_plan.clear();
                        return false;
                    }
                }

                //Report our Success
                return true;
            }

            void baxter_arm_t::append_contingency(plan_t& result_plan, double duration)
            {
                result_plan.link_control_space(input_control_space);
                state_space->copy_to_point( state );
                result_plan.copy_onto_back(state, duration-result_plan.length());
            }

            bool baxter_arm_t::is_left_handed()
            {
                return _is_left_handed;
            }
        }
    }
}






