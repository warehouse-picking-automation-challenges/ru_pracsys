///**
// * @file simple_planning_manipulator.cpp 
// *
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// *
// * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
// *
// * Email: pracsys@googlegroups.com
// */
//
//
//#include "simple_planning_manipulator.hpp"
//
//#include "prx/utilities/spaces/space.hpp"
//#include "prx/simulation/plan.hpp"
//
//#include <pluginlib/class_loader.h>
//#include <boost/bind.hpp>
//#include <boost/assign/list_of.hpp>
//#include <pluginlib/class_list_macros.h>
//#include <cmath>
//
//PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::simple_planning_manipulator_plant_t, prx::sim::system_t)
//
//namespace prx
//{
//    using namespace util;
//    using namespace sim;
//
//    namespace packages
//    {
//        namespace manipulation
//        {
//
//
//            const unsigned simple_planning_manipulator_plant_t::VAR_X = 0;
//            const unsigned simple_planning_manipulator_plant_t::VAR_Y = 1;
//            const unsigned simple_planning_manipulator_plant_t::VAR_Z = 2;
//            const unsigned simple_planning_manipulator_plant_t::VAR_TX = 3;
//            const unsigned simple_planning_manipulator_plant_t::VAR_TY = 4;
//            const unsigned simple_planning_manipulator_plant_t::VAR_TZ = 5;
//            const unsigned simple_planning_manipulator_plant_t::VAR_G = 6;
//
//            simple_planning_manipulator_plant_t::simple_planning_manipulator_plant_t() : simple_manipulator_plant_t() { }
//
//            simple_planning_manipulator_plant_t::~simple_planning_manipulator_plant_t() { }
//
//            void simple_planning_manipulator_plant_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
//            {
//                manipulator_plant_t::init(reader, template_reader);
//                effector_name = pathname + "/" + parameters::get_attribute_as<std::string > ("effector_name", reader, template_reader);
//                right_finger_name = pathname + "/" + parameters::get_attribute_as<std::string > ("right_finger_name", reader, template_reader);
//                left_finger_name = pathname + "/" + parameters::get_attribute_as<std::string > ("left_finger_name", reader, template_reader);
//                left_side_name = pathname + "/" + parameters::get_attribute_as<std::string > ("left_side_name", reader, template_reader);
//                right_side_name = pathname + "/" + parameters::get_attribute_as<std::string > ("right_side_name", reader, template_reader);
//
//                for( size_t i = 0; i < relative_config_map.size(); ++i )
//                {
//                    if( relative_config_map[i].first == effector_name )
//                    {
//                        end_effector_relative_config = &relative_config_map[i].second;
//                    }
//                    if( relative_config_map[i].first == right_finger_name )
//                    {
//                        rfinger_config = &relative_config_map[i].second;
//                    }
//                    else if( relative_config_map[i].first == left_finger_name )
//                    {
//                        lfinger_config = &relative_config_map[i].second;
//                    }
//                    else if( relative_config_map[i].first == right_side_name )
//                    {
//                        rside_config = &relative_config_map[i].second;
//                    }
//                    else if( relative_config_map[i].first == left_side_name )
//                    {
//                        lside_config = &relative_config_map[i].second;
//                    }
//                }
//
//                double tmp_x, tmp_z;
//                max_grasp = parameters::get_attribute_as<double>("max_grasp", reader, template_reader);
//                end_effector_position = parameters::get_attribute_as<double>("end_effector_position", reader, template_reader);
//                lside_config->get_position(&side_x, &side_y, &tmp_z);
//                lfinger_config->get_position(&tmp_x, &finger_y, &tmp_z);
//                side_grasp_y = side_y - max_grasp;
//                finger_grasp_y = finger_y - max_grasp;
//                //                PRX_ERROR_S("side_x: " << side_x << "   side_y: " << side_y << "  finger_y: " << finger_y << "   side_grasp: " << side_grasp_y << "  finger_grasp: " << finger_grasp_y);
//
//                max_step = parameters::get_attribute_as<double>("max_step", reader, template_reader);
//
//                initial_state = state_space->alloc_point();
//                state = state_space->alloc_point();
//                prior_state = state_space->alloc_point();
//                hold_state = state_space->alloc_point();
//
//                control = input_control_space->alloc_point();
//                prior_control = input_control_space->alloc_point();
//
//                dist = 2;
//                interpolation_step = 0;
//                reset = false;
//            }
//
//            void simple_planning_manipulator_plant_t::propagate(const double simulation_step)
//            {
//                state_space->copy_to_point(state);
//                //                PRX_INFO_S("manipulator state : " << state_space->print_memory(2));
//                //                PRX_INFO_S("root state : " << state_space->print_memory(2));
//                //                state_t* keep_state = IK_solver(end_effector_config);
//                //                PRX_WARN_S("IK   state : " << state_space->print_point(keep_state,2));
//                input_control_space->copy_to_point(control);
//                prev_g = state->memory[VAR_G];
//
//                if( state_space->equal_points(state, hold_state, PRX_ZERO_CHECK) )
//                    interpolation_step = 0;
//                else
//                    reset = true;
//
//                if( (!input_control_space->equal_points(control, prior_control, PRX_ZERO_CHECK)) || reset )
//                {
//                    input_control_space->copy_point(prior_control, control);
//
//                    state_space->copy_point(initial_state, state);
//
//                    interpolation_step = max_step / state_space->distance(initial_state, control);
//
//                    dist = 0;
//                    reset = false;
//                }
//
//                dist += interpolation_step;
//
//                if( dist <= 1 )
//                {
//                    state_space->copy_to_point(hold_state);
//                    state_space->interpolate(initial_state, control, dist, state);
//                    state->memory[VAR_G] = control->memory[VAR_G];
//                    state_space->copy_from_point(state);
//
//                }
//                else
//                {
//                    state_space->copy_from_point(control);
//                }
//
//                //Store previous control and the now current state.
//                state_space->copy_point(prior_state, state);
//                input_control_space->copy_point(prior_control, control);
//            }
//
//            void simple_planning_manipulator_plant_t::update_phys_configs(config_list_t& configs) const
//            {
//                double place = configs.size();
//                root_config.set_position(_x, _y, _z);
//                root_config.set_xyzw_orientation(sin(_tx / 2.0), 0.0, 0.0, cos(_tx / 2.0));
//                tmp_orient.set(0.0, sin(_ty / 2.0), 0.0, cos(_ty / 2.0));
//                root_config.set_orientation(root_config.get_orientation() * tmp_orient);
//                tmp_orient.set(0.0, 0.0, sin(_tz / 2.0), cos(_tz / 2.0));
//                root_config.set_orientation(root_config.get_orientation() * tmp_orient);
//                configs.push_back(config_list_element_t(root_geom, root_config));
//
//                if( prev_g != _g )
//                {
//                    if( _g == 1 )
//                    {
//                        rfinger_config->set_position(side_x, -finger_grasp_y, 0);
//                        lfinger_config->set_position(side_x, finger_grasp_y, 0);
//                        rside_config->set_position(side_x, -side_grasp_y, 0);
//                        lside_config->set_position(side_x, side_grasp_y, 0);
//                    }
//                    else
//                    {
//                        rfinger_config->set_position(side_x, -finger_y, 0);
//                        lfinger_config->set_position(side_x, finger_y, 0);
//                        rside_config->set_position(side_x, -side_y, 0);
//                        lside_config->set_position(side_x, side_y, 0);
//                    }
//                }
//
//                plant_t::update_phys_configs(configs);
//                //                PRX_WARN_S("effector[pos] : " << configs[place + end_effector_position].first << "    effector_name : " << effector_name);
//                PRX_ASSERT(configs[place + end_effector_position].first == effector_name);
//                end_effector_config = configs[place + end_effector_position].second;
//
//
//            }
//
//            void simple_planning_manipulator_plant_t::get_effectors_name(std::vector<std::string>& names) const
//            {
//                names.push_back(right_finger_name);
//                names.push_back(left_finger_name);
//            }
//
//            bool simple_planning_manipulator_plant_t::is_grasping() const
//            {
//                return state->memory[VAR_G] == 1;
//            }
//
//            void simple_planning_manipulator_plant_t::get_end_effector_position(std::vector<double>& pos)
//            {
//                end_effector_config.get(tmp_pos, tmp_orient);
//
//                for( int i = 0; i < 3; ++i )
//                    pos[i] = tmp_pos[i];
//
//                for( int i = 0; i < 4; ++i )
//                    pos[3 + i] = tmp_orient[i];
//                return;
//            }
//
//            //            void simple_planning_manipulator_plant_t::get_end_effector_position(std::vector<double>& pos)
//            //            {
//            //                tmp_config = *end_effector_config;
//            //                tmp_config.get(tmp_pos, tmp_orient);
//            //                //                PRX_WARN_S("relative effector pos : " << tmp_pos[0] << "," << tmp_pos[1] << "," << tmp_pos[2]);
//            //                tmp_pos[0] += relative_grasp_position;
//            //                //                PRX_WARN_S("relative object   pos : " << tmp_pos[0] << "," << tmp_pos[1] << "," << tmp_pos[2]);
//            //                tmp_config.set_position(vector_t(root_config.get_orientation().qv_rotation(tmp_pos)));
//            //                //                root_config.print();
//            //                //                tmp_config.print();
//            //                tmp_config = root_config + tmp_config;
//            //                //                PRX_INFO_S("afer sum: ");
//            //                //                tmp_config.print();
//            //                tmp_config.get(tmp_pos, tmp_orient);
//            //
//            //                for( int i = 0; i < 3; ++i )
//            //                    pos[i] = tmp_pos[i];
//            //
//            //                for( int i = 0; i < 4; ++i )
//            //                    pos[3 + i] = tmp_orient[i];
//            //
//            //                //                tmp_config.print();
//            //                //                PRX_INFO_S("manipulator state:  " << state_space->print_memory(2));
//            //
//            //
//            //                //                PRX_INFO_S("root_config: " << state_space->print_memory(2));
//            //                //                tmp_orient = root_config.get_orientation();
//            //                //                PRX_WARN_S("IK : " << state_space->print_point(IK_solver(tmp_config,tmp_orient)));
//            //                //                tmp_config.print();
//            //                //                root_config.print();
//            //                //                PRX_ERROR_S("effector config : " << pos[0] << "," << pos[1] << "," << pos[2]);
//            //                return;
//            //            }
//
//            void simple_planning_manipulator_plant_t::IK_solver(config_t& effector_config, std::vector<double>& state_vec)
//            {
//                tmp_pos = end_effector_relative_config->get_position();
//                tmp_pos = -tmp_pos;
//                tmp_orient = effector_config.get_orientation();
//
//                tmp_pos = tmp_orient.qv_rotation(tmp_pos);
//                tmp_pos += effector_config.get_position();
//
//                state_vec[VAR_X] = tmp_pos[0];
//                state_vec[VAR_Y] = tmp_pos[1];
//                state_vec[VAR_Z] = tmp_pos[2];
//
//
//                tmp_orient.convert_to_euler(tmp_pos);
//                state_vec[VAR_TX] = tmp_pos[0];
//                state_vec[VAR_TY] = tmp_pos[1];
//                state_vec[VAR_TZ] = tmp_pos[2];
//
//                state_vec[VAR_G] = _g;
//
//                //                for(int i = 0 ; i < VAR_G; ++i)
//                //                    PRX_INFO_S("val :  " << state_vec[i]);
//            }
//
//            void simple_planning_manipulator_plant_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
//            {
//                PRX_WARN_S("steering a simple manipulators " << pathname);
//                state_space->copy_to_point(state);
//                state_space->copy_from_point(start);
//                config_list_t configs;
//                update_phys_configs(configs);
//                config_t start_config = end_effector_config;
//
//                configs.clear();
//                state_space->copy_from_point(goal);
//                update_phys_configs(configs);
//                config_t target_config = end_effector_config;
//
//                configs.clear();
//                state_space->copy_from_point(state);
//                update_phys_configs(configs);
//
//                for( double t = 0; t <= 1; t += max_step )
//                {
//                    start_config.interpolate(target_config, t);
//                    IK_solver(start_config, tmp_state);
//                    state_t* new_state = state_space->alloc_point();
//                    state_space->set_from_vector(tmp_state, new_state);
//                    //                    PRX_INFO_S("new point for the plan : " << state_space->print_point(new_state,2));
//                    result_plan.copy_onto_back(new_state, simulation::simulation_step);
//                }
//            }
//
//            void simple_planning_manipulator_plant_t::append_contingency(plan_t& result_plan, double duration)
//            {
//                double difference = duration - result_plan.length();
//                //    PRX_DEBUG_S("Difference in append: "<<difference);
//                PRX_ASSERT(difference >= 0);
//                if( result_plan.size() == 0 )
//                {
//                    state_t* state = state_space->alloc_point();
//                    result_plan.copy_onto_back(state, 0.0);
//                    state_space->free_point(state);
//                }
//                result_plan.back().duration += difference;
//            }
//
//        }
//    }
//}