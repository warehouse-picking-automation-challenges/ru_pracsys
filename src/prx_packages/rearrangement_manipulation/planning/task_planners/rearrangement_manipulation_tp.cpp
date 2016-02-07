/**
 * @file manipulation_tp.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/task_planners/rearrangement_manipulation_tp.hpp"
#include "planning/task_planners/manipulator_tp.hpp"
#include "planning/problem_specifications/rearrangement_manipulation_specification.hpp"
#include "planning/problem_specifications/manipulator_specification.hpp"
#include "planning/problem_specifications/manipulation_mp_specification.hpp"
#include "planning/queries/manipulator_query.hpp"
#include "planning/modules/system_name_validity_checker.hpp"
#include "planning/modules/obstacle_aware_astar.hpp"
#include "planning/graphs/manipulation_graph.hpp"
#include "planning/graphs/pebble_graph.hpp"
#include "planning/modules/pebble_solver.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/goals/goal_state.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

//Includes from manipulation package
#include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"
#include "../../../baxter/simulation/ikfast/baxter_left.hpp"
#include "prx/planning/modules/validity_checkers/temporal_validity_checker.hpp"

#include <vector>
#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp>
//#include <boost/graph/subgraph.hpp>
//#include <boost/graph/compressed_sparse_row_graph.hpp>
#include <boost/graph/connected_components.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::rearrangement_manipulation_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            rearrangement_manipulation_tp_t::rearrangement_manipulation_tp_t()
            {
                poses_set_length = 0;

                stable_pose_memory.push_back(new double());
                stable_pose_memory.push_back(new double());
                stable_pose_space = new space_t("XY", stable_pose_memory);

                _manipulator = NULL;
                _object = NULL;
                collision_object = NULL;

                char* w = std::getenv("PRACSYS_PATH");

                prx_output_dir = std::string(w) + "/prx_output/";
                prx_input_dir = std::string(w) + "/prx_input/";

                mpg_graph_time = 0;

                manip_sampler = NULL;
                system_name_validity_checker = NULL;

                transit_specification = NULL;
                transfer_specification = NULL;

                found_final_arrangement = false;
                validate_solution = false;
            }

            rearrangement_manipulation_tp_t::~rearrangement_manipulation_tp_t()
            {
                PRX_ASSERT(false);
                manip_state_space->free_point(manip_state);
                manip_state_space->free_point(safe_state);
                manip_state_space->free_point(released_point);
                manip_state_space->free_point(retracted_point);

                mo_space->free_point(grasped_point);

                object_state_space->free_point(object_state);
                object_state_space->free_point(initial_pose);
                object_state_space->free_point(goal_pose);

                all_object_space->free_point(real_initial_state);

                stable_pose_space->free_point(stable_pose_state);

                manip_control_space->free_point(manip_ctrl);
                manip_control_space->free_point(safe_control);

                retract_plan.clear();
                retract_path.clear();

                poses_set.clear();
            }

            void rearrangement_manipulation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                task_planner_t::init(reader, template_reader);
                PRX_INFO_S("Initializing Manipulation task planner ...");

                pc_name_manipulator_only = parameters::get_attribute("pc_name_manipulator_only", reader, template_reader, "pc_name_manipulator_only");
                pc_name_object_only = parameters::get_attribute("pc_name_object_only", reader, template_reader, "pc_name_object_only");
                pc_name_manipulator_with_object = parameters::get_attribute("pc_name_manipulator_with_object", reader, template_reader, "pc_name_manipulator_with_object");
                pc_name_manipulator_with_active_object = parameters::get_attribute("pc_name_manipulator_with_active_object", reader, template_reader, "pc_name_manipulator_with_active_object");
                pc_name_transit_inform = parameters::get_attribute("pc_name_transit_inform", reader, template_reader, "pc_name_transit_inform");
                pc_name_transfer_inform = parameters::get_attribute("pc_name_transfer_inform", reader, template_reader, "pc_name_transfer_inform");
                pc_name_all_objects = parameters::get_attribute("pc_name_all_objects", reader, template_reader, "pc_name_all_objects");
                pc_name_grasp_planning = parameters::get_attribute("pc_name_grasp_planning", reader, template_reader, "pc_name_grasp_planning");

                //================================//
                //  For Manipulator Task Planner  //
                //================================//

                if( parameters::has_attribute("manipulation_tp_name", reader, template_reader) )
                    manipulation_tp_name = parameters::get_attribute("manipulation_tp_name", reader, template_reader);
                else
                    PRX_FATAL_S("The name of the manipulation task planner is not specified!");

                const parameter_reader_t* specification_template_reader = NULL;
                if( parameters::has_attribute("manip_specification", reader, template_reader) )
                {
                    if( parameters::has_attribute("manip_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("manip_specification/template"));
                    }
                    output_specifications[manipulation_tp_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "manip_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing manipulation specification!!!");
                }

                output_queries[manipulation_tp_name] = new manipulator_query_t();


                //===========//
                //  Modules  //
                //===========//

                if( reader->has_attribute("stable_pose_space") )
                    stable_pose_space->init(reader->get_child("stable_pose_space").get());
                else if( template_reader != NULL )
                    stable_pose_space->init(template_reader->get_child("stable_pose_space").get());
                else
                    PRX_FATAL_S("Missing stable pose space for rearrangement rearrangement task planner!");

                if( parameters::has_attribute("manip_sampler", reader, template_reader) )
                    manip_sampler = static_cast<manip_sampler_t*>(parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "manip_sampler", template_reader, "manip_sampler"));
                else
                    PRX_FATAL_S("Missing sampler attribute for manipulation task planner!");

                if( parameters::has_attribute("manip_validity_checker", reader, template_reader) )
                {
                    system_name_validity_checker = dynamic_cast<system_name_validity_checker_t*>(parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "manip_validity_checker", template_reader, "manip_validity_checker"));
                    if( system_name_validity_checker == NULL )
                        PRX_FATAL_S("Rearrangement manipulation task planner initialize a validity_checker that it is not system_name_validity_checker!");
                }
                else
                    PRX_FATAL_S("Missing system_name_validity_checker attribute for rearrangement task planner!");

                //=============================//
                //  Tools for Motion Planners  //
                //=============================//

                std::string element = "planners_specifications/transit";
                if( parameters::has_attribute(element, reader, template_reader) )
                {
                    if( parameters::has_attribute(element + "/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                    }
                    transit_specification = dynamic_cast<manipulation_mp_specification_t*>(parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, specification_template_reader, ""));
                    if( transit_specification == NULL )
                        PRX_FATAL_S("The transit specification has to be manipulation_mp_specification_t");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transit planner's specification!!!");
                }

                element = "planners_specifications/transfer";
                if( parameters::has_attribute(element, reader, template_reader) )
                {
                    if( parameters::has_attribute(element + "/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                    }
                    transfer_specification = dynamic_cast<manipulation_mp_specification_t*>(parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, specification_template_reader, ""));
                    if( transfer_specification == NULL )
                        PRX_FATAL_S("The transfer specification has to be manipulation_mp_specification_t");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transfer planner's specification!!!");
                }

                //=================//
                //  For Statistics //
                //=================//

                time_limit = parameters::get_attribute_as<double>("time_limit", reader, template_reader, 3600);
                gather_statistics = parameters::get_attribute_as<bool>("gather_statistics", reader, template_reader, false);
                initial_biasing = parameters::get_attribute_as<bool>("initial_biasing", reader, template_reader, true);

            }

            void rearrangement_manipulation_tp_t::reset()
            {
                retract_path.clear();
                retract_plan.clear();

                poses_set_length = 0;
            }

            void rearrangement_manipulation_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
            }

            const statistics_t* rearrangement_manipulation_tp_t::get_statistics()
            {
                PRX_WARN_S("Get statistics for manipulation tp is not implemented!");
                return new statistics_t();
            }

            void rearrangement_manipulation_tp_t::link_specification(specification_t* new_spec)
            {
                task_planner_t::link_specification(new_spec);
                specs = static_cast<rearrangement_manipulation_specification_t*>(new_spec);
            }

            void rearrangement_manipulation_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
            }

            void rearrangement_manipulation_tp_t::setup()
            {
                PRX_DEBUG_COLOR("Setup rearrangement_manipulation_tp ...", PRX_TEXT_CYAN);

                detect_plants();

                if( _manipulator == NULL )
                    PRX_FATAL_S("You need at least one manipulator for the project to work!");
                if( collision_object == NULL )
                    PRX_FATAL_S("You need at least two movable object for the rearrangement project!");

                PRX_DEBUG_COLOR("manipulator :  " << _manipulator->get_pathname(), PRX_TEXT_RED);
                PRX_DEBUG_COLOR("object :  " << _object->get_pathname(), PRX_TEXT_RED);

                context_flags true_flags(true, true);
                context_flags active_flags(true, false);

                util::hash_t<std::string, context_flags> mappings;
                mappings[_manipulator->get_pathname()] = true_flags;

                model->create_new_planning_context(pc_name_all_objects, mappings, active_flags);

                mappings[_object->get_pathname()] = active_flags;
                model->create_new_planning_context(pc_name_manipulator_with_active_object, mappings);
                model->create_new_planning_context(pc_name_transit_inform, mappings);

                mappings[_object->get_pathname()].plannable = true;
                model->create_new_planning_context(pc_name_manipulator_with_object, mappings);

                model->create_new_planning_context(pc_name_grasp_planning, mappings, active_flags);

                mappings[_manipulator->get_pathname()].set(false, false);
                model->create_new_planning_context(pc_name_object_only, mappings);

                mappings[collision_object->get_pathname()] = active_flags;
                model->create_new_planning_context(pc_name_collision_check, mappings);

                mappings[_manipulator->get_pathname()].set(true, true);
                model->create_new_planning_context(pc_name_transfer_inform, mappings);


                ////////////////////
                //                Testing the planning contexts.
                //                model->use_context(pc_name_real_world);
                //                PRX_DEBUG_COLOR("pc_name_real_world: (" << model->get_state_space()->get_dimension() << " , " << model->get_active_space()->get_dimension() << ") (" << model->get_state_space()->get_space_name() << " , " << model->get_active_space()->get_space_name(),PRX_TEXT_GREEN);
                //
                //                model->use_context(pc_name_grasp_planning);
                //                PRX_DEBUG_COLOR("pc_name_grasp_planning: (" << model->get_state_space()->get_dimension() << " , " << model->get_active_space()->get_dimension() << ") (" << model->get_state_space()->get_space_name() << " , " << model->get_active_space()->get_space_name(),PRX_TEXT_GREEN);
                //                                                
                //                model->use_context(pc_name_manipulator_with_object);
                //                PRX_DEBUG_COLOR("pc_name_manipulator_with_object: (" << model->get_state_space()->get_dimension() << " , " << model->get_active_space()->get_dimension() << ") (" << model->get_state_space()->get_space_name() << " , " << model->get_active_space()->get_space_name(),PRX_TEXT_GREEN);
                //                
                //                model->use_context(pc_name_object_only);
                //                PRX_DEBUG_COLOR("pc_name_object_only: (" << model->get_state_space()->get_dimension() << " , " << model->get_active_space()->get_dimension() << ") (" << model->get_state_space()->get_space_name() << " , " << model->get_active_space()->get_space_name(),PRX_TEXT_GREEN);
                //
                //                model->use_context(pc_name_manipulator_only);
                //                PRX_DEBUG_COLOR("pc_name_manipulator_only: (" << model->get_state_space()->get_dimension() << " , " << model->get_active_space()->get_dimension() << ") (" << model->get_state_space()->get_space_name() << " , " << model->get_active_space()->get_space_name(),PRX_TEXT_GREEN);
                ///////////////////

                //Initializing the spaces.
                model->use_context(pc_name_grasp_planning);
                mo_space = model->get_state_space();
                other_objects_space = model->get_active_space();

                model->use_context(pc_name_all_objects);
                all_object_space = model->get_active_space();
                manip_state_space = model->get_state_space();
                manip_control_space = model->get_control_space();

                model->use_context(pc_name_collision_check);
                object_state_space = model->get_state_space();
                collision_object_space = model->get_active_space();

                //Allocating the helping state/control point variables.
                manip_state = manip_state_space->alloc_point();
                safe_state = manip_state_space->alloc_point();
                released_point = manip_state_space->alloc_point();
                retracted_point = manip_state_space->alloc_point();
                grasped_point = mo_space->alloc_point();
                object_state = object_state_space->alloc_point();
                initial_pose = object_state_space->alloc_point();
                goal_pose = object_state_space->alloc_point();
                real_initial_state = all_object_space->alloc_point();
                stable_pose_state = stable_pose_space->alloc_point();

                PRX_ASSERT(manip_control_space != NULL);
                manip_ctrl = manip_control_space->alloc_point();
                safe_control = manip_control_space->alloc_point();

                manip_control_vec.resize(manip_control_space->get_dimension());

                retract_path.link_space(manip_state_space);
                retract_plan.link_control_space(manip_control_space);

                poses_set_length = 0;

                manip_state_space->set_from_vector(specs->safe_position, safe_state);
                manip_control_space->set_from_vector(specs->safe_position, safe_control);

                //Initialize the special modules that we need to use.
                manip_sampler->link_info(_manipulator, manip_state_space, object_state_space);

                system_name_validity_checker->setup_checker(_manipulator, _object->get_pathname());

                specs->link_spaces(manip_state_space, manip_control_space);
                specs->setup(model);

                transit_constraints = new std::set<unsigned>();
                transfer_constraints = new std::set<unsigned>();

                //Building the specifications for the manipulation motion planners, under the manipulation task planner.
                transit_specification->validity_checker = system_name_validity_checker;
                transit_specification->sampler = specs->sampler;
                transit_specification->_manipulator = _manipulator;
                //The object state space is being used for the collision checking because this is the only active
                //object during the transit state.
                transit_specification->collision_object_space = object_state_space;
                transit_specification->valid_constraints = transit_constraints;
                transit_specification->deserialization_file = prx_input_dir + specs->transit_graph_file;
                transit_specification->link_spaces(manip_state_space, manip_control_space);
                transit_specification->setup(model);

                transfer_specification->validity_checker = system_name_validity_checker;
                transfer_specification->sampler = manip_sampler;
                transfer_specification->_manipulator = _manipulator;
                transfer_specification->object_space = object_state_space;
                transfer_specification->collision_object_space = collision_object_space;
                transfer_specification->valid_constraints = transfer_constraints;
                transfer_specification->deserialization_file = prx_input_dir + specs->transfer_graph_file;
                transfer_specification->link_spaces(mo_space, manip_control_space);
                transfer_specification->setup(model);

                transit_astar = dynamic_cast<obstacle_aware_astar_t*>(transit_specification->astar);
                transfer_astar = dynamic_cast<obstacle_aware_astar_t*>(transfer_specification->astar);

                //Construct the specification for the manipulation task planner.                
                manip_specs = dynamic_cast<manipulator_specification_t*>(output_specifications[manipulation_tp_name]);
                manip_specs->validity_checker = system_name_validity_checker;
                manip_specs->local_planner = specs->local_planner;
                manip_specs->sampler = specs->sampler;
                manip_specs->manip_sampler = manip_sampler;
                manip_specs->link_spaces(manip_state_space, manip_control_space);
                manip_specs->link_extra_spaces(mo_space, manip_control_space);

                manip_specs->max_tries = specs->max_tries;
                manip_specs->max_different_grasps = specs->max_different_grasps;
                manip_specs->retract_distance = specs->retract_distance;
                manip_specs->pc_name_manipulator_only = pc_name_manipulator_only;
                manip_specs->pc_name_object_only = pc_name_object_only;
                manip_specs->pc_name_manipulator_with_object = pc_name_manipulator_with_object;
                manip_specs->pc_name_manipulator_with_active_object = pc_name_manipulator_with_active_object;
                manip_specs->pc_name_transit_inform = pc_name_transit_inform;
                manip_specs->pc_name_transfer_inform = pc_name_transfer_inform;
                //TODO: This has to be the pebble graph setup with k+b objects.
                manip_specs->pc_name_transit_planning = pc_name_all_objects;
                //TODO: This has to be the pebble graph setup with k+b-1 objects on place and 1 in the manipulator.
                manip_specs->pc_name_transfer_planning = pc_name_grasp_planning;
                manip_specs->safe_position = specs->safe_position;
                manip_specs->manipulator = _manipulator;
                manip_specs->object = _object;
                manip_specs->setup(model);

                manip_query = dynamic_cast<manipulator_query_t*>(output_queries[manipulation_tp_name]);
                manip_query->link_spaces(manip_state_space, manip_control_space);


                manip_tp = dynamic_cast<manipulator_tp_t*>(planners[manipulation_tp_name]);

                //                metric->link_space(object_state_space);
                mpg_size = specs->_k + specs->_b;
                mpg_vertices.resize(mpg_size);
            }

            bool rearrangement_manipulation_tp_t::serialize()
            {
                std::string file = prx_output_dir + "Check_this_file.txt";
                PRX_DEBUG_COLOR(" Inside serialize preprocess manipulation, saving to file: " << file, PRX_TEXT_CYAN);
                std::ofstream fout(file.c_str());
                PRX_ASSERT(fout.is_open());
                fout << poses_set_length << std::endl;

                foreach(pose_t pose, poses_set)
                {
                    pose.serialize(fout, manip_state_space, mo_space, object_state_space);
                }
                return true;
            }

            bool rearrangement_manipulation_tp_t::deserialize()
            {
                std::string file = prx_input_dir + specs->poses_file;
                PRX_DEBUG_COLOR(" Inside deserialize rearrangement manipulation task planner, reading from file: " << file, PRX_TEXT_CYAN);
                std::ifstream fin(file.c_str());
                PRX_ASSERT(fin.is_open());
                fin >> poses_set_length;

                poses_set.resize(poses_set_length);
                seed_poses.resize(poses_set_length);
                for( unsigned i = 0; i < poses_set_length; ++i )
                {
                    poses_set[i].deserialize(fin, manip_state_space, manip_control_space, mo_space, object_state_space);
                    seed_poses[i] = std::make_pair(i, poses_set[i].state);
                }

                return true;
            }

            bool rearrangement_manipulation_tp_t::succeeded() const
            {
                //if( input_specification->get_stopping_criterion()->satisfied() )
                return true;
                //return false;
            }

            bool rearrangement_manipulation_tp_t::execute()
            {
                unsigned seeds_start = 0;
                if( specs->deserialize_flag )
                {
                    deserialize();
                    seeds_start = poses_set_length;
                }
                else
                {
                    //TODO: This version might not work because of the manipulation motion planner that does not know how to build a graph.
                    //Knows only to inform it and use it. 
                    PRX_FATAL_S("This version of rearrangement manipulation currently does not work! Exit!!!");
                    compute_random_poses(specs->num_poses);
                    for( unsigned i = 0; i < poses_set_length; ++i )
                    {
                        compute_posible_grasps(poses_set[i], specs->max_different_grasps, specs->max_tries);
                    }
                }

                const std::vector< std::vector< double > >* poses = specs->get_initial_poses();
                unsigned j = 0; //indexing for the real initial state.

                //Build the initial state from the current positions of the cups, given by the query.
                initial_poses_ids.resize(poses->size());

                int pose_index;
                for( unsigned i = 0; i < poses->size(); ++i )
                {
                    object_state_space->copy_vector_to_point(poses->at(i), object_state);
                    if( (pose_index = similar_pose(object_state)) == -1 )
                    {
                        poses_set.push_back(pose_t());
                        poses_set.back().state = object_state_space->clone_point(object_state);
                        PRX_DEBUG_COLOR("Initial pose: " << object_state_space->print_point(poses_set.back().state), PRX_TEXT_CYAN);
                        compute_posible_grasps(poses_set.back(), specs->max_different_grasps, specs->max_tries);
                        if( poses_set.back().grasped_set.size() == 0 )
                        {
                            PRX_ERROR_S("One of the Initial poses cannot be grasped! The problem is unsolvable!");
                            PRX_ERROR_S(object_state_space->print_point(poses_set.back().state));
                            return false;
                        }

                        query_poses.push_back(std::make_pair(poses_set_length, poses_set.back().state));
                        pose_index = poses_set_length;
                        ++poses_set_length;
                    }

                    for( unsigned p = 0; p < object_state_space->get_dimension(); ++p, ++j )
                        real_initial_state->memory[j] = poses->at(i)[p];
                    initial_poses_ids[i] = pose_index;
                    pose_has_object[pose_index] = i;
                }
                //
                //                PRX_DEBUG_COLOR("Initial poses: ", PRX_TEXT_GREEN);
                //
                //                foreach(unsigned i, initial_poses_ids)
                //                {
                //                    PRX_DEBUG_COLOR(i << " ) " << object_state_space->print_point(poses_set[i].state, 5), PRX_TEXT_CYAN);
                //                }

                //Adding the target poses to the poses_set.
                poses = specs->get_target_poses();
                target_poses_ids.resize(poses->size());
                for( unsigned i = 0; i < poses->size(); ++i )
                {
                    object_state_space->copy_vector_to_point(poses->at(i), object_state);
                    if( (pose_index = similar_pose(object_state)) == -1 )
                    {
                        poses_set.push_back(pose_t());
                        poses_set.back().state = object_state_space->clone_point(object_state);
                        compute_posible_grasps(poses_set.back(), specs->max_different_grasps, specs->max_tries);
                        if( poses_set.back().grasped_set.size() == 0 )
                        {
                            PRX_ERROR_S("One of the Target poses cannot be grasped! The problem is unsolvable!");
                            PRX_ERROR_S(object_state_space->print_point(poses_set.back().state));
                            return false;
                        }
                        query_poses.push_back(std::make_pair(poses_set_length, poses_set.back().state));
                        pose_index = poses_set_length;
                        ++poses_set_length;
                    }

                    target_poses_ids[i] = pose_index;
                }
                //                PRX_DEBUG_COLOR("Target poses: ", PRX_TEXT_MAGENTA);
                //
                //                foreach(unsigned i, target_poses_ids)
                //                {
                //                    PRX_DEBUG_COLOR(i << " ) " << object_state_space->print_point(poses_set[i].state, 5), PRX_TEXT_CYAN);
                //                }
                //

                poses = specs->get_extra_poses();
                for( unsigned i = 0; i < poses->size(); ++i )
                {
                    object_state_space->copy_vector_to_point(poses->at(i), object_state);
                    if( similar_pose(object_state) == -1 )
                    {
                        poses_set.push_back(pose_t());
                        poses_set.back().state = object_state_space->clone_point(object_state);
                        compute_posible_grasps(poses_set.back(), specs->max_different_grasps, specs->max_tries);
                        if( poses_set.back().grasped_set.size() == 0 )
                        {
                            PRX_ERROR_S("One of the Extra poses cannot be grasped! This poses will not be added.");
                            PRX_ERROR_S(object_state_space->print_point(poses_set.back().state));
                            poses_set.back().clear(object_state_space, manip_state_space, mo_space);
                            poses_set.pop_back();
                        }
                        else
                        {
                            query_poses.push_back(std::make_pair(poses_set_length, poses_set.back().state));
                            ++poses_set_length;
                        }
                    }
                }

                pose_checked.resize(poses_set_length);

                //After deserialization only the new query poses can give new seeds for the graphs. 
                for( unsigned i = seeds_start; i < poses_set_length; ++i )
                {
                    for( unsigned j = 0; j < poses_set[i].grasped_set.size(); ++j )
                    {
                        transit_specification->add_seed(poses_set[i].retracted_set[j]);
                        transfer_specification->add_seed(poses_set[i].grasped_set[j]);
                    }
                }
                transit_specification->add_seed(safe_state);

                transit_specification->link_poses(&seed_poses);
                transit_specification->link_query_poses(&query_poses);

                transfer_specification->link_poses(&seed_poses);
                transfer_specification->link_query_poses(&query_poses);

                manip_specs->transit_graph_specification = transit_specification;
                manip_specs->transfer_graph_specification = transfer_specification;

                manip_tp->link_specification(manip_specs);
                manip_tp->setup();
                manip_tp->execute();

                resolve_query();

                return true;
            }

            void rearrangement_manipulation_tp_t::resolve_query()
            {
                create_super_graph();

                //                smoothing_info_t* a11 = new smoothing_info_t(manip_control_space);
                //                a11->from_pose = 3;
                //                a11->to_pose = 2;
                //                a11->object_id = 2;
                //                plan_parts.push_back(a11);
                //                smoothing_info_t* a1 = new smoothing_info_t(manip_control_space);
                //                a1->from_pose = 2;
                //                a1->to_pose = 6;
                //                a1->object_id = 2;
                //                plan_parts.push_back(a1);
                //                smoothing_info_t* a2 = new smoothing_info_t(manip_control_space);
                //                a2->from_pose = 5;
                //                a2->to_pose = 0;
                //                a2->object_id = 0;
                //                //                a2->constraints.insert(1);
                //                plan_parts.push_back(a2);
                //                smoothing_info_t* a22 = new smoothing_info_t(manip_control_space);
                //                a22->from_pose = 4;
                //                a22->to_pose = 3;
                //                a22->object_id = 1;
                //                plan_parts.push_back(a22);
                //                smoothing_info_t* a3 = new smoothing_info_t(manip_control_space);
                //                a3->from_pose = 6;
                //                a3->to_pose = 9;
                //                a3->object_id = 2;
                //                plan_parts.push_back(a3);

                //Test Backwards.
                smoothing_info_t* a1 = new smoothing_info_t(manip_control_space);
                a1->from_pose = 3;
                a1->to_pose = 6;
                a1->object_id = 2;
                plan_parts.push_back(a1);
                smoothing_info_t* a2 = new smoothing_info_t(manip_control_space);
                a2->from_pose = 5;
                a2->to_pose = 2;
                a2->object_id = 0;
                //                a2->constraints.insert(1);
                plan_parts.push_back(a2);
                smoothing_info_t* a3 = new smoothing_info_t(manip_control_space);
                a3->from_pose = 4;
                a3->to_pose = 7;
                a3->object_id = 1;
                plan_parts.push_back(a3);
                smoothing_info_t* a4 = new smoothing_info_t(manip_control_space);
                a4->from_pose = 6;
                a4->to_pose = 0;
                a4->object_id = 2;
                a4->constraints.insert(5);
                plan_parts.push_back(a4);
                if( specs->apply_smoothing )
                    smoothing();

                compose_final_plan();
                return;
            }

            void rearrangement_manipulation_tp_t::update_vis_info() const
            {

                return;

                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {

                    planner->update_visualization();
                }
            }

            bool rearrangement_manipulation_tp_t::detect_plants()
            {
                model->use_context("full_space");
                std::vector< plant_t* > all_plants;
                model->get_system_graph().get_plants(all_plants);

                foreach(plant_t* plant, all_plants)
                {
                    if( _manipulator == NULL && dynamic_cast<manipulator_plant_t*>(plant) != NULL )
                        _manipulator = static_cast<manipulator_plant_t*>(plant);

                    if( _object == NULL && dynamic_cast<movable_body_plant_t*>(plant) != NULL )
                        _object = static_cast<movable_body_plant_t*>(plant);
                    else if( collision_object == NULL && dynamic_cast<movable_body_plant_t*>(plant) != NULL )
                        collision_object = static_cast<movable_body_plant_t*>(plant);

                    if( _manipulator != NULL && _object != NULL && collision_object != NULL )
                        return true;
                }
                return false;
            }

            void rearrangement_manipulation_tp_t::compute_random_poses(unsigned size)
            {
                std::string old_context = model->get_current_context();
                model->use_context(pc_name_object_only);
                //poses from the specification correspond to the initial position of the cups.
                poses_set.resize(size);
                seed_poses.resize(size);

                for( unsigned i = 0; i < size; ++poses_set_length, ++i )
                {
                    poses_set[poses_set_length].state = object_state_space->alloc_point();
                    poses_set[poses_set_length].state->memory[2] = specs->z_on_table;
                    poses_set[poses_set_length].state->memory[3] = 0;
                    poses_set[poses_set_length].state->memory[4] = 0;
                    poses_set[poses_set_length].state->memory[5] = 0.70710678118;
                    poses_set[poses_set_length].state->memory[6] = 0.70710678118;

                    do
                    {
                        sampler->sample(stable_pose_space, stable_pose_state);
                        if( stable_pose_state != NULL )
                        {
                            poses_set[poses_set_length].state->memory[0] = stable_pose_state->memory[0];
                            poses_set[poses_set_length].state->memory[1] = stable_pose_state->memory[1];
                        }
                    }
                    while( stable_pose_state == NULL || !validity_checker->is_valid(poses_set[poses_set_length].state) || similar_pose(poses_set[poses_set_length].state) != -1 );
                    seed_poses[poses_set_length] = std::make_pair(poses_set_length, poses_set[poses_set_length].state);
                }

                model->use_context(old_context);
            }

            void rearrangement_manipulation_tp_t::compute_posible_grasps(pose_t& pose, int number_of_grasps, int max_tries)
            {
                std::string old_context = model->get_current_context();
                for( int i = 0; i < number_of_grasps; ++i )
                {
                    if( get_grasp(grasped_point, pose.state, max_tries) )
                    {
                        PRX_DEBUG_COLOR(i << ") grasped the pose: " << object_state_space->print_point(pose.state, 5), PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("===================  Accepted grasp!  =============", PRX_TEXT_GREEN)
                        pose.grasped_set.push_back(mo_space->clone_point(grasped_point));
                        pose.ungrasped_set.push_back(manip_state_space->clone_point(released_point));

                        PRX_ASSERT(manip_state_space->equal_points(released_point, retract_path[0]));
                        pose.retracted_set.push_back(manip_state_space->clone_point(retract_path.back()));

                        //That is ok because state == control for rigid bodies. 
                        manip_state_space->copy_point_to_vector(released_point, manip_control_vec);
                        control_t* ctrl = manip_control_space->alloc_point();
                        manip_control_space->copy_vector_to_point(manip_control_vec, ctrl);

                        double duration = retract_plan[0].duration;
                        retract_plan.copy_onto_front(ctrl, duration);
                        manip_control_space->free_point(ctrl);
                        pose.retracting_plans.push_back(retract_plan);
                        pose.reaching_plans.push_back(plan_t());
                        pose.reaching_plans.back().reverse_plan(retract_plan);
                        PRX_ASSERT(pose.reaching_plans.back().size() == retract_plan.size());
                    }
                }
                PRX_DEBUG_COLOR("=============== DONE =============", PRX_TEXT_BROWN);
                model->use_context(old_context);
            }

            bool rearrangement_manipulation_tp_t::get_grasp(sim::state_t* point, const sim::state_t* pose_state, int max_tries)
            {
                int tries = -1;
                do
                {
                    ++tries;
                    if( tries >= max_tries )
                        return false;

                }

                while( !manip_sampler->sample_near_object(point, mo_space, pose_state) || !is_valid_grasp(point) );
                return true;
            }

            bool rearrangement_manipulation_tp_t::is_valid_grasp(state_t* point)
            {
                model->use_context(pc_name_manipulator_with_object);
                if( !validity_checker->is_valid(point) )
                {
                    PRX_DEBUG_COLOR("Not Valid !", PRX_TEXT_RED);

                    foreach(collision_pair_t pair, model->get_colliding_bodies()->get_body_pairs())
                    {
                        PRX_DEBUG_COLOR(pair.first << "  -  " << pair.second, PRX_TEXT_MAGENTA);
                    }
                    return false;
                }
                //point has to be a grasped point that correspond to both manipulator configuration
                //and the pose of the object that the manipulator grasping. 
                mo_space->copy_from_point(point);
                manip_state_space->copy_to_point(released_point);
                object_state_space->copy_to_point(object_state);

                released_point->memory.back() = 0;
                model->use_context(pc_name_manipulator_with_active_object);
                if( validity_checker->is_valid(released_point) )
                {
                    PRX_DEBUG_COLOR("VALID released point!", PRX_TEXT_GREEN);
                    _manipulator->get_end_effector_offset_configuration(tmp_config, released_point, 0, 0, -(specs->retract_distance));
                    retract_path.clear();
                    retract_plan.clear();
                    if( valid_move(retract_plan, retract_path, released_point, released_point, tmp_config) )
                    {
                        PRX_DEBUG_COLOR("VALID MOVE point!", PRX_TEXT_GREEN);
                        return true;
                    }
                }
                return false;
            }

            bool rearrangement_manipulation_tp_t::valid_move(plan_t& plan, trajectory_t& path, const state_t* manip_start, const state_t* start, config_t & goal_config)
            {
                //                PRX_DEBUG_COLOR("Valid_Move to " << goal_config.print(), PRX_TEXT_BLUE);
                if( _manipulator->IK_steering(plan, manip_control_space, manip_start, goal_config) )
                {
                    //                    PRX_DEBUG_COLOR("Going to propagate : " << manip_state_space->serialize_point(manip_start, 5), PRX_TEXT_BROWN);
                    local_planner->propagate(start, plan, path);
                    if( path.size() != 0 && validity_checker->is_valid(path) )
                        return true;
                }
                return false;
            }

            int rearrangement_manipulation_tp_t::similar_pose(state_t * pose)
            {
                for( unsigned i = 0; i < poses_set_length; ++i )
                    if( poses_set[i].equal(object_state_space, pose) )
                        return i;

                return -1;
            }

            bool rearrangement_manipulation_tp_t::validate_full_path()
            {
                motion_planning_query_t* query = dynamic_cast<motion_planning_query_t*>(input_query);
                trajectory_t full_path;

                model->set_propagate_response(true);
                manip_state_space->copy_from_point(safe_state);
                all_object_space->copy_from_point(real_initial_state);
                model->use_context("full_space");
                const space_t* full_space = model->get_full_state_space();
                full_path.link_space(full_space);
                state_t* curr_state = manip_state_space->clone_point(safe_state);
                state_t* new_state = full_space->alloc_point();
                model->use_context(pc_name_all_objects);

                foreach(plan_step_t step, query->plan)
                {
                    //                    PRX_DEBUG_COLOR(manip_control_space->print_point(step.control, 5) << "   t:" << step.duration, PRX_TEXT_CYAN);
                    model->propagate_once(curr_state, step.control, step.duration, curr_state);
                    full_space->copy_to_point(new_state);
                    full_path.copy_onto_back(new_state);
                    //                    PRX_DEBUG_COLOR(full_space->print_point(new_state, 5), PRX_TEXT_GREEN);
                }
                full_space->free_point(new_state);

                model->use_context("full_space");
                bool valid = validity_checker->is_valid(full_path);
                model->set_propagate_response(false);
                return valid;
            }

            //==============================================================//
            //                       Smoothing Code                         //
            //==============================================================//  

            void rearrangement_manipulation_tp_t::smoothing()
            {
                curr_poses = new unsigned[specs->_k];
                std::string before_str = "";
                std::string smooth1_str = "";
                std::string smooth2_str = "";

                foreach(smoothing_info_t* part, plan_parts)
                {
                    before_str += "< " + part->print() + " >\n";
                    //                    PRX_DEBUG_COLOR("< " << part->print() << " >", PRX_TEXT_GREEN);
                }

                std::set<unsigned> poses;
                detect_object_positions(poses, (*plan_parts.begin())->object_id, plan_parts.begin());
                PRX_DEBUG_COLOR("First poses: " << print(poses), PRX_TEXT_GREEN);
                smooth1_str = before_str;
                while( smoothing_part1() )
                {

                    PRX_DEBUG_COLOR("Before new smoothing round: \n" << before_str, PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("-------------------------------------------------", PRX_TEXT_BROWN);
                    before_str = smooth1_str;
                    smooth1_str = "";

                    foreach(smoothing_info_t* part, plan_parts)
                    {
                        smooth1_str += "< " + part->print() + " >\n";
                        //                        PRX_DEBUG_COLOR("< " << part->print() << " >", PRX_TEXT_GREEN);
                    }
                    PRX_DEBUG_COLOR("Smoothing 1: \n" << smooth1_str, PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("-------------------------------------------------", PRX_TEXT_CYAN);
                    smoothing_part2();

                    smooth2_str = "";

                    foreach(smoothing_info_t* part, plan_parts)
                    {
                        smooth2_str += "< " + part->print() + " >\n";
                        //                        PRX_DEBUG_COLOR("< " << part->print() << " >", PRX_TEXT_GREEN);
                    }
                    PRX_DEBUG_COLOR("==================================================", PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("-----           AFTER ONE LOOP               -----", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("==================================================", PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("Before new smoothing round: \n" << smooth1_str, PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("-------------------------------------------------", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("Smoothing 1: \n" << smooth1_str, PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("-------------------------------------------------", PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("Smoothing 2: \n" << smooth2_str, PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("-------------------------------------------------", PRX_TEXT_GREEN);
                }
                
                smoothing_part3();
            }

            bool rearrangement_manipulation_tp_t::smoothing_part1()
            {
                bool changed = false;
                list<smoothing_info_t*>::iterator it = plan_parts.begin();
                list<smoothing_info_t*>::iterator it_search;
                list<smoothing_info_t*>::iterator it_between;
                while( it != plan_parts.end() )
                {
                    unsigned id = (*it)->object_id;
                    //                    PRX_DEBUG_COLOR((*it)->print(), PRX_TEXT_CYAN);
                    it_search = it;
                    it_search++;
                    if( it_search != plan_parts.end() )
                    {
                        //We need to avoid the case where the objects are already consecutive 
                        if( id != (*it_search)->object_id )
                        {
                            while( it_search != plan_parts.end() )
                            {
                                //                            PRX_DEBUG_COLOR((*it_search)->print(), PRX_TEXT_GREEN);
                                if( (*it_search)->object_id == id )
                                {
                                    bool swap = true;
                                    it_between = it;
                                    it_between++;
                                    while( it_between != it_search && swap )
                                    {
                                        //                                    PRX_DEBUG_COLOR((*it_between)->print(), PRX_TEXT_LIGHTGRAY);
                                        if( (*it_search)->is_constrained_by((*it_between)->from_pose) || (*it_between)->is_constrained_by((*it_search)->to_pose) )
                                            swap = false;
                                        it_between++;
                                    }
                                    if( swap )
                                    {
                                        changed = true;
                                        smoothing_info_t* keep = (*it_search);
                                        plan_parts.erase(it_search);
                                        //                                    PRX_DEBUG_COLOR(keep->print(), PRX_TEXT_MAGENTA);
                                        it_between = it;
                                        it_between++;
                                        plan_parts.insert(it_between, keep);
                                    }
                                    break;
                                }
                                it_search++;
                            }
                        }
                        else
                        {
                            changed = true;
                        }
                    }

                    it++;
                }

                std::string smooth1_str = "";

                foreach(smoothing_info_t* part, plan_parts)
                {
                    smooth1_str += "< " + part->print() + " >\n";
                }
                PRX_DEBUG_COLOR("Before BAckward: \n" << smooth1_str, PRX_TEXT_CYAN);

                //Backwards
                list<smoothing_info_t*>::reverse_iterator rit = plan_parts.rbegin();
                list<smoothing_info_t*>::reverse_iterator rit_search;
                list<smoothing_info_t*>::reverse_iterator rit_between;
                while( rit != plan_parts.rend() )
                {
                    unsigned id = (*rit)->object_id;
                    PRX_DEBUG_COLOR((*rit)->print(), PRX_TEXT_CYAN);
                    rit_search = rit;
                    rit_search++;
                    if( rit_search != plan_parts.rend() )
                    {
                        //We need to avoid the case where the objects are already consecutive 
                        if( id != (*rit_search)->object_id )
                        {
                            while( rit_search != plan_parts.rend() )
                            {
                                PRX_DEBUG_COLOR((*rit_search)->print(), PRX_TEXT_GREEN);
                                if( (*rit_search)->object_id == id )
                                {
                                    bool swap = true;
                                    rit_between = rit;
                                    rit_between++;
                                    while( rit_between != rit_search && swap )
                                    {
                                        PRX_DEBUG_COLOR((*rit_between)->print(), PRX_TEXT_LIGHTGRAY);
                                        if( (*rit_search)->is_constrained_by((*rit_between)->to_pose) || (*rit_between)->is_constrained_by((*rit_search)->from_pose) )
                                            swap = false;
                                        rit_between++;
                                    }
                                    if( swap )
                                    {
                                        changed = true;
                                        smoothing_info_t* keep = (*rit_search);
                                        rit_search++;
                                        PRX_DEBUG_COLOR((*rit_search.base())->print(), PRX_TEXT_RED);
                                        plan_parts.erase(rit_search.base());
                                        PRX_DEBUG_COLOR(keep->print(), PRX_TEXT_MAGENTA);
                                        rit_between = rit;
                                        rit_between++;
                                        plan_parts.insert(rit_between.base(), keep);
                                    }
                                    break;
                                }
                                rit_search++;
                            }
                        }
                        else
                        {
                            changed = true;
                        }
                    }

                    rit++;
                }

                return changed;
            }

            bool rearrangement_manipulation_tp_t::smoothing_part2()
            {
                bool changed = false;

                list<smoothing_info_t*>::iterator it = plan_parts.begin();
                list<smoothing_info_t*>::iterator it2;

                while( it != plan_parts.end() )
                {
                    unsigned id = (*it)->object_id;
                    PRX_DEBUG_COLOR("Checking part: " << (*it)->print(), PRX_TEXT_GREEN);
                    it2 = it;
                    it2++;
                    if( it2 != plan_parts.end() && (*it2)->object_id == id )
                    {
                        changed = true;
                        while( it2 != plan_parts.end() && (*it2)->object_id == id )
                        {
                            it2++;
                        }
                        it2--;
                        PRX_DEBUG_COLOR("Checking part:  " << (*it)->print(), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Checking part2: " << (*it2)->print(), PRX_TEXT_CYAN);
                        unsigned to_pose = (*it2)->to_pose;
                        if( (*it)->from_pose != to_pose )
                        {
                            std::set<unsigned> poses;
                            detect_object_positions(poses, id, it);
                            PRX_DEBUG_COLOR("Detected poses: " << print(poses), PRX_TEXT_MAGENTA);
                            //                            poses.erase((*it)->from_pose);
                            *transit_constraints = poses;
                            *transfer_constraints = poses;
                            transfer_constraints->erase((*it)->from_pose);
                            transfer_constraints->erase(to_pose);

                            manip_query->clear();
                            manip_query->mode = manipulator_query_t::PRX_FULL_PATH;
                            manip_query->path_quality = manipulator_query_t::PRX_BEST_PATH;
                            manip_query->from_pose = (*it)->from_pose;
                            manip_query->to_pose = to_pose;
                            manip_query->start_pose = poses_set[(*it)->from_pose];
                            manip_query->target_pose = poses_set[to_pose];
                            manip_tp->link_query(manip_query);
                            manip_tp->resolve_query();
                            PRX_ASSERT(manip_query->plans[0]->size() != 0);

                            while( it2 != it )
                            {
                                plan_parts.erase(it2);
                                it2--;
                            }

                            (*it)->to_pose = to_pose;
                            (*it)->plan = *manip_query->plans[0];
                            (*it)->constraints = manip_query->full_constraints[0];
                            PRX_DEBUG_COLOR("REturned constraints      : " << print(manip_query->constraints[0]), PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR("REturned full constraints : " << print(manip_query->full_constraints[0]), PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR("STORED full constraints   : " << print((*it)->constraints), PRX_TEXT_CYAN);
                        }
                        else
                        {
                            while( it2 != it )
                            {
                                plan_parts.erase(it2);
                                it2--;
                            }
                            plan_parts.erase(it);
                            it--;
                        }
                    }

                    it++;
                }

                return changed;

            }

            void rearrangement_manipulation_tp_t::smoothing_part3() 
            {
                list<smoothing_info_t*>::iterator it = plan_parts.begin();
                list<smoothing_info_t*>::iterator it2;

                while( it != plan_parts.end() )
                {
                    PRX_DEBUG_COLOR((*it)->reaching_point << "    -       " << (*it)->retracting_point, PRX_TEXT_GREEN);
                    it++;
                }
            }

            void rearrangement_manipulation_tp_t::detect_object_positions(std::set<unsigned> &poses, unsigned id, const std::list<smoothing_info_t*>::iterator& it)
            {
                PRX_DEBUG_COLOR("-------- for the id : " << id << " -----------", PRX_TEXT_MAGENTA);
                bool checked[specs->_k];
                std::fill_n(checked, specs->_k, false);
                checked[id] = true;
                curr_poses[id] = (*it)->from_pose;

                list<smoothing_info_t*>::iterator iter = it;
                while( iter != plan_parts.begin() )
                {
                    unsigned new_id = (*iter)->object_id;
                    PRX_DEBUG_COLOR("Checked id: " << new_id << "  is_checked : " << checked[new_id], PRX_TEXT_CYAN);
                    if( !checked[new_id] )
                    {
                        curr_poses[new_id] = (*iter)->to_pose;
                        checked[new_id] = true;
                    }
                    iter--;
                }

                unsigned new_id = (*iter)->object_id;
                PRX_DEBUG_COLOR("Checked begin id: " << new_id << "  is_checked : " << checked[new_id], PRX_TEXT_BLUE);
                if( !checked[new_id] )
                {
                    curr_poses[new_id] = (*iter)->to_pose;
                    checked[new_id] = true;
                }

                iter = it;
                iter++;
                while( iter != plan_parts.end() )
                {
                    unsigned new_id = (*iter)->object_id;
                    PRX_DEBUG_COLOR("Checked id: " << new_id << "  is_checked : " << checked[new_id], PRX_TEXT_GREEN);
                    if( !checked[new_id] )
                    {
                        curr_poses[new_id] = (*iter)->from_pose;
                        checked[new_id] = true;
                    }
                    iter++;
                }

                for( unsigned i = 0; i < specs->_k; ++i )
                    poses.insert(curr_poses[i]);
                PRX_DEBUG_COLOR("DONE DETECTION : " << print(poses), PRX_TEXT_BROWN);
            }

            //==============================================================//
            //                           MPG Code                           //
            //==============================================================//           

            undirected_graph_t* rearrangement_manipulation_tp_t::create_MPG(std::vector<pebble_edge_t*>& constrained_edges, const std::vector< unsigned >& seed_positions, bool is_fake)
            {
                mpg_graph_time = statistics_clock.measure();
                PRX_DEBUG_COLOR(" =========================================== ", PRX_TEXT_BLUE);
                PRX_DEBUG_COLOR("        Generation of a Pebble Graph         ", PRX_TEXT_MAGENTA);
                PRX_DEBUG_COLOR(" =========================================== ", PRX_TEXT_BLUE);

                undirected_graph_t* MPG_graph = new undirected_graph_t();

                for( unsigned i = 0; i < poses_set_length; ++i )
                    pose_checked[i] = false;

                metric->clear();
                std::set<unsigned> new_poses;

                select_poses_time = statistics_clock.measure();

                unsigned index = 0;
                unsigned num_vertices = 0;
                unsigned num_checked = num_vertices;

                //If we've been given a seed of initial positions to use, go ahead and copy those in
                for( unsigned i = 0; i < seed_positions.size(); ++i )
                {
                    index = seed_positions[i];
                    if( valid_pose_in_MPG(MPG_graph, index) )
                    {
                        PRX_DEBUG_COLOR("Adding seed  : " << object_state_space->print_point(poses_set[index].state, 4) << "   index : " << index, PRX_TEXT_CYAN);
                        undirected_vertex_index_t v = MPG_graph->add_vertex< pebble_node_t > ();
                        mpg_vertices[num_vertices] = v;
                        MPG_graph->get_vertex_as< pebble_node_t > (v)->init_node(object_state_space, poses_set[index].state, index);
                        new_poses.insert(index);
                        pose_checked[ index ] = true;
                        num_checked++;
                        metric->add_point(MPG_graph->operator[](v));
                        ++num_vertices;
                    }
                }
                if( is_fake )
                    return MPG_graph;

                while( num_vertices < mpg_size && num_checked < poses_set_length )
                {
                    int failures = -1;
                    do
                    {
                        int bias_chance = uniform_int_random(0, 99);
                        if( ++failures == specs->max_random_failures )
                        {
                            MPG_graph->clear();
                            delete MPG_graph;
                            mpg_graph_time = 0;
                            grasp_poses_time = 0;
                            connect_poses_time = 0;
                            select_poses_time = 0;
                            return NULL;
                        }
                        //5 % of the time we are biased towards the goal positions
                        if( bias_chance < specs->goal_biasing )
                            index = target_poses_ids[uniform_int_random(0, target_poses_ids.size() - 1)];
                        else
                            index = uniform_int_random(0, poses_set_length - 1);
                    }
                    while( pose_checked[index] );
                    pose_checked[index] = true;
                    ++num_checked;

                    if( valid_pose_in_MPG(MPG_graph, index) )
                    {
                        PRX_DEBUG_COLOR("Adding seed  : " << object_state_space->print_point(poses_set[index].state, 4) << "   index : " << index, PRX_TEXT_CYAN);
                        undirected_vertex_index_t v = MPG_graph->add_vertex< pebble_node_t > ();
                        mpg_vertices[num_vertices] = v;
                        MPG_graph->get_vertex_as< pebble_node_t > (v)->init_node(object_state_space, poses_set[index].state, index);
                        new_poses.insert(index);
                        pose_checked[ index ] = true;
                        num_checked++;
                        if( metric->has_point(MPG_graph->operator[](v)) )
                            PRX_WARN_S("Metric already has the point (" << object_state_space->print_point(MPG_graph->operator[](v)->point, 2) << ")! Skipped add");
                        else
                            metric->add_point(MPG_graph->operator[](v));
                        ++num_vertices;
                    }
                }

                if( num_vertices < mpg_size || pebble_graph_exists(new_poses) )
                {
                    MPG_graph->clear();
                    delete MPG_graph;
                    mpg_graph_time = 0;
                    grasp_poses_time = 0;
                    connect_poses_time = 0;
                    select_poses_time = 0;
                    return NULL;
                }

                if( new_poses.size() != mpg_size )
                    PRX_FATAL_S("Something really really bad happened! mpg_size:" << mpg_size << "   new_mpg_size:" << new_poses.size());

                select_poses_time = statistics_clock.measure() - select_poses_time;

                *transit_constraints = new_poses;

                //Clear Grasping info

                foreach(unsigned key, grasping_poses | boost::adaptors::map_keys)
                {

                    foreach(grasping_info_t* info, grasping_poses[key])
                    {
                        if( info != NULL )
                        {
                            info->clear();
                            delete info;
                        }
                    }
                    grasping_poses[key].clear();
                }
                grasping_poses.clear();

                //Grasp all the poses
                std::vector<undirected_vertex_index_t> mpg_vertices;

                grasp_poses_time = statistics_clock.measure();

                foreach(undirected_vertex_index_t v, boost::vertices(MPG_graph->graph))
                {
                    mpg_vertices.push_back(v);
                    unsigned pose_index = MPG_graph->get_vertex_as<pebble_node_t > (v)->position_index;

                    PRX_DEBUG_COLOR("=============================== ==============", PRX_TEXT_BLUE);
                    PRX_DEBUG_COLOR("      ===      For Pose : " << pose_index << "      ===", PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("=============================================", PRX_TEXT_BLUE);
                    manip_query->clear();
                    manip_query->mode = manipulator_query_t::PRX_REACH_PATH;
                    manip_query->path_quality = manipulator_query_t::PRX_ALL_PATHS;
                    manip_query->start_pose = poses_set[pose_index];

                    //                    all_object_space->copy_from_point(real_initial_state);
                    //                    manip_state_space->copy_from_point(safe_state);
                    manip_tp->link_query(manip_query);
                    manip_tp->resolve_query();

#ifndef NDEBUG
                    PRX_DEBUG_COLOR("Plans SIZE: " << manip_query->plans.size(), PRX_TEXT_MAGENTA);
                    for( unsigned i = 0; i < manip_query->plans.size(); ++i )
                    {
                        if( manip_query->plans[i] != NULL )
                        {
                            // && manip_query->plans[i]->size() > 0 )
                            PRX_DEBUG_COLOR("Plan[" << i << "]: " << manip_query->plans[i]->size() << "    constraints: " << manip_query->constraints[i].size() << "    cost:" << manip_query->solutions_costs[i], PRX_TEXT_GREEN);
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("Plan[" << i << "]: NULL", PRX_TEXT_GREEN);
                        }
                    }
#endif


                    for( unsigned i = 0; i < manip_query->plans.size(); ++i )
                    {
                        if( manip_query->plans[i] != NULL )
                        {
                            grasping_info_t* new_info = new grasping_info_t();
                            new_info->plan = *(manip_query->plans[i]);
                            new_info->retracted_point = new_info->plan.size() - poses_set[pose_index].reaching_plans[i].size();
                            //                            PRX_DEBUG_COLOR("compare points: \n" << manip_control_space->print_point(new_info->plan.at(new_info->retracted_point).control,5) << "\n" << manip_state_space->print_point(poses_set[pose_index].retracted_set[i],5), PRX_TEXT_BROWN);
                            new_info->constraints = manip_query->constraints[i];
                            new_info->full_constraints = manip_query->full_constraints[i];
                            new_info->distance = manip_query->solutions_costs[i];
                            new_info->pose = &poses_set[pose_index];
                            PRX_DEBUG_COLOR(pose_index << ") NEW INFO: " << new_info->print(), PRX_TEXT_LIGHTGRAY);
                            grasping_poses[pose_index].push_back(new_info);
                        }
                        else
                        {
                            grasping_poses[pose_index].push_back(NULL);
                        }
                    }
                }

                grasp_poses_time = statistics_clock.measure() - grasp_poses_time;

                *transfer_constraints = new_poses;

                foreach(unsigned key, grasping_poses | boost::adaptors::map_keys)
                {
                    PRX_DEBUG_COLOR("==============================================================", PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("Pose : " << key, PRX_TEXT_CYAN);

                    foreach(grasping_info_t* info, grasping_poses[key])
                    {
                        if( info != NULL )
                            PRX_DEBUG_COLOR(info->print(), PRX_TEXT_LIGHTGRAY);
                    }
                    PRX_DEBUG_COLOR("==============================================================", PRX_TEXT_CYAN);
                }
                //                PRX_ASSERT(false);

                connect_poses_time = statistics_clock.measure();
                for( unsigned i = 0; i < mpg_size - 1; ++i )
                {
                    //i is the node that we will transfer an object.
                    pebble_node_t* to_node = MPG_graph->get_vertex_as<pebble_node_t > (mpg_vertices[i]);
                    transfer_constraints->erase(to_node->position_index);
                    manip_query->target_pose = poses_set[to_node->position_index];
                    for( unsigned j = i + 1; j < mpg_size; ++j )
                    {
                        pebble_node_t* from_node = MPG_graph->get_vertex_as<pebble_node_t > (mpg_vertices[j]);
                        transfer_constraints->erase(from_node->position_index);
                        PRX_DEBUG_COLOR("=============================================", PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("     ===   Going to connect " << from_node->position_index << " -> " << to_node->position_index << "   ===", PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("=============================================", PRX_TEXT_GREEN);

                        manip_query->clear();
                        manip_query->mode = manipulator_query_t::PRX_TRANSFER_PATH;
                        manip_query->path_quality = manipulator_query_t::PRX_ALL_PATHS;
                        manip_query->start_pose = poses_set[from_node->position_index];
                        manip_tp->link_query(manip_query);
                        manip_tp->resolve_query();

                        int grasp_from, grasp_to, transfer_index;
                        double cost;
                        std::set<unsigned> constraints;
                        std::set<unsigned> full_constraints;

                        if( find_best_path(from_node->position_index, to_node->position_index, constraints, full_constraints, cost, grasp_from, grasp_to, transfer_index) )
                        {
                            PRX_DEBUG_COLOR("grasp_From : " << grasp_from << "    grasp_to:" << grasp_to, PRX_TEXT_MAGENTA);
                            if( constraints.size() == 0 )
                            {
                                PRX_DEBUG_COLOR("Adding an edge : " << from_node->position_index << " -> " << to_node->position_index, PRX_TEXT_GREEN);
                                undirected_edge_index_t e = MPG_graph->add_edge<pebble_edge_t > (mpg_vertices[j], mpg_vertices[i], cost);
                                pebble_edge_t* edge = MPG_graph->get_edge_as<pebble_edge_t > (e);
                                edge->init_edge(mpg_vertices[j], mpg_vertices[i], constraints, from_node->position_index, to_node->position_index);
                                edge->full_constraints = full_constraints;
                                edge->plan = grasping_poses[from_node->position_index][grasp_from]->plan;
                                edge->reaching_point = grasping_poses[from_node->position_index][grasp_from]->retracted_point;
                                edge->plan += *(manip_query->plans[transfer_index]);
                                edge->add_plan_reversed(grasping_poses[to_node->position_index][grasp_to]->plan);
                                edge->retracting_point = edge->plan.size() - grasping_poses[to_node->position_index][grasp_to]->retracted_point;
                                //                                PRX_DEBUG_COLOR("compare points: \n" << manip_control_space->print_point(edge->plan.at(edge->reaching_point).control,5) << "\n" 
                                //                                        << manip_state_space->print_point(grasping_poses[from_node->position_index][grasp_from]->pose->retracted_set[0],5) << "\n" 
                                //                                        << manip_state_space->print_point(grasping_poses[from_node->position_index][grasp_from]->pose->retracted_set[1],5) << "\n" 
                                //                                        << manip_state_space->print_point(grasping_poses[from_node->position_index][grasp_from]->pose->retracted_set[2],5) 
                                //                                        , PRX_TEXT_BROWN);
                                //                                
                                //                                PRX_DEBUG_COLOR("compare points: \n" << manip_control_space->print_point(edge->plan.at(edge->retracting_point).control,5) << "\n" 
                                //                                        << manip_state_space->print_point(grasping_poses[to_node->position_index][grasp_to]->pose->retracted_set[0],5) << "\n" 
                                //                                        << manip_state_space->print_point(grasping_poses[to_node->position_index][grasp_to]->pose->retracted_set[1],5) << "\n" 
                                //                                        << manip_state_space->print_point(grasping_poses[to_node->position_index][grasp_to]->pose->retracted_set[2],5) 
                                //                                        , PRX_TEXT_BROWN);
                                //                                PRX_ASSERT(false);
                                PRX_ASSERT(manip_control_space->equal_points(edge->plan[0].control, edge->plan.back().control));
                            }
                            else
                            {
                                pebble_edge_t* edge = new pebble_edge_t();
                                edge->init_edge(mpg_vertices[j], mpg_vertices[i], constraints, from_node->position_index, to_node->position_index);
                                edge->full_constraints = full_constraints;
                                edge->plan = grasping_poses[from_node->position_index][grasp_from]->plan;
                                edge->reaching_point = grasping_poses[from_node->position_index][grasp_from]->retracted_point;
                                edge->plan += *(manip_query->plans[transfer_index]);
                                edge->add_plan_reversed(grasping_poses[to_node->position_index][grasp_to]->plan);
                                edge->retracting_point = edge->plan.size() - grasping_poses[to_node->position_index][grasp_to]->retracted_point;
                                PRX_DEBUG_COLOR("Constrained edge : " << from_node->position_index << " -> " << to_node->position_index << "     constraints: " << edge->print_constraints(), PRX_TEXT_RED);
                                constrained_edges.push_back(edge);
                            }
                        }
                        transfer_constraints->insert(from_node->position_index);
                    }
                    transfer_constraints->insert(to_node->position_index);
                }
                connect_poses_time = statistics_clock.measure() - connect_poses_time;

                return MPG_graph;
            }

            bool rearrangement_manipulation_tp_t::valid_pose_in_MPG(undirected_graph_t* graph, int index)
            {
                std::string old_context = model->get_current_context();
                model->use_context(pc_name_collision_check);

                //Collision object is the active object in the world. The plannable object's state will be set by the copy_from_point.
                collision_object_space->copy_from_point(poses_set[index].state);

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    if( !validity_checker->is_valid(graph->get_vertex_as<pebble_node_t > (v)->point) )
                    {
                        model->use_context(old_context);
                        return false;
                    }
                }
                model->use_context(old_context);
                return true;
            }

            bool rearrangement_manipulation_tp_t::pebble_graph_exists(const std::set<unsigned> poses)
            {

                foreach(hnode_data_t* data, pebble_graph_storage)
                {
                    if( data->have_the_same_poses(poses) )
                        return true;
                }
                return false;
            }

            bool rearrangement_manipulation_tp_t::find_best_path(unsigned start_index, unsigned target_index, std::set<unsigned>& constraints, std::set<unsigned>& full_constraints, double& edge_cost, int& from, int& to, int &middle)
            {
                edge_cost = PRX_INFINITY;
                unsigned min_constraints = PRX_INFINITY;
                std::set<unsigned> curr_constraints;
                std::set<unsigned> curr_full_constraints;
                double cost = 0;
                from = -1;
                to = -1;
                unsigned trans_index = 0;
                for( unsigned i = 0; i < poses_set[start_index].ungrasped_set.size(); ++i )
                {
                    if( grasping_poses[start_index][i] != NULL )
                    {
                        PRX_DEBUG_COLOR("Grasping pose (s , " << poses_set[start_index].ungrasped_set.size() << "): " << start_index << "." << i << " :" << grasping_poses[start_index][i]->print(), PRX_TEXT_CYAN);
                        for( unsigned j = 0; j < poses_set[target_index].grasped_set.size(); ++j, ++trans_index )
                        {
                            curr_constraints = grasping_poses[start_index][i]->constraints;
                            cost = grasping_poses[start_index][i]->distance;
                            if( manip_query->plans[trans_index] != NULL && grasping_poses[target_index][j] != NULL )
                            {
                                cost += manip_query->solutions_costs[trans_index];
                                cost += grasping_poses[target_index][j]->distance;

                                curr_constraints.insert(manip_query->constraints[trans_index].begin(), manip_query->constraints[trans_index].end());
                                curr_constraints.insert(grasping_poses[target_index][j]->constraints.begin(), grasping_poses[target_index][j]->constraints.end());
                                curr_constraints.erase(target_index);
                                PRX_DEBUG_COLOR("Grasping pose (t , " << poses_set[target_index].grasped_set.size() << "): " << target_index << "." << j << " :" << grasping_poses[target_index][j]->print() << " \n\t\t\t |cost:" << cost << "    |c|:" << curr_constraints.size(), PRX_TEXT_GREEN);
                                if( curr_constraints.size() < min_constraints || (curr_constraints.size() == min_constraints && cost < edge_cost) )
                                {
                                    min_constraints = curr_constraints.size();
                                    constraints = curr_constraints;
                                    full_constraints = grasping_poses[start_index][i]->full_constraints;
                                    full_constraints.insert(manip_query->full_constraints[trans_index].begin(), manip_query->full_constraints[trans_index].end());
                                    full_constraints.insert(grasping_poses[target_index][j]->full_constraints.begin(), grasping_poses[target_index][j]->full_constraints.end());
                                    edge_cost = cost;
                                    from = i;
                                    to = j;
                                    middle = trans_index;
                                }
                            }
                        }
                    }
                }
                return from != -1;
            }

            //==============================================================//
            //                    Super Graph Code                          //
            //==============================================================//

            bool rearrangement_manipulation_tp_t::create_super_graph()
            {
                statistics_clock.reset();
                std::vector<pebble_edge_t*> start_constraints;

                undirected_graph_t* init_mpg = create_MPG(start_constraints, initial_poses_ids, true);
                if( init_mpg == NULL )
                    PRX_FATAL_S("Cannot build the MPG with the initial poses. Problem does not have a solution.");

                pebble_graph_storage.push_back(new hnode_data_t(init_mpg));
                v_initial = super_graph.add_vertex< super_node_t > ();
                super_node_t* h_initial = super_graph.get_vertex_as< super_node_t > (v_initial);
                h_initial->data = new hnode_data_t(*pebble_graph_storage.back());
                h_initial->data->update_signature(initial_poses_ids);
                //                h_initial->data->print("Initial node");

                std::vector<pebble_edge_t*> end_constraints;
                undirected_graph_t* target_mpg = create_MPG(end_constraints, target_poses_ids, true);
                if( target_mpg == NULL )
                    PRX_FATAL_S("Cannot build the MPG with the initial poses. Problem does not have a solution.");

                pebble_graph_storage.push_back(new hnode_data_t(target_mpg));
                v_target = super_graph.add_vertex< super_node_t > ();
                super_node_t* h_target = super_graph.get_vertex_as< super_node_t > (v_target);
                h_target->data = new hnode_data_t(*pebble_graph_storage.back());
                h_target->data->update_signature(target_poses_ids);
                //                h_target->data->print("Target node");

                if( initial_biasing )
                {
                    unsigned comb_n = initial_poses_ids.size();
                    unsigned comb_r = PRX_MINIMUM(comb_n, mpg_size - comb_n);
                    std::vector < std::vector<unsigned > > combinations = hnode_data_t::generate_arrangements(comb_n, comb_r);

                    std::vector<unsigned> biasing_init_combination(comb_n + comb_r, 0);
                    std::vector<unsigned> biasing_goal_combination(comb_n + comb_r, 0);

                    //Both initial and target poses have the same size.
                    for( unsigned i = 0; i < comb_n; ++i )
                    {
                        biasing_init_combination[i] = initial_poses_ids[i];
                        biasing_goal_combination[i] = target_poses_ids[i];
                    }


                    for( unsigned i = 0; i < combinations.size(); ++i )
                    {
                        for( unsigned j = 0, c = 0; j < comb_n; ++j )
                        {
                            if( combinations[i][j] == 1 )
                            {
                                biasing_init_combination[comb_n + c] = target_poses_ids[j];
                                biasing_goal_combination[comb_n + c] = initial_poses_ids[j];
                                ++c;
                            }
                        }

                        if( expand(biasing_init_combination) )
                        {
                            if( gather_statistics )
                                write_statistics();
                            if( found_final_arrangement )
                                break;
                        }
                        if( statistics_clock.measure() >= time_limit )
                            return false;

                        if( expand(biasing_goal_combination) )
                        {
                            if( gather_statistics )
                                write_statistics();
                            if( found_final_arrangement )
                                break;
                        }
                        if( statistics_clock.measure() >= time_limit )
                            return false;
                    }
                } //&&&&&&&&&&&&&&& END OF BIASING...&&&&&&&&&&&&&&&&

                if( gather_statistics )
                    write_statistics();

                int h_iter = 0;
                while( !found_final_arrangement )
                {
                    PRX_DEBUG_COLOR("Hypertree iteration: " << h_iter, PRX_TEXT_MAGENTA);
                    PRX_DEBUG_COLOR("H before: " << boost::num_vertices(super_graph.graph) << " nodes, " << boost::num_edges(super_graph.graph) << " edges.\n", PRX_TEXT_GREEN);

                    super_node_t* node_to_expand = random_selection(super_graph);
                    if( node_to_expand )
                    {
                        if( statistics_clock.measure() >= time_limit )
                            return false;
                        std::vector< unsigned > seed_positions;
                        node_to_expand->data->generate_valid_arrangement(seed_positions);
                        if( expand(seed_positions) )
                        {
                            if( gather_statistics )
                                write_statistics();
                        }
                    }
                    else
                    {
                        PRX_FATAL_S("Error, random selection of a super node failed.");
                    }
                    PRX_DEBUG_COLOR("H after: " << boost::num_vertices(super_graph.graph) << " nodes, " << boost::num_edges(super_graph.graph) << " edges.\n", PRX_TEXT_GREEN);
                    h_iter++;
                }

                query_super_graph();
                return true;
            }

            bool rearrangement_manipulation_tp_t::expand(std::vector< unsigned >& seed_positions)
            {
                //Using this seed, I create the pumped configuration (0-level pebble graph)
                std::vector< pebble_edge_t* > constrained_edges;
                undirected_graph_t* new_graph = create_MPG(constrained_edges, seed_positions);
                //We were unable to generate a new node given this seed. :(
                if( new_graph == NULL )
                {

                    PRX_DEBUG_COLOR("EXPAND FAILED TO CREATE NODE!?\n\n", PRX_TEXT_RED);
                    return false;
                }
                //Using that graph, I generate a huge set of node data.
                std::vector< const super_node_t* > node_data;
                std::vector< std::vector< unsigned > > sig_map;

                generate_nodes(node_data, sig_map, new_graph);

                if( found_final_arrangement )
                    return true;

                connect_siblings(node_data, constrained_edges);

                int num = boost::connected_components(super_graph.graph, super_graph.components);

                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("    Hyper node Info after Sibling edges:  ", PRX_TEXT_MAGENTA);
                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("--- Connected Components : " << num, PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR("--- Vertices : " << boost::num_vertices(super_graph.graph), PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR("--- Edges : " << boost::num_edges(super_graph.graph), PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR(" ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ ", PRX_TEXT_RED);

                if( super_graph.components[v_initial] == super_graph.components[v_target] )
                {
                    found_final_arrangement = true;
                }

                return true;
            }

            void rearrangement_manipulation_tp_t::generate_nodes(std::vector< const super_node_t* >& new_nodes, std::vector< std::vector< unsigned > >& sig_map, undirected_graph_t * p_graph)
            {
                generated_indices.clear();

                //generate our initial data from which to generate possible nodes                
                pebble_graph_storage.push_back(new hnode_data_t(p_graph));
                PRX_INFO_S("Starting a new Pebble Graph: " << pebble_graph_storage.size());
                if( gather_statistics )
                {
                    P_e += boost::num_edges(p_graph->graph);
                    P_cc += boost::connected_components(p_graph->graph, p_graph->components);
                }
                hnode_data_t* hold = pebble_graph_storage.back();

                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("    Generated the node which stores original info    ", PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_GREEN);

                hold->print("Generation, Hold node");

                //Generate the signatures                 
                std::vector< std::vector< unsigned > > Sigma = hold->generate_signatures(specs->_k);

                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("    All of the other nodes are being generated now   ", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("     Generated " << Sigma.size() << " signatures.", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_CYAN);

                foreach(std::vector< unsigned > sigma, Sigma)
                {
                    hnode_data_t* check = new hnode_data_t(*hold);
                    check->replace_signature(sigma);

                    //Let's see if this has already been put in the graph
                    undirected_vertex_index_t v_new = is_hnode_generated(check);
                    //Now, if that node had already existed,
                    if( v_new == NULL )
                    {
                        v_new = super_graph.add_vertex< super_node_t > ();
                        super_graph.get_vertex_as< super_node_t > (v_new)->data = check;
                        super_graph.get_vertex_as< super_node_t > (v_new)->data->print("New Node in the Super-Graph");
                    }
                    else
                    {
                        PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("    This Node Already Exist   ", PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_MAGENTA);
                        delete check;
                    }
                    generated_indices.push_back(v_new);
                    super_node_t* new_hnode = super_graph.get_vertex_as< super_node_t > (v_new);
                    new_nodes.push_back(new_hnode);
                    sig_map.push_back(new_hnode->data->signature);

                    connect_supernode(new_hnode);
                    int num = boost::connected_components(super_graph.graph, super_graph.components);


                    PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("    Hyper node Info after Hnode edges:  ", PRX_TEXT_MAGENTA);
                    PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("--- Connected Components : " << num, PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("--- Vertices : " << boost::num_vertices(super_graph.graph), PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("--- Edges : " << boost::num_edges(super_graph.graph), PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR(" ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ ", PRX_TEXT_RED);

                    if( super_graph.components[v_initial] == super_graph.components[v_target] )
                    {
                        found_final_arrangement = true;
                        return;
                    }
                }
            }

            void rearrangement_manipulation_tp_t::connect_supernode(const super_node_t * hnode)
            {
                std::vector< super_node_t* > potential_links;

                foreach(undirected_vertex_index_t v, boost::vertices(super_graph.graph))
                {
                    super_node_t* node = super_graph.get_vertex_as< super_node_t > (v);
                    if( v != hnode->index )
                    {
                        if( node->data->same_poses(hnode->data->positions) >= specs->_k )
                        {
                            potential_links.push_back(node);
                        }
                    }
                }

                //Now, we have all of the super nodes to which we can potentially link to, let's try to connect
                std::set< unsigned > arrangement;

                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR(" ===========   Connect New Node " << hnode->node_id << "    ============= ", PRX_TEXT_CYAN);

                foreach(super_node_t* node, potential_links)
                {
                    arrangement.clear();
                    //Try to find an arrangement which allows a connection.                                                            
                    if( !boost::edge(hnode->index, node->index, super_graph.graph).second && hnode->data->p_graph != node->data->p_graph && connection_arrangment(arrangement, hnode, node) )
                    {
                        //Construct and add the appropriate edges to the super_graph.
                        PRX_DEBUG_COLOR("       H-Nodes are now connected!!! " << hnode->node_id << " -> " << node->node_id, PRX_TEXT_GREEN);
                        undirected_edge_index_t e = super_graph.add_edge< super_edge_t > (hnode->index, node->index);
                        super_graph.get_edge_as< super_edge_t > (e)->init_as_switch(arrangement);

                        if( gather_statistics )
                            ++no_super_edges;
                    }
                }
                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_CYAN);
            }

            void rearrangement_manipulation_tp_t::connect_siblings(const std::vector< const super_node_t* >& new_nodes, std::vector< pebble_edge_t* >& edges)
            {
                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("         Connect Siblings   #:" << new_nodes.size(), PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR(" =================================================== ", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("", PRX_TEXT_CYAN);

                std::vector<unsigned> new_signature(new_nodes[0]->data->signature.size());

                for( unsigned i = 0; i < new_nodes.size(); ++i )
                {
                    const super_node_t* node = new_nodes[i];
                    PRX_DEBUG_COLOR("- Signature : " << node->data->print_signature(), PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("- CC_DATA will be: " << node->data->print_cc_data(), PRX_TEXT_BROWN);

                    foreach(pebble_edge_t* edge, edges)
                    {
                        PRX_DEBUG_COLOR("--- EDGE constraints: " << edge->print_constraints(), PRX_TEXT_CYAN);
                        unsigned from_component, to_component;
                        get_from_to_components(node, edge, from_component, to_component);
                        //If the edge doesn't let us get into a new cc, then it's useless, skip
                        if( (from_component != to_component) && node->data->signature[from_component] > 0 && is_feasible_edge(node->data, edge, to_component) )
                        {
                            PRX_DEBUG_COLOR("----- Valid path! Goint to connect :  " << from_component << " -> " << to_component, PRX_TEXT_CYAN);

                            std::vector<unsigned> new_signature = node->data->signature;
                            new_signature[from_component]--;
                            new_signature[to_component]++;
                            const super_node_t* other_node = find_sibling_with_signature(new_nodes, new_signature);

                            //If I found a super node that is sibling and has the new signature.
                            if( other_node != NULL )
                            {
                                PRX_DEBUG_COLOR("------- Found super node sibling with signature : " << other_node->data->print_signature(), PRX_TEXT_CYAN);
                                if( !boost::edge(node->index, other_node->index, super_graph.graph).second )
                                {
                                    PRX_DEBUG_COLOR("---------  From:  " << edge->p_source << " -> " << edge->p_target, PRX_TEXT_GREEN);
                                    undirected_edge_index_t e = super_graph.add_edge< super_edge_t > (node->index, other_node->index);
                                    super_edge_t* s_edge = super_graph.get_edge_as< super_edge_t > (e);
                                    s_edge->init_as_motion(node->index, edge->constraints, std::make_pair(edge->p_source, edge->p_target), edge->plan);
                                    s_edge->reaching_point = edge->reaching_point;
                                    s_edge->retracting_point = edge->retracting_point;
                                    s_edge->full_constraints = edge->full_constraints;
                                    if( gather_statistics )
                                        ++no_sibling_edges;
                                }
                            }
                        }
                    }
                }
            }

            void rearrangement_manipulation_tp_t::query_super_graph()
            {
                specs->astar->link_graph(&super_graph);
                specs->astar->set_astar_mode(astar_module_t::PRX_NO_EDGE_INFO);

                super_graph.get_vertex_as<super_node_t > (v_initial)->data->print("Initial Node");
                super_graph.get_vertex_as<super_node_t > (v_target)->data->print("Target Node");
                //                PRX_ASSERT(false);

                if( specs->astar->solve(v_initial, v_target) )
                {

                    std::deque< undirected_vertex_index_t > path_vertices;
                    PRX_ASSERT(specs->astar->get_found_goal() == v_target);
                    specs->astar->extract_path(v_initial, specs->astar->get_found_goal(), path_vertices);
                    PRX_DEBUG_COLOR("Found path!!!  Size : " << path_vertices.size(), PRX_TEXT_MAGENTA);

                    std::vector<unsigned> current_arrangement = initial_poses_ids;

                    for( int i = 0; i < (int)path_vertices.size() - 1; ++i )
                    {
                        undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], super_graph.graph).first;
                        super_edge_t* edge = super_graph.get_edge_as<super_edge_t > (e);
                        super_node_t* node = super_graph.get_vertex_as<super_node_t > (path_vertices[i]);
                        node->data->print("This is the node I will take for the path");
                        PRX_DEBUG_COLOR("ARRANGEMENT: " << print(current_arrangement), PRX_TEXT_BROWN);
                        PRX_DEBUG_COLOR("====================================================================", PRX_TEXT_RED);
                        if( edge->has_motion )
                        {
                            //   PRX_ASSERT(false);
                            PRX_DEBUG_COLOR("Has motion, arrangement : " << print(current_arrangement), PRX_TEXT_GREEN);
                            PRX_DEBUG_COLOR("Motion : " << edge->motion.first << " <-> " << edge->motion.second, PRX_TEXT_GREEN);
                            PRX_DEBUG_COLOR("Constraints: " << print(edge->poses), PRX_TEXT_GREEN);
                            std::vector<unsigned> free_positions;

                            foreach(unsigned index, node->data->positions)
                            {
                                if( index != edge->motion.first && index != edge->motion.second )
                                {
                                    if( std::find(edge->poses.begin(), edge->poses.end(), index) == edge->poses.end() )
                                        free_positions.push_back(index);
                                }
                            }
                            PRX_DEBUG_COLOR("Free positions: " << print(free_positions), PRX_TEXT_GREEN)

                                    unsigned nr = free_positions.size();
                            std::vector< std::vector< unsigned > > combinations = node->data->generate_arrangements(nr, specs->_k - 1);
                            std::set<unsigned> tmp_set;
                            unsigned source_pos = edge->get_source_position(path_vertices[i]);
                            PRX_DEBUG_COLOR("Source pose: " << source_pos, PRX_TEXT_CYAN);
                            for( unsigned c = 0; c < combinations.size(); ++c )
                            {
                                for( unsigned j = 0; j < nr; ++j )
                                {
                                    if( combinations[c][j] == 1 )
                                        tmp_set.insert(free_positions[j]);
                                }
                                tmp_set.insert(source_pos);

                                if( can_assign(node, tmp_set) )
                                {
                                    PRX_DEBUG_COLOR("From arrangement : " << print(current_arrangement), PRX_TEXT_GREEN);
                                    PRX_DEBUG_COLOR("To arrangement   : " << print(tmp_set), PRX_TEXT_CYAN);
                                    solve_path(*node, current_arrangement, std::vector<unsigned>(tmp_set.begin(), tmp_set.end()));
                                    PRX_DEBUG_COLOR("After Query::: Plan Parts Size of " << plan_parts.size() << " arrangement : " << print(current_arrangement), PRX_TEXT_MAGENTA);
                                    break;
                                }
                                tmp_set.clear();
                            }

                            smoothing_info_t* part = new smoothing_info_t(manip_control_space);
                            part->from_pose = edge->get_source_position(path_vertices[i]);
                            part->to_pose = edge->get_the_other_position(part->from_pose);
                            PRX_ASSERT(pose_has_object[part->from_pose] != -1);
                            PRX_ASSERT(pose_has_object[part->to_pose] == -1 || pose_has_object[part->to_pose] == 0);
                            part->object_id = pose_has_object[part->from_pose];
                            pose_has_object[part->from_pose] = -1;
                            pose_has_object[part->to_pose] = part->object_id;
                            part->constraints = edge->full_constraints;
                            part->reaching_point = edge->reaching_point;
                            part->retracting_point = edge->retracting_point;
                            edge->get_plan(path_vertices[i], part->plan);
                            plan_parts.push_back(part);

                            PRX_DEBUG_COLOR("Motion::: Plan Size of " << plan_parts.size() << "   arrangement: " << print(current_arrangement), PRX_TEXT_MAGENTA);
                            for( unsigned j = 0; j < current_arrangement.size(); ++j )
                            {
                                if( current_arrangement[j] == edge->motion.first )
                                {
                                    current_arrangement[j] = edge->motion.second;
                                    break;
                                }
                                else if( current_arrangement[j] == edge->motion.second )
                                {
                                    current_arrangement[j] = edge->motion.first;
                                    break;
                                }
                            }

                            PRX_DEBUG_COLOR("After motion:  " << print(current_arrangement), PRX_TEXT_CYAN);
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("From arrangement : " << print(current_arrangement), PRX_TEXT_GREEN);

                            PRX_DEBUG_COLOR("To arrangement   : " << print(edge->poses), PRX_TEXT_CYAN);

                            solve_path(*node, current_arrangement, std::vector< unsigned > (edge->poses.begin(), edge->poses.end()));

                            PRX_DEBUG_COLOR("After path  : " << print(current_arrangement), PRX_TEXT_CYAN);
                        }
                    }
                    super_graph.get_vertex_as<super_node_t > (path_vertices.back())->data->print("This is the FINAL node I will take for the path");
                    for( unsigned i = 0; i < poses_set.size(); ++i )
                    {
                        PRX_DEBUG_COLOR(i << ")  " << object_state_space->print_point(poses_set[i].state, 6), PRX_TEXT_CYAN);

                    }
                    solve_path(*(super_graph.get_vertex_as<super_node_t > (path_vertices.back())), current_arrangement, target_poses_ids);
                }

                if( validate_solution )
                    validate_full_path();
            }

            void rearrangement_manipulation_tp_t::compose_final_plan()
            {
                motion_planning_query_t* query = dynamic_cast<motion_planning_query_t*>(input_query);
                query->plan.link_control_space(manip_control_space);

                foreach(smoothing_info_t* part, plan_parts)
                {
                    query->plan += part->plan;
                    PRX_DEBUG_COLOR("< " << part->print() << ">", PRX_TEXT_CYAN);
                    //                    PRX_DEBUG_COLOR("< " << part->print() << ">", (31 + (i % 7)));
                }
                PRX_ASSERT(false);
                if( !validate_full_path() )
                    PRX_ASSERT_MSG(false, "Not valid path after smoothing");
                //                for( int i = 0; i < plan_parts.size(); ++i )
                //                {
                //                    
                //                    query->plan += plan_parts[i]->plan;
                //                    PRX_DEBUG_COLOR("< " << plan_parts[i]->print() << ">", (31 + (i % 7)));
                //                }
            }

            bool rearrangement_manipulation_tp_t::connection_arrangment(std::set< unsigned >& arrangement, const super_node_t* node, const super_node_t * other)
            {
                //First, I'm assuming I already checked that they had enough common
                // positions, but I need to get those positions.
                std::vector< unsigned > common_positions(node->data->positions.size() + other->data->positions.size());
                std::vector< unsigned >::iterator it;

                unsigned cp_size = common_positions.size();
                it = std::set_intersection(node->data->positions.begin(), node->data->positions.end(), other->data->positions.begin(), other->data->positions.end(), common_positions.begin());
                PRX_ASSERT(cp_size == common_positions.size());
                common_positions.resize(it - common_positions.begin());

                PRX_DEBUG_COLOR(" ================================================================ ", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("   Attempting to find an connecting arrangement  common pos: " << common_positions.size(), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR(" ================================================================ ", PRX_TEXT_GREEN);

                node->data->print();
                other->data->print();

                //Generate all possible arrangements these common positions could have
                std::vector< std::vector< unsigned > > arrangements = node->data->generate_arrangements(common_positions.size(), specs->_k);
                //Then, start iterating the arrangements
                PRX_DEBUG_COLOR(" How many are the computed arrangements?  " << arrangements.size(), PRX_TEXT_GREEN);
                for( unsigned i = 0; i < arrangements.size(); ++i )
                {
                    arrangement.clear();
                    //For each position
                    for( unsigned j = 0; j < common_positions.size(); ++j )
                    {
                        //If we are to include it
                        if( arrangements[i][j] )
                        {
                            arrangement.insert(common_positions[j]);
                        }
                    }
                    PRX_ASSERT(arrangement.size() == specs->_k);
                    //Now that we have the arrangement positions, test if we can make the assignment
                    if( can_assign(node, arrangement) && can_assign(other, arrangement) )
                        return true;
                }

                arrangement.clear();
                return false;
            }

            bool rearrangement_manipulation_tp_t::can_assign(const super_node_t* node, const std::set< unsigned >& positions) const
            {
                unsigned counter = 0;

                foreach(std::set< unsigned > cc, node->data->cc_data)
                {

                    int count = node->data->signature[counter];
                    //For each vertex which is part of that connected component

                    foreach(unsigned p, cc)
                    {
                        //If that position index matches a position in the arrangment we want to check
                        if( positions.count(p) )
                        {
                            //We decrement the count for this connected component
                            --count;
                            if( count < 0 )
                                return false;
                        }
                    }
                    //Now that we have the correct count of the number of positions we want in this cc, compare to signature
                    counter++;
                }
                return true;
            }

            void rearrangement_manipulation_tp_t::get_from_to_components(const super_node_t* node, pebble_edge_t* edge, unsigned& component_from, unsigned& component_to)
            {
                unsigned edge_from = edge->p_source;
                unsigned edge_to = edge->p_target;


                bool got_from = false;
                bool got_to = false;
                for( unsigned i = 0; i < node->data->cc_data.size(); ++i )
                {
                    if( !got_from && node->data->cc_data[i].count(edge_from) )
                    {
                        component_from = i;
                        got_from = true;
                    }
                    if( !got_to && node->data->cc_data[i].count(edge_to) )
                    {
                        component_to = i;
                        got_to = true;
                    }
                    if( got_from && got_to )
                        return;
                }
            }

            bool rearrangement_manipulation_tp_t::is_feasible_edge(const hnode_data_t* node_data, pebble_edge_t* edge, unsigned component_to)
            {
                //For each Connected Component of the graph
                for( unsigned i = 0; i < node_data->cc_sizes.size(); ++i )
                {
                    unsigned cc_size = node_data->cc_sizes[i];

                    foreach(unsigned pos, edge->constraints)
                    {
                        cc_size -= node_data->cc_data[i].count(pos);
                    }

                    if( i == component_to )
                        cc_size--;

                    if( cc_size < node_data->signature[i] )
                        return false;
                }
                return true;
            }

            const super_node_t * rearrangement_manipulation_tp_t::find_sibling_with_signature(const std::vector<const super_node_t*>& new_nodes, const std::vector<unsigned>& signature)
            {

                foreach(const super_node_t* node, new_nodes)
                {
                    if( node->data->signature == signature )
                        return node;
                }
                return NULL;
            }

            undirected_vertex_index_t rearrangement_manipulation_tp_t::is_hnode_generated(const hnode_data_t* new_node) const
            {

                foreach(undirected_vertex_index_t v, boost::vertices(super_graph.graph))
                {
                    if( v != v_initial && v != v_target && super_graph.get_vertex_as< super_node_t > (v)->data->is_equal(new_node) )
                    {
                        return v;
                    }
                }
                return NULL;
            }

            super_node_t * rearrangement_manipulation_tp_t::random_selection(const undirected_graph_t & graph)
            {
                //randomly select a node in the super_graph
                unsigned index = uniform_int_random(2, boost::num_vertices(graph.graph) - 1);
                //expand that node                
                unsigned ct = 0;

                foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
                {
                    if( index == ct )
                    {
                        PRX_ASSERT(v != v_initial);
                        PRX_ASSERT(v != v_target);
                        return graph.get_vertex_as< super_node_t > (v);
                    }
                    ++ct;
                }
                return NULL;
            }

            void rearrangement_manipulation_tp_t::solve_path(const super_node_t& node, std::vector< unsigned >& start_arrangement, const std::vector< unsigned > goal_arrangement)
            {

                PRX_ASSERT(start_arrangement.size() == goal_arrangement.size());
                pebble_solver_t* solver = new pebble_solver_t();

                std::vector< std::pair<undirected_vertex_index_t, undirected_vertex_index_t> > plan_vertices;
                solver->resolve_query(plan_vertices, *(node.data->p_graph), start_arrangement, goal_arrangement, object_state_space);

                for( unsigned i = 0; i < plan_vertices.size(); ++i )
                {
                    smoothing_info_t* part = new smoothing_info_t(manip_control_space);
                    part->from_pose = node.data->p_graph->get_vertex_as<pebble_node_t > (plan_vertices[i].first)->position_index;
                    part->to_pose = node.data->p_graph->get_vertex_as<pebble_node_t > (plan_vertices[i].second)->position_index;
                    PRX_ASSERT(pose_has_object[part->from_pose] != -1);
                    PRX_ASSERT(pose_has_object[part->to_pose] == -1 || pose_has_object[part->to_pose] == 0);
                    part->object_id = pose_has_object[part->from_pose];
                    pose_has_object[part->from_pose] = -1;
                    pose_has_object[part->to_pose] = part->object_id;
                    //                    PRX_DEBUG_COLOR("< " << node.data->p_graph->get_vertex_as<pebble_node_t > (plan_vertices[i].first)->position_index << "," << node.data->p_graph->get_vertex_as<pebble_node_t > (plan_vertices[i].second)->position_index << ">", (31 + (i % 7)));
                    undirected_edge_index_t e = boost::edge(plan_vertices[i].first, plan_vertices[i].second, node.data->p_graph->graph).first;
                    part->constraints = node.data->p_graph->get_edge_as<pebble_edge_t > (e)->full_constraints;
                    pebble_edge_t* edge = node.data->p_graph->get_edge_as<pebble_edge_t > (e);
                    part->reaching_point = edge->reaching_point;
                    part->retracting_point = edge->retracting_point;
                    edge->get_plan(part->plan, plan_vertices[i].first);
                    plan_parts.push_back(part);

                    PRX_DEBUG_COLOR(part->print(), (31 + (i % 7)));
                }


                for( size_t i = 0; i < start_arrangement.size(); ++i )
                    start_arrangement[i] = goal_arrangement[i];
                delete solver;


            }

            void rearrangement_manipulation_tp_t::write_statistics() { }

            std::string rearrangement_manipulation_tp_t::print(const std::vector<unsigned>& arrangement)
            {

                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, arrangement)
                {
                    output << i << " , ";
                }
                return output.str();

            }

            std::string rearrangement_manipulation_tp_t::print(const std::set<unsigned>& constratins)
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, constratins)
                {
                    output << i << " , ";
                }
                return output.str();
            }

        }
    }
}