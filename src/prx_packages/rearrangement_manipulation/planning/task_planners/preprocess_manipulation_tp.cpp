/**
 * @file preprocess_manipulation_tp.cpp
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

#include "planning/task_planners/preprocess_manipulation_tp.hpp"
#include "planning/motion_planners/manipulation_mp.hpp"
#include "planning/problem_specifications/manipulation_mp_specification.hpp"
#include "planning/problem_specifications/rearrangement_manipulation_specification.hpp"
#include "planning/modules/system_name_validity_checker.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"

//Includes from manipulation package
#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "../../../manipulation/planning/modules/pose.hpp"
#include "../../../manipulation/simulation/plants/movable_body_plant.hpp"
#include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"
#include "planning/problem_specifications/manipulation_specification.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp>
#include <vector>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::preprocess_manipulation_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace baxter;
        using namespace manipulation;

        namespace rearrangement_manipulation
        {

            preprocess_manipulation_tp_t::preprocess_manipulation_tp_t()
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

                rpg_graph_time = 0;

                manip_sampler = NULL;
                system_name_validity_checker = NULL;
            }

            preprocess_manipulation_tp_t::~preprocess_manipulation_tp_t()
            {
                manip_state_space->free_point(manip_state);
                manip_state_space->free_point(safe_state);
                manip_state_space->free_point(released_point);
                manip_state_space->free_point(retracted_point);

                mo_space->free_point(grasped_point);

                object_state_space->free_point(object_state);

                stable_pose_space->free_point(stable_pose_state);

                manip_control_space->free_point(manip_ctrl);
                manip_control_space->free_point(safe_control);

                retract_plan.clear();
                retract_path.clear();

                poses_set.clear();
            }

            void preprocess_manipulation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                task_planner_t::init(reader, template_reader);
                PRX_INFO_S("Initializing Manipulation task planner ...");

                pc_name_manipulator_only = parameters::get_attribute("pc_name_manipulator_only", reader, template_reader, "pc_name_manipulator_only");
                pc_name_object_only = parameters::get_attribute("pc_name_object_only", reader, template_reader, "pc_name_object_only");
                pc_name_manipulator_with_object = parameters::get_attribute("pc_name_manipulator_with_object", reader, template_reader, "pc_name_manipulator_with_object");
                pc_name_transfer = parameters::get_attribute("pc_name_transfer", reader, template_reader, "pc_name_transfer");
                pc_name_transit = parameters::get_attribute("pc_name_transit", reader, template_reader, "pc_name_transit");
                pc_name_objects = parameters::get_attribute("pc_name_objects", reader, template_reader, "pc_name_objects");

                transit_motion_planner_name = parameters::get_attribute("transit_motion_planner_name", reader, template_reader, "transit_motion_planner_name");
                transfer_motion_planner_name = parameters::get_attribute("transfer_motion_planner_name", reader, template_reader, "transfer_motion_planner_name");
                transit_manipulation_mp_name = parameters::get_attribute("transit_manipulation_mp_name", reader, template_reader, "transit_manipulation_mp_name");
                transfer_manipulation_mp_name = parameters::get_attribute("transfer_manipulation_mp_name", reader, template_reader, "transfer_manipulation_mp_name");

                const parameter_reader_t* specification_template_reader = NULL;
                if( parameters::has_attribute("transit_manipulation_specification", reader, template_reader) )
                {
                    if( parameters::has_attribute("transit_manipulation_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("transit_manipulation_specification/template"));
                    }
                    output_specifications[transit_manipulation_mp_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "transit_manipulation_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transit manipulation specification!!!");
                }

                if( parameters::has_attribute("transfer_manipulation_specification", reader, template_reader) )
                {
                    if( parameters::has_attribute("transfer_manipulation_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("transfer_manipulation_specification/template"));
                    }
                    output_specifications[transfer_manipulation_mp_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "transfer_manipulation_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transfer manipulation specification!!!");
                }

                if( parameters::has_attribute("transit_motion_planner_specification", reader, template_reader) )
                {
                    if( parameters::has_attribute("transit_motion_planner_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("transit_motion_planner_specification/template"));
                    }
                    output_specifications[transit_motion_planner_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "transit_motion_planner_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transit motion planner's specification!!!");
                }

                if( parameters::has_attribute("transfer_motion_planner_specification", reader, template_reader) )
                {
                    if( parameters::has_attribute("transfer_motion_planner_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("transfer_motion_planner_specification/template"));
                    }
                    output_specifications[transfer_motion_planner_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "transfer_motion_planner_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transfer motion planner's specification!!!");
                }

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
                        PRX_FATAL_S("Preprocess manipulation task planner initialize a validity_checker that it is not system_name_validity_checker!");
                }
                else
                    PRX_FATAL_S("Missing system_name_validity_checker attribute for preprocess manipulation task planner!");

                transit_graph_file = parameters::get_attribute("transit_graph_file", reader, template_reader, "transit_graph.txt");
                transfer_graph_file = parameters::get_attribute("transfer_graph_file", reader, template_reader, "transfer_graph.txt");
                informed_transit_graph_file = parameters::get_attribute("informed_transit_graph_file", reader, template_reader, "informed_transit_graph.txt");
                informed_transfer_graph_file = parameters::get_attribute("informed_transfer_graph_file", reader, template_reader, "informed_transfer_graph.txt");
                poses_file = parameters::get_attribute("poses_file", reader, template_reader, "poses.txt");

                if( parameters::has_attribute("fix_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("fix_poses", reader, template_reader))
                    {

                        fix_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }

                if( parameters::has_attribute("cheating_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("cheating_poses", reader, template_reader))
                    {

                        cheating_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }


            }

            void preprocess_manipulation_tp_t::reset() { }

            void preprocess_manipulation_tp_t::link_specification(specification_t* new_spec)
            {
                task_planner_t::link_specification(new_spec);
                specs = static_cast<rearrangement_manipulation_specification_t*>(new_spec);

            }

            void preprocess_manipulation_tp_t::setup()
            {
                PRX_DEBUG_COLOR("Setup preprocess_manipulation_tp ...", PRX_TEXT_CYAN);
                detect_plants();

                if( _manipulator == NULL )
                    PRX_FATAL_S("You need at least one manipulator for the project to work!");
                if( collision_object == NULL )
                    PRX_FATAL_S("You need at least two movable object for the rearrangement project!");

                context_flags true_flags(true, true);
                context_flags active_flags(true, false);

                util::hash_t<std::string, context_flags> mappings;
                mappings[_manipulator->get_pathname()] = true_flags;
                mappings[_object->get_pathname()] = active_flags;
                model->create_new_planning_context(pc_name_transit, mappings);

                mappings[_object->get_pathname()].plannable = true;
                model->create_new_planning_context(pc_name_transfer, mappings, active_flags);

                model->create_new_planning_context(pc_name_manipulator_with_object, mappings);

                mappings[_manipulator->get_pathname()].set(false, false);
                model->create_new_planning_context(pc_name_objects, mappings, active_flags);

                model->create_new_planning_context(pc_name_object_only, mappings);

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
                model->use_context(pc_name_transfer);
                mo_space = model->get_state_space();

                model->use_context(pc_name_objects);
                object_state_space = model->get_state_space();
                collision_object_space = model->get_active_space();

                model->use_context(pc_name_manipulator_only);
                manip_state_space = model->get_state_space();
                manip_control_space = model->get_control_space();

                //Allocating the helping state/control point variables.
                manip_state = manip_state_space->alloc_point();
                safe_state = manip_state_space->alloc_point();
                released_point = manip_state_space->alloc_point();
                retracted_point = manip_state_space->alloc_point();
                grasped_point = mo_space->alloc_point();
                object_state = object_state_space->alloc_point();
                stable_pose_state = stable_pose_space->alloc_point();

                manip_ctrl = manip_control_space->alloc_point();
                safe_control = manip_control_space->alloc_point();

                manip_control_vec.resize(manip_control_space->get_dimension());

                retract_path.link_space(manip_state_space);
                retract_plan.link_control_space(manip_control_space);

                poses_set_length = 0;

                manip_sampler->link_info(_manipulator, manip_state_space, object_state_space);

                input_specification->link_spaces(manip_state_space, manip_control_space);
                input_specification->setup(model);
                manip_state_space->set_from_vector(specs->safe_position, safe_state);
                manip_control_space->set_from_vector(specs->safe_position, safe_control);

                system_name_validity_checker->setup_checker(_manipulator, _object->get_pathname());


                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[transit_motion_planner_name]);
                graph_specification->validity_checker = input_specification->validity_checker;
                graph_specification->local_planner = input_specification->local_planner;
                graph_specification->sampler = input_specification->sampler;
                graph_specification->link_spaces(manip_state_space, manip_control_space);
                graph_specification->setup(model);
                graph_specification->get_stopping_criterion()->link_motion_planner((motion_planner_t*)planners[transit_motion_planner_name]);

                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[transfer_motion_planner_name]);
                graph_specification->validity_checker = input_specification->validity_checker;
                graph_specification->local_planner = input_specification->local_planner;
                graph_specification->sampler = manip_sampler;
                graph_specification->link_spaces(mo_space, manip_control_space);
                graph_specification->setup(model);
                graph_specification->get_stopping_criterion()->link_motion_planner((motion_planner_t*)planners[transfer_motion_planner_name]);

                manipulation_specification = dynamic_cast<manipulation_mp_specification_t*>(output_specifications[transit_manipulation_mp_name]);
                manipulation_specification->validity_checker = system_name_validity_checker;
                manipulation_specification->local_planner = input_specification->local_planner;
                manipulation_specification->sampler = input_specification->sampler;
                manipulation_specification->_manipulator = _manipulator;
                //The object state space is being used for the collision checking because this is the only active
                //object during the transit state.
                manipulation_specification->collision_object_space = object_state_space;
                manipulation_specification->graph_deserialization_file = prx_output_dir + transit_graph_file;
                manipulation_specification->serialization_file = prx_output_dir + informed_transit_graph_file;
                manipulation_specification->link_spaces(manip_state_space, manip_control_space);
                manipulation_specification->setup(model);

                manipulation_specification = dynamic_cast<manipulation_mp_specification_t*>(output_specifications[transfer_manipulation_mp_name]);
                manipulation_specification->validity_checker = system_name_validity_checker;
                manipulation_specification->local_planner = input_specification->local_planner;
                manipulation_specification->sampler = manip_sampler;
                manipulation_specification->_manipulator = _manipulator;
                manipulation_specification->object_space = object_state_space;
                manipulation_specification->collision_object_space = collision_object_space;
                manipulation_specification->graph_deserialization_file = prx_output_dir + transfer_graph_file;
                manipulation_specification->serialization_file = prx_output_dir + informed_transfer_graph_file;
                manipulation_specification->link_spaces(mo_space, manip_control_space);
                manipulation_specification->setup(model);
            }

            bool preprocess_manipulation_tp_t::execute()
            {
                const prm_star_statistics_t* statistics;

                if( cheating_poses.size() != 0 )
                    compute_cheating_seeds();

                //Compute the random poses.
                compute_random_poses();

                model->use_context(pc_name_transit);
                for( unsigned i = 0; i < (unsigned)poses_set_length; ++i )
                    compute_posible_grasps(poses_set[i], specs->max_different_grasps, specs->max_tries);

                //Building the Transit graph.
                PRX_DEBUG_COLOR("Building the Transit graph.", PRX_TEXT_BROWN);
                model->use_context(pc_name_manipulator_only);
                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[transit_motion_planner_name]);
                graph_specification->clear();

                for( unsigned i = 0; i < poses_set_length; ++i )
                {

                    foreach(state_t* state, poses_set[i].retracted_set)
                    {
                        graph_specification->add_seed(state);
                    }
                }
                if( cheating_transit_seeds.size() != 0 )
                {
                    foreach(state_t* state, cheating_transit_seeds)
                    {
                        graph_specification->add_seed(state);
                    }
                }
                graph_specification->add_seed(safe_state);

                motion_planner_t* motion_planner;
                motion_planner = dynamic_cast<motion_planner_t*>(planners[transit_motion_planner_name]);
                motion_planner->link_specification(graph_specification);
                motion_planner->set_param("", "serialize_flag", true);
                motion_planner->set_param("", "serialization_file", transit_graph_file);
                motion_planner->setup();


                try
                {
                    motion_planner->execute();
                }
                catch( stopping_criteria_t::stopping_criteria_satisfied e )
                {
                    statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
                    PRX_DEBUG_COLOR("Transit Graph statistics : |seed| : " << graph_specification->get_seeds().size() << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_GREEN);

                }

                motion_planner->serialize();
                motion_planner->update_visualization();


                //Building the Transfer graph.
                PRX_DEBUG_COLOR("Building the Transfer graph.", PRX_TEXT_BROWN);
                model->use_context(pc_name_manipulator_with_object);
                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[transfer_motion_planner_name]);
                graph_specification->clear();
                for( unsigned i = 0; i < poses_set_length; ++i )
                {

                    foreach(state_t* state, poses_set[i].grasped_set)
                    {
                        graph_specification->add_seed(state);
                    }
                }
                if( cheating_transfer_seeds.size() != 0 )
                {
                    foreach(state_t* state, cheating_transfer_seeds)
                    {
                        graph_specification->add_seed(state);
                    }
                }

                motion_planner = dynamic_cast<motion_planner_t*>(planners[transfer_motion_planner_name]);
                motion_planner->link_specification(graph_specification);
                motion_planner->set_param("", "serialize_flag", true);
                motion_planner->set_param("", "serialization_file", transfer_graph_file);
                motion_planner->setup();

                try
                {
                    motion_planner->execute();
                }
                catch( stopping_criteria_t::stopping_criteria_satisfied e )
                {
                    statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
                    PRX_DEBUG_COLOR("Transfer Graph statistics : |seed| : " << graph_specification->get_seeds().size() << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_GREEN);

                }
                motion_planner->serialize();
                motion_planner->update_visualization();

                //Inform the Transit graph.
                PRX_DEBUG_COLOR("Inform the Transit graph.", PRX_TEXT_BROWN);
                model->use_context(pc_name_transit);
                manipulation_specification = dynamic_cast<manipulation_mp_specification_t*>(output_specifications[transit_manipulation_mp_name]);
                manip_mp = dynamic_cast<manipulation_mp_t*>(planners[transit_manipulation_mp_name]);
                PRX_ASSERT(manip_mp != NULL);
                manipulation_specification->link_poses(&pose_seeds);
                manip_state_space->copy_from_point(safe_state);

                manip_mp->link_specification(manipulation_specification);
                manip_mp->setup();
                manip_mp->execute();
                manip_mp->update_visualization();

                //Inform the Transfer graph.
                PRX_DEBUG_COLOR("Inform the Transfer graph.", PRX_TEXT_BROWN);
                model->use_context(pc_name_transfer);
                manipulation_specification = dynamic_cast<manipulation_mp_specification_t*>(output_specifications[transfer_manipulation_mp_name]);
                manip_mp = dynamic_cast<manipulation_mp_t*>(planners[transfer_manipulation_mp_name]);
                PRX_ASSERT(manip_mp != NULL);
                manipulation_specification->link_poses(&pose_seeds);
                manip_state_space->copy_from_point(safe_state);

                manip_mp->link_specification(manipulation_specification);
                manip_mp->setup();
                manip_mp->execute();
                manip_mp->update_visualization();

                serialize();

                return true;
            }

            const statistics_t* preprocess_manipulation_tp_t::get_statistics()
            {
                return NULL;
            }

            bool preprocess_manipulation_tp_t::succeeded() const
            {
                return true;
            }

            void preprocess_manipulation_tp_t::resolve_query() { }

            bool preprocess_manipulation_tp_t::serialize()
            {
                std::string file = prx_output_dir + poses_file;
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

            bool preprocess_manipulation_tp_t::deserialize()
            {
                std::string file = prx_output_dir + poses_file;
                PRX_DEBUG_COLOR(" Inside serialize preprocess manipulation, saving to file: " << file, PRX_TEXT_CYAN);
                std::ifstream fin(file.c_str());
                PRX_ASSERT(fin.is_open());
                fin >> poses_set_length;

                poses_set.resize(poses_set_length);
                for( unsigned i = 0; i < poses_set_length; ++i )
                {
                    poses_set[i].deserialize(fin, manip_state_space, manip_control_space, mo_space, object_state_space);
                }
                return true;
            }

            bool preprocess_manipulation_tp_t::detect_plants()
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

            void preprocess_manipulation_tp_t::compute_random_poses()
            {
                model->use_context(pc_name_object_only);
                //poses from the specification correspond to the initial position of the cups.
                poses_set.resize(fix_poses.size() + specs->num_poses);
                pose_seeds.resize(fix_poses.size() + specs->num_poses);

                //Start first with the fixed poses.
                for( unsigned i = 0; i < fix_poses.size(); ++poses_set_length, ++i )
                {
                    poses_set[poses_set_length].state = object_state_space->alloc_point();
                    object_state_space->copy_vector_to_point(fix_poses[i], poses_set[poses_set_length].state);
                    pose_seeds[poses_set_length] = std::make_pair(poses_set_length, poses_set[poses_set_length].state);
                }

                for( unsigned i = 0; i < specs->num_poses; ++poses_set_length, ++i )
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
                    pose_seeds[poses_set_length] = std::make_pair(poses_set_length, poses_set[poses_set_length].state);
                }
            }

            void preprocess_manipulation_tp_t::compute_posible_grasps(pose_t& pose, int number_of_grasps, int max_tries)
            {
                std::string old_context = model->get_current_context();
                for( int i = 0; i < number_of_grasps; ++i )
                {
                    if( get_grasp(grasped_point, pose.state, max_tries) )
                    {
                        pose.grasped_set.push_back(mo_space->clone_point(grasped_point));
                        pose.ungrasped_set.push_back(manip_state_space->clone_point(released_point));

                        PRX_ASSERT(manip_state_space->equal_points(released_point, retract_path[0]));
                        pose.retracted_set.push_back(manip_state_space->clone_point(retract_path.back()));

                        //That is ok because state == control for rigid bodies. 
                        manip_state_space->copy_point_to_vector(released_point, manip_control_vec);
                        manip_control_space->copy_vector_to_point(manip_control_vec, manip_ctrl);

                        retract_plan.copy_onto_front(manip_ctrl, retract_plan[0].duration);
                        pose.retracting_plans.push_back(retract_plan);
                        pose.reaching_plans.push_back(plan_t());
                        pose.reaching_plans.back().reverse_plan(retract_plan);
                        PRX_ASSERT(pose.reaching_plans.back().size() == retract_plan.size());
                    }
                }
                model->use_context(old_context);
            }

            void preprocess_manipulation_tp_t::compute_cheating_seeds()
            {
                for( unsigned c = 0; c < cheating_poses.size(); ++c )
                {
                    object_state_space->copy_vector_to_point(cheating_poses[c], object_state);
                    for( unsigned i = 0; i < specs->max_different_grasps; ++i )
                    {
                        if( get_grasp(grasped_point, object_state, specs->max_tries, false) )
                        {
                            cheating_transfer_seeds.push_back(mo_space->clone_point(grasped_point));
                            cheating_transit_seeds.push_back(manip_state_space->clone_point(released_point));
                        }
                    }
                }
                PRX_DEBUG_COLOR("Cheating... Transit: " << cheating_transit_seeds.size() << "    Transfer: " << cheating_transfer_seeds.size(), PRX_TEXT_LIGHTGRAY);
                PRX_ASSERT(false);
            }

            bool preprocess_manipulation_tp_t::get_grasp(sim::state_t* point, const sim::state_t* pose_state, int max_tries, bool full_motion)
            {

                int tries = -1;
                do
                {
                    ++tries;
                    if( tries >= max_tries )
                        return false;

                }
                while( !manip_sampler->sample_near_object(point, mo_space, pose_state) || !is_valid_grasp(point, full_motion) );
                return true;
            }

            bool preprocess_manipulation_tp_t::is_valid_grasp(state_t* point, bool full_motion)
            {
                model->use_context(pc_name_manipulator_with_object);
                if( !validity_checker->is_valid(point) )
                    return false;

                //point has to be a grasped point that correspond to both manipulator configuration
                //and the pose of the object that the manipulator grasping. 
                mo_space->copy_from_point(point);
                manip_state_space->copy_to_point(released_point);
                object_state_space->copy_to_point(object_state);

                released_point->memory.back() = 0;
                model->use_context(pc_name_transit);
                if( !validity_checker->is_valid(released_point) )
                    return false;

                if( full_motion )
                {
                    _manipulator->get_end_effector_offset_configuration(tmp_config, released_point, 0, 0, -(specs->retract_distance));
                    retract_path.clear();
                    retract_plan.clear();
                    if( !valid_move(retract_plan, retract_path, released_point, released_point, tmp_config) )
                        return false;
                }
                return true;
            }

            bool preprocess_manipulation_tp_t::valid_move(plan_t& plan, trajectory_t& path, const state_t* manip_start, const state_t* start, config_t & goal_config)
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

            int preprocess_manipulation_tp_t::similar_pose(state_t * pose)
            {
                for( unsigned i = 0; i < poses_set_length; ++i )
                    if( poses_set[i].equal(object_state_space, pose) )
                        return i;

                return -1;
            }
        }
    }
}