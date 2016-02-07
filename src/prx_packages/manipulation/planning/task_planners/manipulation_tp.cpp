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

#include "planning/task_planners/manipulation_tp.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/distance_metrics/ann_metric/ann_distance_metric.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/statistics/statistics.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"

#include "planning/modules/samplers/manip_sampler.hpp"
#include "planning/problem_specifications/manipulation_specification.hpp"
#include "planning/queries/manipulation_query.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/biconnected_components.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>
#include "prx/planning/motion_planners/prm_star/prm_star.hpp"

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            manipulation_tp_t::manipulation_tp_t()
            {
                _manipulator = NULL;
                _object = NULL;
                manip_sampler = NULL;
                safe_state = NULL;

                pc_name_manipulator_only = "";
                pc_name_object_only = "";
                pc_name_manipulator_with_object = "";
                pc_name_manipulator_with_active_object = "";
                pc_name_transit_planning = "";
                pc_name_transfer_planning = "";

                serialize_grasped_graph = false;
                serialize_ungrasped_graph = false;
                deserialize_grasped_graph = false;
                deserialize_ungrasped_graph = false;
                graph_builder = false;

                char* w = std::getenv("PRACSYS_PATH");
                prx_dir = std::string(w);

                reach_phase = "reach_phase";
                transfer_phase = "transfer_phase";
                retract_phase = "retract_phase";
            }

            manipulation_tp_t::~manipulation_tp_t()
            {
                manip_state_space->free_point(manip_state);
                manip_state_space->free_point(safe_state);
                manip_state_space->free_point(released_point);

                mo_space->free_point(grasped_point);

                object_state_space->free_point(object_state);
                object_state_space->free_point(initial_pose);
                object_state_space->free_point(goal_pose);

                real_object_space->free_point(real_initial_state);

                manip_control_space->free_point(manip_ctrl);
                manip_control_space->free_point(safe_control);

                retract_plan.clear();
                retract_path.clear();
            }

            void manipulation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                task_planner_t::init(reader, template_reader);
                PRX_INFO_S("Initializing Manipulation task planner ...");

                if( parameters::has_attribute("pc_name_manipulator_only", reader, template_reader) )
                    pc_name_manipulator_only = parameters::get_attribute("pc_name_manipulator_only", reader, template_reader);
                else
                    PRX_WARN_S("Planning context for manipulator only is not specified!");

                if( parameters::has_attribute("pc_name_object_only", reader, template_reader) )
                    pc_name_object_only = parameters::get_attribute("pc_name_object_only", reader, template_reader);
                else
                    PRX_WARN_S("Planning context for object only is not specified!");

                if( parameters::has_attribute("pc_name_manipulator_with_object", reader, template_reader) )
                    pc_name_manipulator_with_object = parameters::get_attribute("pc_name_manipulator_with_object", reader, template_reader);
                else
                    PRX_WARN_S("Planning context for manipulator with cup is not specified!");

                if( parameters::has_attribute("pc_name_manipulator_with_active_object", reader, template_reader) )
                    pc_name_manipulator_with_active_object = parameters::get_attribute("pc_name_manipulator_with_active_object", reader, template_reader);
                else
                    PRX_WARN_S("Planning context for manipulator with cup is not specified!");

                if( parameters::has_attribute("pc_name_transit_planning", reader, template_reader) )
                    pc_name_transit_planning = parameters::get_attribute("pc_name_transit_planning", reader, template_reader);
                else
                    PRX_WARN_S("Planning context for real world is not specified!");

                if( parameters::has_attribute("pc_name_transfer_planning", reader, template_reader) )
                    pc_name_transfer_planning = parameters::get_attribute("pc_name_transfer_planning", reader, template_reader);
                else
                    PRX_WARN_S("Planning context for planning world is not specified!");

                ungrasped_name = parameters::get_attribute("ungrasped_name", reader, template_reader);
                grasped_name = parameters::get_attribute("grasped_name", reader, template_reader);

                std::string element;
                if( parameters::has_attribute("graph_specifications", reader, template_reader) )
                {
                    const parameter_reader_t* specification_template_reader = NULL;
                    element = "graph_specifications/" + ungrasped_name;
                    if( parameters::has_attribute(element, reader, template_reader) )
                    {
                        if( parameters::has_attribute(element + "/template", reader, template_reader) )
                        {
                            specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                        }
                        output_specifications[ungrasped_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, specification_template_reader, "");

                        if( specification_template_reader != NULL )
                        {
                            delete specification_template_reader;
                            specification_template_reader = NULL;
                        }
                    }
                    else
                    {
                        PRX_FATAL_S("Missing motion planning specification for the ungrasped graph!");
                    }

                    element = "graph_specifications/" + grasped_name;
                    if( parameters::has_attribute(element, reader, template_reader) )
                    {
                        if( parameters::has_attribute(element + "/template", reader, template_reader) )
                        {
                            specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                        }
                        output_specifications[grasped_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, specification_template_reader, "");
                        if( specification_template_reader != NULL )
                        {
                            delete specification_template_reader;
                            specification_template_reader = NULL;
                        }
                    }
                    else
                    {
                        PRX_FATAL_S("Missing motion planning specification for the grasped graph!");
                    }
                }

                //                if( parameters::has_attribute("graph_goals", reader, template_reader) )
                //                {
                //                    element = "graph_goals/ungrasped";
                //                    if( parameters::has_attribute(element, reader, template_reader) )
                //                        ungrasped_goal = (multiple_goal_states_t*)parameters::initialize_from_loader<goal_t > ("prx_utilities", reader, element, template_reader, element);
                //                    else
                //                        PRX_FATAL_S("Missing ungrasped goal_t variable!");
                //
                //                    element = "graph_goals/grasped";
                //                    if( parameters::has_attribute(element, reader, template_reader) )
                //                        grasped_goal = (multiple_goal_states_t*)parameters::initialize_from_loader<goal_t > ("prx_utilities", reader, element, template_reader, element);
                //                    else
                //                        PRX_FATAL_S("Missing grasped goal_t variable!");
                //                }

                if( parameters::has_attribute("phase_queries", reader, template_reader) )
                {
                    const parameter_reader_t* query_template_reader = NULL;
                    element = "phase_queries/reach_phase";
                    if( parameters::has_attribute(element, reader, template_reader) )
                    {
                        if( parameters::has_attribute(element + "/template", reader, template_reader) )
                        {
                            query_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                        }
                        output_queries[reach_phase] = parameters::initialize_from_loader<query_t > ("prx_planning", reader, element, query_template_reader, "");
                        if( query_template_reader != NULL )
                        {
                            delete query_template_reader;
                            query_template_reader = NULL;
                        }
                    }
                    else
                        PRX_FATAL_S("Missing " << element << " query!");

                    element = "phase_queries/transfer_phase";
                    if( parameters::has_attribute(element, reader, template_reader) )
                    {
                        if( parameters::has_attribute(element + "/template", reader, template_reader) )
                        {
                            query_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                        }

                        output_queries[transfer_phase] = parameters::initialize_from_loader<query_t > ("prx_planning", reader, element, query_template_reader, "");
                        if( query_template_reader != NULL )
                        {
                            delete query_template_reader;
                            query_template_reader = NULL;
                        }
                    }
                    else
                        PRX_FATAL_S("Missing " << element << " query!");

                    element = "phase_queries/retract_phase";
                    if( parameters::has_attribute(element, reader, template_reader) )
                    {
                        if( parameters::has_attribute(element + "/template", reader, template_reader) )
                        {
                            query_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                        }
                        output_queries[retract_phase] = parameters::initialize_from_loader<query_t > ("prx_planning", reader, element, query_template_reader, "");
                        if( query_template_reader != NULL )
                        {
                            delete query_template_reader;
                            query_template_reader = NULL;
                        }
                    }
                    else
                        PRX_FATAL_S("Missing " << element << " query!");
                }
                else
                    PRX_FATAL_S("Missing the initialization of the output queries!");


                visualize_graph = parameters::get_attribute_as<bool>("visualize_graph", reader, template_reader, false);

                if( parameters::has_attribute("manip_sampler", reader, template_reader) )
                    manip_sampler = static_cast<manip_sampler_t*>(parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "manip_sampler", template_reader, "manip_sampler"));
                else
                    PRX_WARN_S("Missing manipulation sampler attribute for manipulation task planner!");

            }

            void manipulation_tp_t::reset()
            {
                retract_path.clear();
                retract_plan.clear();
            }

            void manipulation_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
            }

            const statistics_t* manipulation_tp_t::get_statistics()
            {
                PRX_WARN_S("Get statistics for manipulation tp is not implemented!");
                return new statistics_t();
            }

            void manipulation_tp_t::link_specification(specification_t* new_spec)
            {
                task_planner_t::link_specification(new_spec);
                specs = static_cast<manipulation_specification_t*>(new_spec);
                retraction_config.set_position( 0, 0, -specs->retract_distance );
                retraction_config.set_xyzw_orientation( 0, 0, 0, 1 );
            }

            void manipulation_tp_t::link_query(query_t* new_query)
            {
                //                PRX_DEBUG_POINT("Linking Query");
                task_planner_t::link_query(new_query);
                manip_query = static_cast<manipulation_query_t*>(new_query);
            }

            void manipulation_tp_t::setup()
            {
                PRX_DEBUG_COLOR("Setup manipulation tp ...", PRX_TEXT_CYAN);

                detect_plants();

                context_flags true_flags(true, true);
                context_flags active_flags(true, false);

                util::hash_t<std::string, context_flags> mappings;
                mappings[_manipulator->get_pathname()] = true_flags;

                model->create_new_planning_context(pc_name_transit_planning, mappings, active_flags);

                mappings[_object->get_pathname()] = active_flags;
                model->create_new_planning_context(pc_name_manipulator_with_active_object, mappings);

                mappings[_object->get_pathname()].plannable = true;
                model->create_new_planning_context(pc_name_manipulator_with_object, mappings);

                model->create_new_planning_context(pc_name_transfer_planning, mappings, active_flags);

                mappings[_manipulator->get_pathname()].set(false, false);
                model->create_new_planning_context(pc_name_object_only, mappings);


                PRX_DEBUG_COLOR("manipulator :  " << _manipulator->get_pathname(), PRX_TEXT_RED);
                PRX_DEBUG_COLOR("object :  " << _object->get_pathname(), PRX_TEXT_RED);

                ////////////////////
                //                Testing the planning contexts.
                //                model->use_context(pc_name_transit_planning);
                //                PRX_DEBUG_COLOR("pc_name_transit_planning: (" << model->get_state_space()->get_dimension() << " , " << model->get_active_space()->get_dimension() << ") (" << model->get_state_space()->get_space_name() << " , " << model->get_active_space()->get_space_name(),PRX_TEXT_GREEN);
                //
                //                model->use_context(pc_name_transfer_planning);
                //                PRX_DEBUG_COLOR("pc_name_transfer_planning: (" << model->get_state_space()->get_dimension() << " , " << model->get_active_space()->get_dimension() << ") (" << model->get_state_space()->get_space_name() << " , " << model->get_active_space()->get_space_name(),PRX_TEXT_GREEN);
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
                model->use_context(pc_name_transfer_planning);
                mo_space = model->get_state_space();
                other_objects_space = model->get_active_space();

                model->use_context(pc_name_transit_planning);
                real_object_space = model->get_active_space();
                manip_state_space = model->get_state_space();
                manip_control_space = model->get_control_space();

                model->use_context(pc_name_object_only);
                object_state_space = model->get_state_space();

                //Allocating the helping state/control point variables.
                manip_state = manip_state_space->alloc_point();
                safe_state = manip_state_space->alloc_point();
                released_point = manip_state_space->alloc_point();
                grasped_point = mo_space->alloc_point();
                object_state = object_state_space->alloc_point();
                initial_pose = object_state_space->alloc_point();
                goal_pose = object_state_space->alloc_point();
                real_initial_state = real_object_space->alloc_point();

                manip_ctrl = manip_control_space->alloc_point();
                safe_control = manip_control_space->alloc_point();

                manip_control_vec.resize(manip_control_space->get_dimension());
                mo_control_vec.resize(mo_space->get_dimension());

                retract_path.link_space(manip_state_space);
                retract_plan.link_control_space(manip_control_space);

                if( manip_sampler == NULL )
                    manip_sampler = specs->manip_sampler;
                else
                    manip_sampler->link_info(_manipulator, manip_state_space, object_state_space, mo_space);

                specs->link_spaces(manip_state_space, manip_control_space);
                specs->setup(model);
                manip_state_space->copy_vector_to_point(specs->safe_position, safe_state);
                manip_control_space->copy_vector_to_point(specs->safe_position, safe_control);

                //Setup the output queries
                reach_query = dynamic_cast<motion_planning_query_t*>(output_queries[reach_phase]);
                reach_query->link_spaces(manip_state_space, manip_control_space);
                //For the reach_phase query the starting point will always be the safe state.
                reach_query->link_start(safe_state);
                //                reach_goal = dynamic_cast<multiple_goal_states_t*>(reach_query->get_goal());
                PRX_ASSERT(reach_query->q_collision_type == manipulation_query_t::PRX_ACTIVE_COLLISIONS_REPROPAGATE_EDGES);

                transfer_query = dynamic_cast<motion_planning_query_t*>(output_queries[transfer_phase]);
                transfer_query->link_spaces(mo_space, manip_control_space);
                transfer_goal = dynamic_cast<multiple_goal_states_t*>(transfer_query->get_goal());
                PRX_ASSERT(transfer_query->q_collision_type == manipulation_query_t::PRX_ACTIVE_COLLISIONS_REPROPAGATE_EDGES);

                retract_query = dynamic_cast<motion_planning_query_t*>(output_queries[retract_phase]);
                retract_query->link_spaces(manip_state_space, manip_control_space);
                //For the retraction face the safe state will be the initial state because we are going to use the reach function in order
                //to compute the path to retract from a pose. Then we will reverse the path.
                retract_query->link_start(safe_state);
                //                ((multiple_goal_states_t*)retract_query->get_goal())->set_goal_state(safe_state);
                PRX_ASSERT(retract_query->q_collision_type == manipulation_query_t::PRX_ACTIVE_COLLISIONS_REPROPAGATE_EDGES);

                //Setup Transit Motion Planner
                graph_specification = static_cast<motion_planning_specification_t*>(output_specifications[ungrasped_name]);
                graph_specification->validity_checker = specs->validity_checker;
                graph_specification->local_planner = specs->local_planner;
                graph_specification->sampler = specs->sampler;
                graph_specification->astar = specs->astar;
                graph_specification->link_spaces(manip_state_space, manip_control_space);
                graph_specification->setup(model);

                motion_planner_t* motion_planner;
                motion_planner = dynamic_cast<motion_planner_t*>(planners[ungrasped_name]);
                graph_specification->clear_seeds();
                graph_specification->get_stopping_criterion()->link_motion_planner(motion_planner);
                if( deserialize_ungrasped_graph )
                {
                    motion_planner->set_param("", "deserialize_flag", deserialize_ungrasped_graph);
                    motion_planner->set_param("", "deserialization_file", ungrasped_graph_file_name);
                }

                //Setup Transfer Motion Planner
                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[grasped_name]);
                graph_specification->validity_checker = specs->validity_checker;
                graph_specification->local_planner = specs->local_planner;
                ((manip_sampler_t*)graph_specification->sampler)->link_info(_manipulator, manip_state_space, object_state_space, mo_space);
                graph_specification->astar = specs->transfer_astar;
                graph_specification->link_spaces(mo_space, manip_control_space);
                graph_specification->setup(model);

                motion_planner = dynamic_cast<motion_planner_t*>(planners[grasped_name]);
                graph_specification->clear_seeds();
                graph_specification->get_stopping_criterion()->link_motion_planner(motion_planner);
                if( deserialize_grasped_graph )
                {
                    motion_planner->set_param("", "deserialize_flag", deserialize_grasped_graph);
                    motion_planner->set_param("", "deserialization_file", grasped_graph_file_name);

                }

                validate();
            }

            bool manipulation_tp_t::serialize()
            {
                bool serialized = false;
                if( serialize_ungrasped_graph )
                {
                    model->use_context(pc_name_manipulator_only);
                    motion_planner_t* motion_planner = dynamic_cast<motion_planner_t*>(planners[ungrasped_name]);
                    motion_planner->set_param("", "serialize_flag", serialize_ungrasped_graph);
                    motion_planner->set_param("", "serialization_file", ungrasped_graph_file_name);
                    motion_planner->serialize();
                    serialized = true;
                }
                if( serialize_grasped_graph )
                {
                    model->use_context(pc_name_manipulator_with_object);
                    motion_planner_t* motion_planner = dynamic_cast<motion_planner_t*>(planners[grasped_name]);
                    motion_planner->set_param("", "serialize_flag", serialize_grasped_graph);
                    motion_planner->set_param("", "serialization_file", grasped_graph_file_name);
                    motion_planner->serialize();
                    serialized = true;
                }

                return serialized;
            }

            bool manipulation_tp_t::deserialize()
            {
                if( deserialize_ungrasped_graph )
                {
                    model->use_context(pc_name_manipulator_only);
                    dynamic_cast<motion_planner_t*>(planners[ungrasped_name])->deserialize();
                }
                if( deserialize_grasped_graph )
                {
                    model->use_context(pc_name_manipulator_with_object);
                    dynamic_cast<motion_planner_t*>(planners[grasped_name])->deserialize();
                }
                return true;
            }

            bool manipulation_tp_t::succeeded() const
            {
                //if( input_specification->get_stopping_criterion()->satisfied() )
                return true;
                //return false;
            }

            bool manipulation_tp_t::execute()
            {
                //HERE HERE HERE - Do crazy debugging stuffs here!
                PRX_DEBUG_COLOR("Beginning crazy debugging stuffs!", PRX_TEXT_GREEN);

                PRX_FATAL_S("DONE!");


                model->use_context(pc_name_object_only);
                motion_planner_t* motion_planner;
                const prm_star_statistics_t* statistics;

                //Building the Transit graph.
                model->use_context(pc_name_manipulator_only);
                graph_specification = static_cast<motion_planning_specification_t*>(output_specifications[ungrasped_name]);
                motion_planner = dynamic_cast<motion_planner_t*>(planners[ungrasped_name]);

                graph_specification->clear_seeds();
                graph_specification->set_seeds(specs->get_transit_seeds());
                motion_planner->link_specification(graph_specification);
                motion_planner->setup();
                if( deserialize_ungrasped_graph )
                {
                    motion_planner->deserialize();
                }
                else
                {
                    try
                    {
                        motion_planner->execute();
                    }
                    catch( stopping_criteria_t::stopping_criteria_satisfied e )
                    {
                        statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
                        PRX_DEBUG_COLOR("Transit Graph statistics : |seed| : " << graph_specification->get_seeds().size() << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_GREEN);

                    }
                }
                if( serialize_ungrasped_graph )
                    motion_planner->serialize();

                //Building Transfer graph
                model->use_context(pc_name_manipulator_with_object);

                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[grasped_name]);
                motion_planner = dynamic_cast<motion_planner_t*>(planners[grasped_name]);

                graph_specification->clear_seeds();
                graph_specification->set_seeds(specs->get_transfer_seeds());
                motion_planner->link_specification(graph_specification);
                motion_planner->setup();
                if( deserialize_grasped_graph )
                {
                    motion_planner->deserialize();
                }
                else
                {
                    try
                    {
                        motion_planner->execute();
                    }
                    catch( stopping_criteria_t::stopping_criteria_satisfied e )
                    {
                        statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
                        PRX_DEBUG_COLOR("Trasfer Graph statistics : |seed| : " << graph_specification->get_seeds().size() << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_GREEN);

                    }
                }

                if( serialize_grasped_graph )
                    motion_planner->serialize();
                if( !graph_builder )
                    resolve_query();

                return true;
            }

            void manipulation_tp_t::resolve_query()
            {

                if( manip_query->mode == manipulation_query_t::PRX_TRANSIT )
                {
                    std::string old_context = model->get_current_context();
                    model->use_context(pc_name_transit_planning);
                    reach_query->clear();
                    reach_query->set_start(manip_query->get_start_state());
                    dynamic_cast<multiple_goal_states_t*>(reach_query->get_goal())->init_goal_states(manip_query->get_goal()->get_goal_points());
                    planners[ungrasped_name]->link_query(reach_query);
                    planners[ungrasped_name]->resolve_query();
                    if( reach_query->plan.size() == 0 )
                    {
                        add_null_path();
                    }
                    else
                    {
//                        manip_state_space->copy_from_point(manip_query->get_start_state());
//                        manip_state_space->copy_to_vector(manip_control_vec);
//                        manip_control_space->copy_vector_to_point(manip_control_vec, manip_ctrl);
//                        reach_query->plan.copy_onto_front(manip_ctrl, reach_query->plan[0].duration);
                        update_query_information(reach_query);
                    }
                    model->use_context(old_context);
                    return;
                }
                else if( manip_query->mode == manipulation_query_t::PRX_TRANSFER )
                {
                    std::string old_context = model->get_current_context();
                    model->use_context(pc_name_transfer_planning);
                    transfer_query->clear();
                    transfer_query->link_start(manip_query->get_start_state());
                    ((multiple_goal_states_t*)transfer_query->get_goal())->init_goal_states(manip_query->get_goal()->get_goal_points());
                    planners[grasped_name]->link_query(transfer_query);
                    planners[grasped_name]->resolve_query();
                    if( transfer_query->plan.size() == 0 )
                    {
                        add_null_path();
                    }
                    else
                    {
//                        mo_space->copy_from_point(manip_query->get_start_state());
//                        manip_state_space->copy_to_vector(manip_control_vec);
//                        manip_control_space->copy_vector_to_point(manip_control_vec, manip_ctrl);
//                        PRX_ASSERT(manip_ctrl->memory.back() == 1);
//                        transfer_query->plan.copy_onto_front(manip_ctrl, transfer_query->plan[0].duration);
                        update_query_information(transfer_query);
                    }
                    model->use_context(old_context);
                    return;
                }

                unsigned min_size = PRX_INFINITY;

                if( manip_query->start_pose.state == NULL )
                {
                    PRX_ASSERT(false);
                    manip_query->build_initial_pose(object_state_space);
                    compute_posible_grasps(manip_query->start_pose, specs->max_different_grasps, specs->max_tries);
                }

                if( manip_query->target_pose.state == NULL && (manip_query->mode == manipulation_query_t::PRX_FULL_PATH || manip_query->mode == manipulation_query_t::PRX_TRANSFER_PATH) )
                {
                    PRX_ASSERT(false);
                    manip_query->build_target_pose(object_state_space);
                    compute_posible_grasps(manip_query->target_pose, specs->max_different_grasps, specs->max_tries);
                }

                object_state_space->copy_from_point(manip_query->start_pose.state);
                manip_state_space->copy_from_point(safe_state);

                if( manip_query->mode == manipulation_query_t::PRX_FULL_PATH )
                {
                    manipulate(manip_query->start_pose, manip_query->target_pose);
                }
                else
                {
                    unsigned size;
                    if( manip_query->path_quality == manipulation_query_t::PRX_ALL_PATHS )
                    {
                        if( manip_query->mode == manipulation_query_t::PRX_REACH_PATH )
                        {
                            size = manip_query->start_pose.ungrasped_set.size();
                            for( unsigned i = 0; i < size; ++i )
                            {
                                if( reach(manip_query->start_pose, reach_query, i) )
                                {
                                    update_query_information(reach_query);
                                }
                                else
                                    add_null_path();
                            }
                        }
                        else if( manip_query->mode == manipulation_query_t::PRX_RETRACT_PATH )
                        {
                            size = manip_query->start_pose.ungrasped_set.size();
                            for( unsigned i = 0; i < size; ++i )
                            {
                                if( retract(manip_query->start_pose, retract_query, i) )
                                {
                                    update_query_information(retract_query);
                                }
                                else
                                    add_null_path();
                            }
                        }
                        else if( manip_query->mode == manipulation_query_t::PRX_TRANSFER_PATH )
                        {
                            for( unsigned i = 0; i < manip_query->start_pose.ungrasped_set.size(); ++i )
                            {
                                for( unsigned j = 0; j < manip_query->target_pose.ungrasped_set.size(); ++j )
                                {
                                    if( tranfer(manip_query->start_pose, manip_query->target_pose, transfer_query, i, j) )
                                    {
                                        update_query_information(transfer_query);
                                    }
                                    else
                                        add_null_path();
                                }
                            }
                        }
                    }
                    else if( manip_query->path_quality == manipulation_query_t::PRX_FIRST_PATH )
                    {
                        if( manip_query->mode == manipulation_query_t::PRX_REACH_PATH )
                        {
                            for( unsigned i = 0; i < manip_query->start_pose.ungrasped_set.size(); ++i )
                            {
                                if( reach(manip_query->start_pose, reach_query, i) )
                                {
                                    update_query_information(reach_query);
                                    break;
                                }
                            }
                        }
                        else if( manip_query->mode == manipulation_query_t::PRX_RETRACT_PATH )
                        {
                            for( unsigned i = 0; i < manip_query->start_pose.ungrasped_set.size(); ++i )
                            {
                                if( retract(manip_query->start_pose, retract_query, i) )
                                {
                                    update_query_information(retract_query);
                                    break;
                                }
                            }
                        }
                        else if( manip_query->mode == manipulation_query_t::PRX_TRANSFER_PATH )
                        {
                            for( unsigned i = 0; i < manip_query->start_pose.ungrasped_set.size(); ++i )
                            {
                                for( unsigned j = 0; j < manip_query->target_pose.ungrasped_set.size(); ++j )
                                {
                                    if( tranfer(manip_query->start_pose, manip_query->target_pose, transfer_query, i, j) )
                                    {
                                        update_query_information(transfer_query);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    else if( manip_query->path_quality == manipulation_query_t::PRX_BEST_PATH )
                    {
                        manip_query->plans.push_back(new plan_t());
                        manip_query->plans.back()->link_control_space(manip_control_space);
                        if( manip_query->mode == manipulation_query_t::PRX_REACH_PATH )
                        {
                            for( unsigned i = 0; i < manip_query->start_pose.ungrasped_set.size(); ++i )
                                if( reach(manip_query->start_pose, reach_query, i) && reach_query->plan.size() < min_size )
                                {
                                    min_size = reach_query->plan.size();
                                    update_query_information(reach_query, 0);
                                }
                        }
                        else if( manip_query->mode == manipulation_query_t::PRX_RETRACT_PATH )
                        {
                            for( unsigned i = 0; i < manip_query->start_pose.ungrasped_set.size(); ++i )

                                if( retract(manip_query->start_pose, retract_query, i) && retract_query->plan.size() < min_size )
                                {
                                    min_size = retract_query->plan.size();
                                    update_query_information(retract_query, 0);
                                }
                        }
                        else if( manip_query->mode == manipulation_query_t::PRX_TRANSFER_PATH )
                        {
                            for( unsigned i = 0; i < manip_query->start_pose.ungrasped_set.size(); ++i )
                                for( unsigned j = 0; j < manip_query->target_pose.ungrasped_set.size(); ++j )
                                    if( tranfer(manip_query->start_pose, manip_query->target_pose, transfer_query, i, j) && transfer_query->plan.size() < min_size )
                                    {
                                        min_size = transfer_query->plan.size();
                                        update_query_information(transfer_query, 0);
                                    }
                        }
                    }

                }
                if( manip_query->plans[0] != NULL && manip_query->plans[0]->size() > 0 )
                    manip_query->plan = *(manip_query->plans[0]);
            }

            bool manipulation_tp_t::reach(pose_t& pose, query_t* query, int grasp_index)
            {
                std::string old_context = model->get_current_context();
                model->use_context(pc_name_transit_planning);
                //                reach_goal->set_goal_state(pose.retracted_set[grasp_index]);
                graph_query = dynamic_cast<motion_planning_query_t*>(query);
                dynamic_cast<multiple_goal_states_t*>(graph_query->get_goal())->init_with_goal_state(pose.ungrasped_set[grasp_index]);
                graph_query->clear();
                planners[ungrasped_name]->link_query(graph_query);
                planners[ungrasped_name]->resolve_query();

                if( graph_query->plan.size() == 0 )
                {
                    model->use_context(old_context);
                    return false;
                }

                graph_query->plan.copy_onto_front(safe_control, graph_query->plan[0].duration);
//                graph_query->plan += pose.reaching_plans[grasp_index];
                model->use_context(old_context);
                return true;
            }

            bool manipulation_tp_t::retract(pose_t& pose, query_t* query, int grasp_index)
            {
                if( !reach(pose, query, grasp_index) )
                    return false;
                graph_query = dynamic_cast<motion_planning_query_t*>(query);

#ifndef NDEBUG
                //TODO: REMOVE this is for debugging only
                plan_t plan = graph_query->plan; //TODO: REMOVE this is for debugging only
                int plan_size = plan.size();
                //TODO: REMOVE this is for debugging only
#endif

                graph_query->plan.reverse();
#ifndef NDEBUG
                //////////////////////// START OF DEBUGGING ////////////////////
                //TODO: REMOVE this is for debugging only
                unsigned sz = plan.size();
                PRX_ASSERT(sz == (unsigned)plan_size);
                for( unsigned i = 0; i < sz; ++i )
                    PRX_ASSERT(manip_control_space->equal_points(plan[sz - i - 1].control, graph_query->plan[i].control));
                //////////////////////// END OF DEBUGGING ////////////////////
#endif
                return true;
            }

            bool manipulation_tp_t::tranfer(pose_t& start_pose, pose_t& target_pose, plan::query_t* query, int start_grasp_index, int target_grasp_index)
            {
                std::string old_context = model->get_current_context();
                model->use_context(pc_name_transfer_planning);
                graph_query = dynamic_cast<motion_planning_query_t*>(query);
                graph_query->clear();
                graph_query->link_start(start_pose.grasped_set[start_grasp_index]);
                ((multiple_goal_states_t*)graph_query->get_goal())->init_with_goal_state(target_pose.grasped_set[target_grasp_index]);
                planners[grasped_name]->link_query(graph_query);
                planners[grasped_name]->resolve_query();

                if( graph_query->plan.size() == 0 )
                {
                    model->use_context(old_context);
                    return false;
                }

                manip_state_space->copy_point_to_vector(start_pose.ungrasped_set[start_grasp_index], manip_control_vec);
                manip_control_space->copy_vector_to_point(manip_control_vec, manip_ctrl);
                manip_ctrl->memory.back() = 1;
                graph_query->plan.copy_onto_front(manip_ctrl, graph_query->plan[0].duration);
#ifndef NDEBUG
                unsigned sz = graph_query->plan.size();
                for( unsigned i = 0; i < sz; ++i )
                    PRX_ASSERT(graph_query->plan[i].control->memory.back() == 1);
#endif
                model->use_context(old_context);
                return true;
            }

            bool manipulation_tp_t::manipulate(pose_t& start_pose, pose_t& target_pose)
            {

                unsigned min_path_size = PRX_INFINITY;
                bool found_path = false;
                if( manip_query->path_quality == manipulation_query_t::PRX_BEST_PATH )
                {
                    manip_query->plans.push_back(new plan_t());
                    manip_query->plans.back()->link_control_space(manip_control_space);
                }
                for( unsigned i = 0; i < start_pose.ungrasped_set.size(); ++i )
                {
                    reach_query->clear();
                    if( reach(start_pose, reach_query, i) )
                    {
                        //                        path_length += reach_query->path.length();
                        PRX_DEBUG_COLOR("=====================", PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("==== AFter reach ====", PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("=====================", PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("Manip: " << manip_state_space->print_memory(6), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("Objects: " << real_object_space->print_memory(3), PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("Reached pose, plan length: " << reach_query->plan.size() << "\nGrasped position: " << manip_control_space->print_point(reach_query->plan.back().control, 4), PRX_TEXT_GREEN);

                        for( unsigned j = 0; j < target_pose.grasped_set.size(); ++j )
                        {
                            transfer_query->clear();
                            if( tranfer(start_pose, target_pose, transfer_query, i, j) )
                            {
                                PRX_DEBUG_COLOR("=====================", PRX_TEXT_GREEN);
                                PRX_DEBUG_COLOR("== AFter Transfer  ==", PRX_TEXT_LIGHTGRAY);
                                PRX_DEBUG_COLOR("=====================", PRX_TEXT_GREEN);
                                PRX_DEBUG_COLOR("Manip: " << manip_state_space->print_memory(6), PRX_TEXT_LIGHTGRAY);
                                PRX_DEBUG_COLOR("Objects: " << real_object_space->print_memory(3), PRX_TEXT_MAGENTA);
                                //                                path_length += transfer_query->path.length();

                                retract_query->clear();
                                if( retract(target_pose, retract_query, j) )
                                {

                                    PRX_DEBUG_COLOR("=====================", PRX_TEXT_GREEN);
                                    PRX_DEBUG_COLOR("=== AFter retract ===", PRX_TEXT_LIGHTGRAY);
                                    PRX_DEBUG_COLOR("=====================", PRX_TEXT_GREEN);
                                    PRX_DEBUG_COLOR("Manip: " << manip_state_space->print_memory(6), PRX_TEXT_LIGHTGRAY);
                                    PRX_DEBUG_COLOR("Objects: " << real_object_space->print_memory(3), PRX_TEXT_MAGENTA);
                                    //                                    path_length += retract_query->path.length();

                                    combine_queries(min_path_size);
                                    if( manip_query->path_quality == manipulation_query_t::PRX_FIRST_PATH )
                                        return true;

                                    found_path = true;
                                }
                                else
                                {
                                    if( manip_query->path_quality == manipulation_query_t::PRX_ALL_PATHS )
                                        add_null_path();
                                }
                            }
                            else
                            {
                                if( manip_query->path_quality == manipulation_query_t::PRX_ALL_PATHS )
                                    add_null_path();
                            }
                        }
                    }
                    else
                    {
                        if( manip_query->path_quality == manipulation_query_t::PRX_ALL_PATHS )
                            for( unsigned i = 0; i < target_pose.ungrasped_set.size(); ++i )
                                add_null_path();
                    }

                }

                if( !found_path )
                    PRX_DEBUG_COLOR("Manipulate path does NOT exist!!! ", PRX_TEXT_RED);

                return found_path;
            }

            void manipulation_tp_t::add_null_path()
            {
                manip_query->plans.push_back(NULL);
            }

            void manipulation_tp_t::update_query_information(plan::query_t* query, int index)
            {
                graph_query = dynamic_cast<motion_planning_query_t*>(query);

                manip_query->found_path = true;
                if( index != -1 )
                {
                    PRX_ASSERT(manip_query->path_quality == manipulation_query_t::PRX_BEST_PATH);
                    (*manip_query->plans[index]) = graph_query->plan;
                    return;
                }

                manip_query->plans.push_back(new plan_t());
                manip_query->plans.back()->link_control_space(manip_control_space);
                (*manip_query->plans.back()) = graph_query->plan;

            }

            void manipulation_tp_t::combine_queries(unsigned& min_path_size)
            {
                plan_t* plan = new plan_t();
                *plan = reach_query->plan;
                //                                    plan->copy_onto_back(face1_plan.back().control, 0.1);
                //                                    plan->back().control->memory.back() = 1;
                *plan += transfer_query->plan;
                //                                    plan->copy_onto_back(face2_plan.back().control, 0.1);
                //                                    plan->back().control->memory.back() = 0;
                *plan += retract_query->plan;
                //                                    plan->copy_onto_back(safe_control, 0.02);

                PRX_DEBUG_COLOR("Start of face1: " << manip_control_space->print_point(reach_query->plan[0].control, 4), PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("End of face1: " << manip_control_space->print_point(reach_query->plan.back().control, 4), PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("Start of face2: " << manip_control_space->print_point(transfer_query->plan[0].control, 4), PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR("End of face2: " << manip_control_space->print_point(transfer_query->plan.back().control, 4), PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR("Start of face3: " << manip_control_space->print_point(retract_query->plan[0].control, 4), PRX_TEXT_RED);
                PRX_DEBUG_COLOR("End of face3: " << manip_control_space->print_point(retract_query->plan.back().control, 4), PRX_TEXT_RED);

                manip_query->found_path = true;
                if( manip_query->path_quality == manipulation_query_t::PRX_BEST_PATH )
                {
                    if( plan->size() < min_path_size )
                    {
                        min_path_size = plan->size();
                        *manip_query->plans[0] = *plan;
                    }
                    delete plan;
                    return;
                }

                manip_query->plans.push_back(plan);
                manip_query->plans.back()->link_control_space(manip_control_space);

                //TODO: Do I really need that, or it was nice for debugging?
                //It should be work of the top task planner to do that.
                //For this project though this is the top task planner. In order to visualize
                //the path we need to do that.
                if( manip_query->path_quality == manipulation_query_t::PRX_FIRST_PATH )
                {
                    manip_query->plan = *plan;
                }
            }

            //The manipulation function to generate the plan and path in a specific environment which is preset. The start and goal pose indices form the query and the results are updated in the manipulation query which it accepts as argument.

            void manipulation_tp_t::set_param(const std::string& parameter_name, const boost::any& value)
            {
                PRX_DEBUG_COLOR("Set param from manipulation_tp : " << parameter_name, PRX_TEXT_MAGENTA);

                if( parameter_name == "serialize_flag" )
                {
                    serialize_grasped_graph = boost::any_cast<bool>(value);
                    serialize_ungrasped_graph = boost::any_cast<bool>(value);
                }
                else if( parameter_name == "deserialize_flag" )
                {
                    deserialize_grasped_graph = boost::any_cast<bool>(value);
                    deserialize_ungrasped_graph = boost::any_cast<bool>(value);
                }
                else if( parameter_name == "ungrasped_graph_file_name" )
                {
                    ungrasped_graph_file_name = boost::any_cast<std::string > (value);
                }
                else if( parameter_name == "grasped_graph_file_name" )
                {
                    grasped_graph_file_name = boost::any_cast<std::string > (value);
                }
                else if( parameter_name == "graph_builder" )
                {
                    graph_builder = boost::any_cast<bool>(value);
                }
            }

            void manipulation_tp_t::update_vis_info() const
            {

                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {

                    planner->update_visualization();
                }
            }

            void manipulation_tp_t::detect_plants()
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

                    if( _manipulator != NULL && _object != NULL )
                        return;
                }
            }

            void manipulation_tp_t::compute_posible_grasps(pose_t& pose, int number_of_grasps, int max_tries)
            {
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

                        retract_plan.reverse();
                        retract_plan.copy_onto_front(manip_ctrl, retract_plan[0].duration);
                        pose.reaching_plans.push_back(retract_plan);
                        pose.reaching_plans.back().reverse();
//                        pose.retracting_plans.push_back(retract_plan);
//                        pose.reaching_plans.push_back(plan_t());
//                        pose.reaching_plans.back().reverse_plan(retract_plan);
//                        PRX_ASSERT(pose.reaching_plans.back().size() == retract_plan.size());
                    }
                }

            }

            bool manipulation_tp_t::get_grasp(sim::state_t* point, const sim::state_t* pose_state, int max_tries)
            {
                int tries = -1;
                do
                {
                    ++tries;
                    if( tries >= max_tries )
                        return false;

                }

                while( !manip_sampler->sample_near_object(point, pose_state) || !is_valid_grasp(point) );
                return true;
            }

            bool manipulation_tp_t::is_valid_grasp(state_t* point)
            {
                config_t r_config = retraction_config;

                std::string old_context = model->get_current_context();
                model->use_context(pc_name_manipulator_with_object);
                if( !validity_checker->is_valid(point) )
                    return false;

                //point has to be a grasped point that correspond to both manipulator configuration
                //and the pose of the object that the manipulator grasping.
                mo_space->copy_from_point(point);
                manip_state_space->copy_to_point(released_point);
                object_state_space->copy_to_point(object_state);

                released_point->memory.back() = 0;
                model->use_context(pc_name_manipulator_with_active_object);
                if( validity_checker->is_valid(released_point) )
                {
                    //Because the manipulator should still be in the grasped state, this should be fine?
                    _manipulator->get_end_effector_configuration( tmp_config );
                    r_config.relative_to_global( tmp_config );

                    retract_path.clear();
                    retract_plan.clear();
                    if( valid_move(retract_plan, retract_path, released_point, released_point, r_config) )
                    {
                        model->use_context(old_context);
                        return true;
                    }
                }
                model->use_context(old_context);
                return false;
            }

            bool manipulation_tp_t::valid_move(plan_t& plan, trajectory_t& path, const state_t* manip_start, const state_t* start, config_t & goal_config)
            {
                // PRX_DEBUG_COLOR("Valid_Move to " << goal_config.print(), PRX_TEXT_BLUE);
                config_t start_config;

                const space_t* m_space = model->get_state_space();
                //Alright, have to do a little dancing here to make sure we keep the right state
                m_space->copy_to_point( manip_state );
                m_space->copy_from_point( manip_start );

                //And make sure we get the config of the manipulator here
                _manipulator->get_end_effector_configuration( start_config );

                if( _manipulator->IK_steering(start_config, goal_config, plan) )
                {
                    //Restore the state
                    m_space->copy_from_point( manip_state );
                    // PRX_DEBUG_COLOR("Going to propagate : " << manip_state_space->serialize_point(manip_start, 5), PRX_TEXT_BROWN);
                    local_planner->propagate(start, plan, path);
                    if( path.size() != 0 && validity_checker->is_valid(path) )
                        return true;
                }
                //Restore the state
                m_space->copy_from_point( manip_state );
                return false;
            }

            bool manipulation_tp_t::validate()
            {
                bool is_valid = true;

                if( _manipulator == NULL )
                {
                    is_valid = false;
                    PRX_WARN_S("You need at least one manipulator for the project to work!!!");
                }

                if( _object == NULL )
                {
                    is_valid = false;
                    PRX_WARN_S("You need at least one movable object for the project to work!!!");
                }

                if( manip_sampler == NULL )
                {
                    is_valid = false;
                    PRX_WARN_S("Manipulation sampler is not specified!");
                }

                if( safe_state == NULL )
                {
                    is_valid = false;
                    PRX_WARN_S("Safe state is not specified!");
                }

                if( pc_name_manipulator_only == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The pc_name_manipulator_only is not specified!");
                }

                if( pc_name_object_only == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The pc_name_object_only is not specified!");
                }

                if( pc_name_manipulator_with_object == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The pc_name_manipulator_with_object is not specified!");
                }

                if( pc_name_manipulator_with_active_object == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The pc_name_manipulator_with_active_object is not specified!");
                }

                if( pc_name_transit_planning == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The pc_name_transit_planning is not specified!");
                }

                if( pc_name_transfer_planning == "" )
                {
                    is_valid = false;
                    PRX_WARN_S("The pc_name_transfer_planning is not specified!");
                }

                return is_valid;
            }
        }
    }
}
