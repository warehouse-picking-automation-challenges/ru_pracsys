/**
 * @file ground_truth_query_application.cpp 
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

#include "prx/planning/applications/ground_truth_query_application.hpp"
#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/stopping_criteria/element/goal_criterion.hpp"

#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"

#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include "prx_simulation/query_msg.h"
#include "prx_simulation/state_msg.h"

#include <pluginlib/class_list_macros.h>

//---- temporary
#include "prx/planning/motion_planners/rrt/rrt_statistics.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace plan
    {

        PLUGINLIB_EXPORT_CLASS( prx::plan::ground_truth_query_application_t, prx::plan::planning_application_t)

        using namespace comm;

        ground_truth_query_application_t::ground_truth_query_application_t()
        {
            goal_metric = new linear_distance_metric_t();
            avg_construction_time = avg_planning_time = avg_resolve_time = avg_visualize_time = 0.0;
            total_construction_time = total_planning_time = total_resolve_time = total_visualize_time = 0.0;
            total_nodes = total_time = total_steps = 0.0;
            avg_number_nodes = avg_time = avg_steps = 0.0;
            num_queries = 0;
        }

        ground_truth_query_application_t::~ground_truth_query_application_t()
        {
            delete goal_metric;
        }

        void ground_truth_query_application_t::init(const parameter_reader_t* reader)
        {

            // PRX_DEBUG_S("Before init of planning application");
            planning_application_t::init(reader);
            // PRX_DEBUG_S("After init of planning application");
            // planning_application_t::initialize_spaces("ground_truth");


        }

        void ground_truth_query_application_t::execute()
        {
            this->root_task->link_specification(root_specifications[0]);
            this->root_task->setup();
            // if( !root_queries.empty() )
            // {
            //     this->root_task->link_query(root_queries[0]);
            //     this->root_task->execute();
            // PRX_DEBUG_S("HELLO FROM GROUND TRUTH EXECUTE");
            //     if( visualize )
            //     {
            //         PRX_INFO_S("Updating visualization geoms.");

            //         this->root_task->update_visualization();

            //         ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
            //     }

            //     if(dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan.length()>0)
            //         ((comm::planning_comm_t*)comm::plan_comm)->publish_plan(consumer_path, dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan);
            // }
            // else
            // {
            //     this->root_task->execute();
            //     if( visualize )
            //     {
            //         PRX_INFO_S("Updating visualization geoms.");

            //         this->root_task->update_visualization();

            //         ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
            //     }
            // }
        }

        void ground_truth_query_application_t::process_query_callback(const prx_simulation::query_msg& msg)
        {
            num_queries++;
            sys_clock_t timer_root_task, total_time_root_task;
            total_time_root_task.reset();
            PRX_ERROR_S("Process query callback!\n Consumer:" << msg.consumer << "\n Goal region radius: " << msg.goal_region_radius);
            for( unsigned i = 0; i < msg.goal.size(); i++ )
            {
                PRX_INFO_S("Goal : " << i << " is: " << msg.goal[i]);
            }
            double construction_time = 0, planning_time = 0, resolve_time = 0, visualize_time = 0;
            timer_root_task.reset();
            // Construct the query
            if( consumer_to_space_name_mapping.find(msg.consumer) == this->consumer_to_space_name_mapping.end() )
            {
                PRX_FATAL_S("Consumer to space name mapping has not been set for "<<msg.consumer<<" in planner structure input file!");
            }

            if(use_sys_mapping)
            {
                model->use_context("full_space");
                unsigned iter = msg.plant_locations.plant_paths.size();
                for (unsigned i = 0; i < iter; ++i)
                {
                    if(simulator_to_planning_sys_mapping.find(msg.plant_locations.plant_paths[i]) != simulator_to_planning_sys_mapping.end() )
                    {

                        std::vector<double> state_vector = msg.plant_locations.plant_states[i].elements;
                        std::string planning_system = simulator_to_planning_sys_mapping[msg.plant_locations.plant_paths[i]];
                        model->get_system(split_path(planning_system).second)->get_state_space()->set_from_vector(state_vector);
                        break;
                    }
                }
            }


            //---------------------------
            model->use_context(this->consumer_to_space_name_mapping[msg.consumer]);
            //---------------------------
            PRX_WARN_S("1 ************************ " << this->consumer_to_space_name_mapping[msg.consumer]);
            

            space_t* s_space = model->get_state_space();
            PRX_WARN_S("2 ************************ " << s_space->get_space_name());
            space_t* c_space = model->get_control_space();
            

            // Set start state
            PRX_WARN_S("3 ************************");
            s_space->set_from_vector(msg.start);
            //    state_t* start_state = s_space->alloc_point();

            //    PRX_ERROR_S ("Start state :" << s_space->print_point(start_state,3));

            // Set the goal and the distance metric
            PRX_WARN_S("4 ************************");
            radial_goal_region_t* new_goal = new radial_goal_region_t();

            //    new_metric->link_space(s_space);
            PRX_DEBUG_S("Set goal");
            new_goal->set_goal(goal_metric, msg.goal, msg.goal_region_radius);

            PRX_DEBUG_S("Creating motion planning query");
            // Create a new motion planning query
            motion_planning_query_t new_query;
            //    new_query.set_start(start_state);
            new_query.set_start_vec(msg.start);
            // PRX_FATAL_S("Now you have to use specification and not the query in order to get the stopping criterion!");
//            new_query.set_stopping_criterion(&s_criteria);
            new_query.set_goal(new_goal);
            new_query.link_spaces(s_space, c_space);

            // Query the planner underneath me
            PRX_DEBUG_S("Link query");
            root_task->link_query(&new_query);
            // PRX_DEBUG_S("Setup");
            // root_task->setup();
            construction_time = timer_root_task.measure_reset();
            
            try
            {
                PRX_INFO_S("Trying to excute root task");
                root_task->execute();
            }
            catch( stopping_criteria_t::stopping_criteria_satisfied e )
            {
                planning_time = timer_root_task.measure_reset();
                PRX_ERROR_S("Stopping criteria satisfied " << e.what());
                root_task->resolve_query();
                resolve_time = timer_root_task.measure_reset();
            }
            catch( stopping_criteria_t::interruption_criteria_satisfied e )
            {
                planning_time = timer_root_task.measure_reset();
                PRX_ERROR_S("Interruption criteria!");
                root_task->resolve_query();
                resolve_time = timer_root_task.measure_reset();
            }

            if( visualize )
            {
                       PRX_WARN_S("Updating visualization geoms.");

                this->root_task->update_visualization();

                ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                visualize_time = timer_root_task.measure_reset();
            }

            PRX_INFO_S(new_query.plan.print());
            ((comm::planning_comm_t*)comm::plan_comm)->publish_plan(consumer_path, new_query.plan);
            // const rrt_statistics_t* stats = dynamic_cast<const rrt_statistics_t*>(root_task->get_statistics());
            // total_construction_time += construction_time;
            // total_planning_time += planning_time;
            // total_resolve_time += resolve_time;
            // total_visualize_time += visualize_time;
            // total_nodes += stats->num_vertices;
            // total_steps += stats->steps;
            // total_time += total_time_root_task.measure();

            // avg_construction_time = total_construction_time / num_queries;
            // avg_number_nodes = total_nodes / num_queries;
            // avg_planning_time = total_planning_time / num_queries;
            // avg_resolve_time = total_resolve_time / num_queries;
            // avg_visualize_time = total_visualize_time / num_queries;
            // avg_steps = total_steps / num_queries;
            // avg_time = total_time / num_queries;
            // double total_time = avg_planning_time + avg_resolve_time + avg_visualize_time;
            // PRX_DEBUG_S("\n\nSTATS! \nQuery construction time: " << construction_time << "\n Planning time: " << planning_time << "\n Resolve time: " << resolve_time << "\n Visualize time:"
            //             << avg_visualize_time << "\n Number of RRT nodes: " << stats->num_vertices << ", Steps: " << stats->steps << ", Planning Time: " << stats->time << ", Total Time: " << total_time_root_task.measure());


            // PRX_ERROR_S("\n\nAVERAGED STATS! \nQuery construction time: " << avg_construction_time << "\n Planning time: " << avg_planning_time << "\n Resolve time: " << avg_resolve_time << "\n Visualize time:"
            //             << avg_visualize_time << "\n Number of RRT nodes: " << avg_number_nodes << ", Steps: " << avg_steps << ", Planning Time: " << avg_time << ", Total Time: " << total_time);

            // delete stats;

        }

        void ground_truth_query_application_t::process_ground_truth_callback(const prx_simulation::state_msg& msg)
        {
            // PRX_ERROR_S("Process ground truth callback!");
            // model->use_context("full_space");
            // state_t* ground_truth = model->get_state_space()->alloc_point();
            // model->get_state_space()->set_from_vector(sim_to_plan_state(msg.elements, "ground_truth"), ground_truth);
            // model->push_state(ground_truth);
            // model->get_state_space()->free_point(ground_truth);
        }

    }
}