/**
 * @file replanning_controller.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/systems/controllers/consumer/replanning_controller.hpp"
#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

//DEBUG
#include "prx/utilities/math/2d_geometry/angle.hpp"

#include <iostream>

PLUGINLIB_EXPORT_CLASS(prx::sim::replanning_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        //using namespace simulation::communication;

        replanning_controller_t::replanning_controller_t()
        {
            plan_index = -1;
            sent_query = false;
            goal_radius = PRX_ZERO_CHECK;
        }

        replanning_controller_t::~replanning_controller_t()
        {
            child_state_space->free_point(future_end_state);
        }

        void replanning_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            PRX_DEBUG_COLOR("Replanning controller initialized", PRX_TEXT_CYAN);
            consumer_controller_t::init(reader, template_reader);
            //    child_state_space = subsystems.begin()->second.get()->get_state_space();
            //    smooth = parameters::get_attribute_as<bool>("smooth", reader, template_reader);
            std::vector<const parameter_reader_t*> goal_readers, template_goal_readers;

            goal_radius = goal_states[goal_index]->get_radius();
            current_goal_state = goal_states[goal_index]->get_goal_points().front();

            //    // Create the first plan (zero control plan)
            //    control_t* ctrl;
            //    ctrl= output_control_space->alloc_point();
            //    output_control_space->zero(ctrl);
            //    plan_step_t zero_control(ctrl, 0.001);
            //    zero_plan.steps.push_back(zero_control);
            //    zero_plan.input_control_space = output_control_space;
            //    zero_plan.end_state = child_state_space->alloc_point();
            future_end_state = child_state_space->alloc_point();
            child_state_space->copy_to_point(future_end_state);

            query_timer.reset();

        }

        void replanning_controller_t::propagate(const double simulation_step)
        {
            //    PRX_INFO_S ("Propagate");
            controller_t::propagate(simulation_step);

        }

        void replanning_controller_t::verify() const
        {
            PRX_ASSERT(controller_state_space->get_dimension() == 1);

            controller_t::verify();
        }

        void replanning_controller_t::copy_plan(const plan_t& inplan)
        {
            //    sys_clock_t timer;
            //    timer.reset();
               // PRX_INFO_S ("Replanning controller has received a plan! " << inplan.print());
            //    PRX_DEBUG_S ("What's mah state?" << state_space->print_point(init_state));
            //    PRX_DEBUG_S ("Previous future state: " << child_state_space->print_point(future_end_state, 3));
            child_state_space->copy_point(future_end_state, inplan.get_end_state());
               // PRX_ERROR_S ("Next future state: " << child_state_space->print_point(future_end_state, 3));
            queued_plans.push_back(inplan);
            sent_query = false;
            //    PRX_ERROR_S("Checking da plan: " << queued_plans.back().print(get_output_control_space()));
            //    double trash;
            //    if (plan.steps.empty())
            //    {
            //        PRX_WARN_S ("Plan finished! Getting new plan from queue");
            ////        std::cin >> trash;
            //        plan = queued_plans.front();
            //        queued_plans.pop_front();
            //    }
            //    
            //    got_plan = true;
            //    sent_query = false;
            //    PRX_WARN_S ("This took : " << timer.measure());

        }

        void replanning_controller_t::query_planner()
        {
            //    PRX_DEBUG_S("Query planner");
            // If we haven't reached our final destination
            if( goal_index < (int)goal_states.size() )
            {
                //        PRX_DEBUG_S("We're not done yet");
                // Get the current state
                child_state_space->copy_to_point(get_state);

                // Publish a ground truth message
                // ((comm::simulation_comm_t*)comm::sim_comm)->publish_ground_truth((*state_memory[0]), planning_node);

                // Make a start state based on our future end state (of the current plan)
                std::vector<double> start_state;
                for( unsigned i = 0; i < child_state_space->get_dimension(); i++ )
                {
                    start_state.push_back(future_end_state->at(i));
                }

                // Query planning
                double dist = child_state_space->distance(future_end_state, current_goal_state);
                if( dist > goal_radius )
                {
                               PRX_INFO_S ("Distance: " << dist << " vs. goal radius: " << goal_radius);
                    ((comm::planning_comm_t*)comm::plan_comm)->publish_planning_query(
                                                                                      start_state, goal_states[goal_index]->get_goal_vec(), goal_states[goal_index]->get_radius(), pathname, planning_node, true, smooth, homogeneous_setup, false);
                }
                else if( goal_index < (int)goal_states.size() - 1 )
                {
                    //            PRX_WARN_S ("Waypoint increase!");
                    //            double trash;
                    //            std::cin >> trash;
                    goal_index++;
                    goal_radius = goal_states[goal_index]->get_radius();
                    current_goal_state = goal_states[goal_index]->get_goal_points().front();
                    ((comm::planning_comm_t*)comm::plan_comm)->publish_planning_query(
                                                                                      start_state, goal_states[goal_index]->get_goal_vec(), goal_states[goal_index]->get_radius(), pathname, planning_node, true, smooth, homogeneous_setup, false);
                }
                else
                {
                    goal_index++;
                    PRX_DEBUG_COLOR("No more goals to plan for!", PRX_TEXT_CYAN);
                }
            }
            sent_query = true;
            query_timer.reset();
        }

        void replanning_controller_t::compute_control()
        {
                // child_state_space->copy_to_point(get_state);
               double dist = child_state_space->distance(get_state, current_goal_state);
               // if (dist <= goal_radius)
                    // PRX_INFO_S("dist: "<<dist);
               //     sent_query = false;
            // if( goal_index < (int)goal_states.size() )
            // {
                if( plan.size() == 0 )
                {
                    //        std::cin >> trash;
                    if( !queued_plans.empty() )
                    {
                        plan = queued_plans.front();
                        queued_plans.pop_front();
                        got_plan = true;
                    }
                }
            // }
            control_t* new_control;
            if( !sent_query )
                query_planner();
            //    PRX_INFO_S ("Compute control");
            if( !got_plan )
            {
                if( query_timer.measure() > 5 )
                    sent_query = false;
                new_control = contingency_plan.get_control_at(0);
            }
            else
            {
                new_control = plan.get_next_control(simulation::simulation_step);

                if( new_control != NULL )
                {
                    //            PRX_INFO_S("********consumer control : " << output_control_space->print_point(new_control));
                    output_control_space->copy_point(computed_control, new_control);

                    //            PRX_WARN_S("keep_last_control : " << keep_last_control << "  c size:" << plan.steps.size());
                    //            if(keep_last_control && plan.steps.size() == 0)
                    //            {            
                    //                last_control = new_control;
                    //    //            PRX_LOG_INFO("Plan size : %u || Holding control", plan.steps.size() );
                    //            }

                    *state_memory[0] = *state_memory[0] + simulation::simulation_step;
                }
                else
                {
                    //            PRX_WARN_S ("Contingency plan gooooo");
                    new_control = contingency_plan.get_control_at(0);
                }
            }


            //    const control_t* ctrl;
            //    
            ////    if(plan.steps.size() != 0)    
            ////        ctrl = plan.steps.front().control;
            ////    else 
            //    if(last_control != NULL && keep_last_control)
            //    {
            //        ctrl = last_control;
            //        got_plan = false;
            //    }
            //    else    
            //        ctrl = computed_control;            
            output_control_space->copy_from_point(new_control);
            subsystems.begin()->second->compute_control();

        }

    }
}