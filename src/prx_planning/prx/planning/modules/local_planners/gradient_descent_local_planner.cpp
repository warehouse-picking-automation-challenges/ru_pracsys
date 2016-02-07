/**
 * @file gradient_descent_local_planner.cpp
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

#include "prx/planning/modules/local_planners/gradient_descent_local_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::gradient_descent_local_planner_t, prx::plan::local_planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        void gradient_descent_local_planner_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            local_planner_t::init(reader, template_reader);
            accepted_radius = parameters::get_attribute_as<double>("accepted_radius", reader, template_reader);
            learning_rate = parameters::get_attribute_as<double>("learning_rate", reader, template_reader);
            attempts = parameters::get_attribute_as<unsigned>("num_controls", reader, template_reader, 20);
            max_multiple = parameters::get_attribute_as<unsigned>("max_multiple", reader, template_reader, 50);
        }

        void gradient_descent_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj, bool connect)
        {
            if( connect )
            {
                gradient_steer(start, goal, plan);
            }
            else
            {
                plan.clear();
                plan.append_onto_front(floor(max_multiple * rand() / RAND_MAX) * simulation::simulation_step);
                sampler->sample(world_model->get_control_space(), plan[0].control);
            }
            propagate(start, plan, traj);
        }

        void gradient_descent_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, state_t* result, bool connect)
        {
            if( connect )
            {
                gradient_steer(start, goal, plan);
            }
            else
            {
                plan.clear();
                plan.append_onto_front(floor(max_multiple * rand() / RAND_MAX) * simulation::simulation_step);
                sampler->sample(world_model->get_control_space(), plan[0].control);
            }

            propagate_step(start, plan, result);

        }

        void gradient_descent_local_planner_t::gradient_steer(const state_t* start, const state_t* goal, plan_t& plan)
        {
            //for now only going to do piecewise constant plans
            plan.clear();
            traj.link_space(world_model->get_state_space());
            plan.append_onto_front(max_multiple * simulation::simulation_step);
            const space_t* control_space = world_model->get_control_space();
            sampler->sample(control_space, plan[0].control);
            unsigned count = 0;
            std::vector<double> old_control(control_space->get_dimension());
            std::vector<double> test_control(control_space->get_dimension());
            std::vector<double> control_below(control_space->get_dimension());
            std::vector<double> control_above(control_space->get_dimension());
            std::vector<std::pair<double, double> > diffs(control_space->get_dimension());
            while( count < attempts )
            {
                traj.clear();
                plan[0].duration = max_multiple * simulation::simulation_step;
                propagate(start, plan, traj);
                unsigned num_sim_steps = 0;
                unsigned best_sim_step = 0;
                double best_dist = PRX_INFINITY;
                for( trajectory_t::iterator it = traj.begin(); it != traj.end(); it++ )
                {
                    num_sim_steps++;
                    if( metric->distance_function(*it, goal) < best_dist )
                    {
                        best_dist = metric->distance_function(*it, goal);
                        best_sim_step = num_sim_steps;
                    }
                }
                plan[0].duration = best_sim_step * simulation::simulation_step;
                if( best_dist < accepted_radius )
                {
                    return;
                }
                else
                {
                    state_t* state = world_model->get_state_space()->alloc_point();
                    for( unsigned i = 0; i < control_space->get_dimension(); i++ )
                    {
                        old_control[i] = plan[0].control->at(i);
                        control_below[i] = old_control[i] - .01 * (control_space->get_bounds()[i]->get_upper_bound() - control_space->get_bounds()[i]->get_lower_bound());
                        control_above[i] = old_control[i] + .01 * (control_space->get_bounds()[i]->get_upper_bound() - control_space->get_bounds()[i]->get_lower_bound());
                        diffs[i].first = diffs[i].second = 0;
                    }
                    for( unsigned i = 0; i < control_space->get_dimension(); i++ )
                    {
                        test_control = old_control;
                        test_control[i] = control_below[i];
                        control_space->set_from_vector(test_control, plan[0].control);
                        propagate_step(start, plan, state);
                        diffs[i].first = metric->distance_function(state, goal);
                        test_control[i] = control_above[i];
                        control_space->set_from_vector(test_control, plan[0].control);
                        propagate_step(start, plan, state);
                        diffs[i].second = metric->distance_function(state, goal);
                    }
                    world_model->get_state_space()->free_point(state);

                    //now that all the differences have been computed, determine the direction to move
                    test_control = old_control;
                    for( unsigned i = 0; i < control_space->get_dimension(); i++ )
                    {
                        test_control[i] += (diffs[i].first - diffs[i].second)*(learning_rate);
                    }
                    control_space->set_from_vector(test_control, plan[0].control);
                }

                count++;
            }

            //    PRX_INFO_S(plan.print());

        }

        //trajectory_t gradient_descent_local_planner_t::propagate_step(const state_t* start, const control_t* control)
        //{
        //    trajectory_t trajectory = world_model->propagate(start,control,duration_step);
        //    return trajectory;
        //}
        //
        //double gradient_descent_local_planner_t::propagate_step(const state_t* start,const control_t* control, state_t* result) 
        //{
        //    PRX_LOG_ERROR("NOT IMPLEMENTED");
        //}
        //    
        //trajectory_t gradient_descent_local_planner_t::propagate(const state_t* start,const state_t* goal, control_t* & control)
        //{   
        //    std::vector<control_t*> controls;
        //    std::vector<trajectory_t> trajs;
        //    int num_tries = 4;
        //    for(int i=0;i<num_tries;i++)
        //    {
        //        controls.push_back(world_model->get_control_space()->alloc_point());
        //        sampler->sample(world_model->get_control_space(),controls[i]);
        //        trajs.push_back( two_n_propagate(start,goal,control,num_controls/num_tries));
        //    }
        //    
        //    double best_dist = PRX_INFINITY;
        //    int index=-1;
        //    for(int i=0;i<num_tries;i++)
        //    {
        //        double temp = metric->distance_function(goal,trajs[i].states.back());
        //        if(temp < best_dist)
        //        {
        //            best_dist = temp;
        //            if(index!=-1)
        //            {
        //                trajs[index].clear(world_model->get_state_space());
        //            }
        //            index = i;
        //        }
        //        else
        //        {
        //            trajs[i].clear(world_model->get_state_space());
        //        }
        //    }
        //    
        //    world_model->get_control_space()->copy_point(control,controls[index]);
        //    
        //    foreach(control_t* control, controls)
        //    {
        //        world_model->get_control_space()->free_point(control);
        //    }
        //    
        //    return trajs[index];
        //    
        //    
        //}
        //
        //trajectory_t gradient_descent_local_planner_t::improve_control(const state_t* start,const state_t* goal, control_t* & control)
        //{   
        //    return two_n_propagate(start,goal,control,num_controls);
        //}
        //
        //trajectory_t gradient_descent_local_planner_t::two_n_propagate(const state_t* start,const state_t* goal, control_t* & control, int num_things)
        //{
        //    double time_to_propagate = duration_step;
        //    
        //    std::vector<bounds_t*> control_bounds = world_model->get_control_space()->get_bounds();
        //    //PRX_WARN_S("Control before: "<<world_model->pull_control_space()->print_point(control));
        //    control_t* max_step = world_model->get_control_space()->clone_point(control);
        ////    element_iterator_t bound_iter = world_model->get_control_space()->get_element_iterator(max_step);
        //    
        //    trajectory_t trajectory;
        //    trajectory_t tempTrajectory;
        //    
        //    //get initial control to try
        //    control_t* sample_control;
        //    sample_control = world_model->get_control_space()->clone_point(control);
        //    control_t* previous_control = world_model->get_control_space()->clone_point(control);
        //    unsigned int control_size = world_model->get_control_space()->get_dimension();
        ////    if(with_time)
        ////    {
        ////        for(unsigned int i=0;i<control_size;i++)
        ////        {
        ////            bound_iter[i] = ((double)(control_bounds.get_upper_bound_at(i) - control_bounds.get_lower_bound_at(i) ))/(num_controls*4.0);
        ////        }
        ////    }
        ////    else
        ////    {
        //        for(unsigned int i=0;i<control_size;i++)
        //        {
        //            max_step->at(i) = ((double)(control_bounds[i]->get_upper_bound() - control_bounds[i]->get_lower_bound() ))/(num_controls*1.0);
        //        }
        //    //}
        //    
        //    trajectory = propagate_step(start,sample_control);
        //    tempTrajectory = propagate_step(start,sample_control);
        //    
        //    double first_cost = metric->distance_function(goal,trajectory.states.back());
        //            
        //    vector_t gradient(control_size);
        //    control_t* minus_epsilon;
        //    control_t* plus_epsilon;
        //    minus_epsilon = world_model->get_control_space()->alloc_point();
        //    plus_epsilon = world_model->get_control_space()->alloc_point();
        ////    element_iterator_t original_iter = world_model->pull_control_space()->get_element_iterator(sample_control);
        ////    element_iterator_t minus_iter = world_model->pull_control_space()->get_element_iterator(minus_epsilon);
        ////    element_iterator_t plus_iter = world_model->pull_control_space()->get_element_iterator(plus_epsilon);
        //    
        //    int num_iterations = 1;
        //    
        //    if(with_time)
        //    {
        //        num_iterations = 1;
        //           
        //    PRX_WARN_S("Control before: "<<world_model->get_control_space()->print_point(sample_control)<<" Distance: "<<first_cost<<" Time: "<<duration_step);
        //    }
        //    for(int outer = 0; outer < num_iterations; outer++)
        //    {
        //        
        //        if(with_time)
        //        {
        ////            double best_distance = PRX_INFINITY;
        ////            double best_time = 0;
        ////            for(double simulation_time = min_time_step ; simulation_time <= max_time_step; simulation_time+=sim_step )
        ////            {
        ////                double current_distance;
        ////                trajectory_t less_time = world_model->propagate(start,sample_control,simulation_time);
        ////                current_distance = metric->distance_function(goal,less_time.states.back());
        ////
        ////                if(current_distance < best_distance)
        ////                {
        ////                    best_distance = current_distance;
        ////                    best_time = simulation_time;
        ////                }
        ////
        ////                less_time.clear(world_model->pull_state_space());
        ////
        ////            }
        ////            time_to_propagate = best_time;
        //        }
        //
        //        for(int iter = 0 ; iter < num_things; iter++)
        //        {
        //            world_model->get_control_space()->copy_point(previous_control,sample_control);
        //            for(unsigned int i=0;i<control_size;i++)
        //            {
        //                for(unsigned int index=0;index<control_size;index++)
        //                {
        //                    if(index == i)
        //                    {
        //                        minus_epsilon->at(index) = sample_control->at(index) - max_step->at(i);
        //                        plus_epsilon->at(index) = sample_control->at(index) + max_step->at(i);
        //                    }
        //                    else
        //                    {
        //                        minus_epsilon->at(index) = sample_control->at(index);
        //                        plus_epsilon->at(index)  = sample_control->at(index);
        //                    }
        //                }
        //                //now test these directions
        //                double minus_cost;
        //                double plus_cost;
        //                
        //                
        //                world_model->propagate(start,minus_epsilon,time_to_propagate,tempTrajectory);
        //                minus_cost = metric->distance_function(goal,tempTrajectory.states.back());
        //
        //                //PRX_INFO_S("START: "<<world_model->pull_state_space()->print_point(start));
        //                //PRX_WARN_S("MINUS: "<<world_model->pull_control_space()->print_point(minus_epsilon)<<" Duration: "<<time_to_propagate<<" "<<tempTrajectory.states.size());
        //                //PRX_INFO_S("MINUS: "<<world_model->pull_state_space()->print_point(tempTrajectory.states.back()));
        //                
        //                world_model->propagate(start,plus_epsilon,time_to_propagate,tempTrajectory);
        //                plus_cost = metric->distance_function(goal,tempTrajectory.states.back());
        //                
        //                //PRX_INFO_S("PLUS: "<<world_model->pull_state_space()->print_point(tempTrajectory.states.back()));
        //                
        //                //PRX_WARN_S("PLUS: "<<world_model->pull_control_space()->print_point(plus_epsilon));
        //                //PRX_INFO_S("Plus: "<<plus_cost<<" Minus: "<<minus_cost);
        //                
        //                gradient[i] = (plus_cost - minus_cost)/(2*max_step->at(i));
        //                if(gradient[i] > 2*max_step->at(i))
        //                    gradient[i] = 2*max_step->at(i);
        //                else if(gradient[i] < -2*max_step->at(i) )
        //                    gradient[i] = -2*max_step->at(i);
        //
        //            }
        //
        //            for(unsigned int index=0;index<control_size;index++)
        //            {
        //                sample_control->at(index) += -1*gradient[index];
        //            }
        //            if(with_time)
        //            {
        //                PRX_WARN_S("Control between: "<<world_model->get_control_space()->print_point(sample_control)<<" Duration: "<<time_to_propagate);
        //                PRX_WARN_S("Gradient: "<<gradient);
        //            }
        //            if(!world_model->get_control_space()->satisfies_bounds(sample_control))
        //            {
        //                iter=num_controls;
        //                world_model->get_control_space()->copy_point(sample_control,previous_control);
        //            }
        //            if(gradient.norm() < .001)
        //            {
        //                break;
        //            }
        //        }
        //        
        //        world_model->propagate(start,sample_control,time_to_propagate,trajectory);
        //        if(with_time)
        //            PRX_WARN_S("Control after: "<<world_model->get_control_space()->print_point(sample_control)<<" Distance: "<<metric->distance_function(goal,trajectory.states.back())<<" Time: "<<time_to_propagate);
        //    }
        //    
        //    world_model->get_control_space()->copy_point(control,sample_control);
        //    world_model->propagate(start,sample_control,time_to_propagate,trajectory);
        //        
        //    tempTrajectory.clear(world_model->get_state_space());
        //    
        //    //PRX_WARN_S("Control after: "<<world_model->pull_control_space()->print_point(sample_control));
        //    world_model->get_control_space()->free_point(sample_control);
        //    world_model->get_control_space()->free_point(minus_epsilon);
        //    world_model->get_control_space()->free_point(plus_epsilon);
        //    world_model->get_control_space()->free_point(previous_control);
        //    
        //    return trajectory;
        //    
        //}
        //
        //
        //
    }
}