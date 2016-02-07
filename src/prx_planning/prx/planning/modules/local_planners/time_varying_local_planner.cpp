/**
 * @file time_varying_local_planner.cpp
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
#include "prx/planning/modules/local_planners/time_varying_local_planner.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/definitions/random.hpp"


#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::time_varying_local_planner_t, prx::plan::local_planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace plan
    {

        void time_varying_local_planner_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            local_planner_t::init(reader, template_reader);
            num_controls = parameters::get_attribute_as<int>("num_controls", reader, template_reader, 1);
            lower_multiple = parameters::get_attribute_as<int>("lower_multiple", reader, template_reader, 50);
            upper_multiple = parameters::get_attribute_as<int>("upper_multiple", reader, template_reader, 500);
            trajs = new trajectory_t[num_controls];
            plans = new plan_t[num_controls];
        }

        void time_varying_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj, bool connect)
        {

            // // determinism checking code
            // plan_t plan1;
            // plan_t plan2;
            // trajectory_t traj1;
            // trajectory_t traj2;
            // trajectory_t traj3;
            // plan1.link_control_space(world_model->get_control_space());
            // plan2.link_control_space(world_model->get_control_space());


            // int random_number = uniform_int_random(lower_multiple, upper_multiple);
            // plan1.append_onto_back(random_number*simulation::simulation_step);
            // sampler->sample(world_model->get_control_space(), plan1.back().control);
            // plan2.append_onto_back(random_number*simulation::simulation_step);
            // sampler->sample(world_model->get_control_space(), plan2.back().control);
            // traj1.link_space(world_model->get_state_space());
            // traj2.link_space(world_model->get_state_space());
            // traj3.link_space(world_model->get_state_space());
            // this->propagate(start, plan1, traj1);
            // this->propagate(start, plan1, traj2);
            // PRX_INFO_S("FIRST RUN: "<<world_model->get_state_space()->print_point(traj1.back()));
            // PRX_INFO_S("SECOND RUN: "<<world_model->get_state_space()->print_point(traj2.back()));



            // traj2.clear();
            // plan1+=plan2;
            // this->propagate(traj1.back(), plan2, traj2);
            // this->propagate(start, plan1, traj3);
            // PRX_INFO_S("FIRST RUN: "<<world_model->get_state_space()->print_point(traj2.back()));
            // PRX_INFO_S("SECOND RUN: "<<world_model->get_state_space()->print_point(traj3.back()));

            // exit(0);






            int old_num_controls = num_controls;
            if( !connect )
            {
                num_controls = 1;
            }
            for( int i = 0; i < num_controls; i++ )
            {
                int random_number = uniform_int_random(lower_multiple, upper_multiple);
                plans[i].link_control_space(world_model->get_control_space());
                plans[i].clear();
                plans[i].append_onto_back(random_number*simulation::simulation_step);
                sampler->sample(world_model->get_control_space(), plans[i].back().control);
                trajs[i].link_space(world_model->get_state_space());
                this->propagate(start, plans[i], trajs[i]);
            }
            double dist = metric->distance_function(trajs[0][trajs[0].size() - 1], goal);
            unsigned index = 0;
            for( int i = 1; i < num_controls; i++ )
            {
                double new_dist = metric->distance_function(trajs[i][trajs[i].size() - 1], goal);
                if( new_dist < dist )
                {
                    dist = new_dist;
                    index = i;
                }
            }
            plan = plans[index];
            traj = trajs[index];

            num_controls = old_num_controls;
        }

        void time_varying_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, state_t* result, bool connect)
        {
            int old_num_controls = num_controls;
            if( !connect )
            {
                num_controls = 1;
            }
            for( int i = 0; i < num_controls; i++ )
            {
                plans[i].link_control_space(world_model->get_control_space());
                plans[i].link_state_space(world_model->get_state_space());
                plans[i].clear();
                int random_number = uniform_int_random(lower_multiple, upper_multiple);
                plans[i].link_control_space(world_model->get_control_space());
                plans[i].clear();
                plans[i].append_onto_back(random_number*simulation::simulation_step);
                sampler->sample(world_model->get_control_space(), plans[i].back().control);
                propagate_step(start, plans[i], result);
                plans[i].copy_end_state(result);
            }
            double dist = metric->distance_function(plans[0].get_end_state(), goal);
            unsigned index = 0;
            for( int i = 1; i < num_controls; i++ )
            {
                double new_dist = metric->distance_function(plans[i].get_end_state(), goal);
                if( new_dist < dist )
                {
                    dist = new_dist;
                    index = i;
                }
            }
            plan = plans[index];
            world_model->get_state_space()->copy_point(result, plans[index].get_end_state());
            num_controls = old_num_controls;
        }


    }
}