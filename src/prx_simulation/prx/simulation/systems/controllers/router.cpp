/**
 * @file router.cpp
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

#include "prx/simulation/systems/controllers/router.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::router_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        router_t::router_t() { }

        router_t::~router_t() { }

        void router_t::construct_spaces()
        {
            if( output_control_space )
            {
                if( computed_control )
                    output_control_space->free_point(computed_control);
                delete output_control_space;
            }
            if( state_space )
                delete state_space;

            std::vector<const space_t*> spaces;
            for( unsigned i = 0; i < subsystem_names.size(); i++ )
            {
                spaces.push_back(subsystems[subsystem_names[i]]->get_control_space());
            }
            output_control_space = new space_t(spaces);
            for( unsigned i = 0; i < subsystem_names.size(); i++ )
            {
                control_map[subsystem_names[i]] = output_control_space->get_subspace(spaces[i]);
            }
            input_control_space = output_control_space;
            computed_control = output_control_space->alloc_point();


            spaces.clear();
            for( unsigned i = 0; i < subsystem_names.size(); i++ )
            {
                spaces.push_back(subsystems[subsystem_names[i]]->get_state_space());
            }
            state_space = new space_t(spaces, controller_state_space);
            for( unsigned i = 0; i < subsystem_names.size(); i++ )
            {
                state_map[subsystem_names[i]] = state_space->get_subspace(spaces[i]);
            }
        }

        void router_t::append_contingency(plan_t& result_plan, double duration)
        {
            //split the plan into sub plans for each subsystem

            foreach(std::string name, subsystem_names)
            {
                subplans[name].link_control_space(subsystems[name]->get_control_space());
            }

            foreach(std::string name, subsystem_names)
            {
                control_t* control = subsystems[name]->get_control_space()->alloc_point();

                foreach(plan_step_t step, result_plan)
                {
                    subplans[name].copy_onto_back(control, step.duration);
                    for( unsigned i = control_map[name].first; i < control_map[name].second; i++ )
                    {
                        subplans[name].back().control->at(i - control_map[name].first) = step.control->at(i);
                    }
                }
                subsystems[name]->get_control_space()->free_point(control);
            }

            //call append contingency on all subsystems

            foreach(std::string name, subsystem_names)
            {
                subsystems[name]->append_contingency(subplans[name], duration);
            }

            //reconstruct full plan: ASSUMES ONLY ONE PLAN STEP HAS BEEN ADDED
            double length = result_plan.length();
            control_t* control = input_control_space->alloc_point();
            result_plan.copy_onto_back(control, duration - length);
            input_control_space->free_point(control);

            foreach(std::string name, subsystem_names)
            {
                for( unsigned i = control_map[name].first; i < control_map[name].second; i++ )
                {
                    result_plan[result_plan.size() - 1].control->at(i) = subplans[name][subplans[name].size() - 1].control->at(i - control_map[name].first);
                }
            }

            foreach(std::string name, subsystem_names)
            {
                subplans[name].clear();
            }

        }

        void router_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
        {
            //convert the given states to the subsystem states
            hash_t<std::string, state_t*> start_states;
            hash_t<std::string, state_t*> goal_states;

            foreach(std::string name, subsystem_names)
            {
                subplans[name].link_control_space(subsystems[name]->get_control_space());
                start_states[name] = subsystems[name]->get_state_space()->alloc_point();
                goal_states[name] = subsystems[name]->get_state_space()->alloc_point();
                for( unsigned i = state_map[name].first; i < state_map[name].second; i++ )
                {
                    start_states[name]->at(i - state_map[name].first) = start->at(i);
                    goal_states[name]->at(i - state_map[name].first) = goal->at(i);
                }
            }

            //ask the subsystems for plans

            foreach(std::string name, subsystem_names)
            {
                subsystems[name]->steering_function(start_states[name], goal_states[name], subplans[name]);
            }


            //modify them so they are the correct length
            double length = 0;

            foreach(std::string name, subsystem_names)
            {
                double new_length = subplans[name].length();
                if( new_length > length )
                    length = new_length;
            }

            foreach(std::string name, subsystem_names)
            {
                double new_length = subplans[name].length();
                if( (length - new_length) > PRX_ZERO_CHECK )
                    subsystems[name]->append_contingency(subplans[name], length);
            }
            //combine them into a single plan
            result_plan.clear();
            std::set<double> cut_locations;
            cut_locations.insert(0);

            foreach(std::string name, subsystem_names)
            {
                double running_total = 0;

                foreach(plan_step_t step, subplans[name])
                {
                    running_total += step.duration;
                    cut_locations.insert(running_total);
                }
            }

            std::set<double>::iterator first = cut_locations.begin();
            std::set<double>::iterator second = cut_locations.begin();
            std::advance(second, 1);
            std::set<double>::iterator end = cut_locations.end();
            for(; second != end; first++, second++ )
            {
                if( (*second)-(*first) > PRX_ZERO_CHECK )
                {
                    control_t* temp_control = input_control_space->alloc_point();
                    result_plan.copy_onto_back(temp_control, *(second) - *(first));
                    input_control_space->free_point(temp_control);

                    foreach(std::string name, subsystem_names)
                    {
                        std::pair<unsigned,unsigned> control_local=control_map[name];
                        control_t* subcontrol = subplans[name].consume_control(result_plan.back().duration);
                        for( unsigned i = control_local.first; i < control_local.second; i++ )
                            result_plan.back().control->at(i) = subcontrol->at(i - control_local.first);
                    }
                }
            }

            //delete temporary memory

            foreach(std::string name, subsystem_names)
            {
                subsystems[name]->get_state_space()->free_point(start_states[name]);
                subsystems[name]->get_state_space()->free_point(goal_states[name]);
                subplans[name].clear();
            }

            return;
        }
    }
}
