/**
 * @file validity_checker.cpp 
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

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        std::deque<std::pair<unsigned,unsigned> > queue;
        pluginlib::ClassLoader<validity_checker_t> validity_checker_t::loader("prx_planning", "prx::plan::validity_checker_t");

        validity_checker_t::validity_checker_t() : world_model(NULL) { }

        void validity_checker_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader) 
        {
            cost_function_name = parameters::get_attribute_as<std::string>("cost_function",reader,template_reader,"default_uniform");
        }

        bool validity_checker_t::is_valid(const trajectory_t& input)
        {
            // foreach(state_t* state,input)
            // {
            //     if(!is_valid(state))
            //         return false;
            // }
            // return true;
            queue.clear();
            unsigned min = 0;
            unsigned max = input.size()-1;
            queue.push_back(std::pair<unsigned,unsigned>(min,max));
            while(queue.size()!=0)
            {
                std::pair<unsigned,unsigned > interval = queue.front();
                queue.pop_front();
                min = interval.first;
                max = interval.second;
                unsigned index = (max+min)>>1;
                if( !is_valid(input[index]) )
                    return false;
                else if(min!=index)
                {
                    queue.push_back(std::pair<unsigned,unsigned>(min,index-1));
                    queue.push_back(std::pair<unsigned,unsigned>(index+1,max));
                }
                else if(max!=min)
                {
                    queue.push_back(std::pair<unsigned,unsigned>(index+1,max));
                }
            }
            // return is_valid(input,min,max);
            return true;

            // foreach(state_t* state,input)
            // {
            //     if(!is_valid(state))
            //         return false;
            // }
            // return true;
            // unsigned min = 0;
            // unsigned max = input.size()-1;
            // return is_valid(input,min,max);
        }
        bool validity_checker_t::is_valid(const trajectory_t& input, unsigned min, unsigned max)
        {
            unsigned index = (max+min)>>1;
            if(!is_valid(input[index]))
                return false;
            else if(max==min)
                return true;
            else if(min!=index)
                return is_valid(input,min,index-1) && is_valid(input,index+1,max);
            else
                return is_valid(input,index+1,max);
        }

        void validity_checker_t::valid_trajectory(const space_t* space, const trajectory_t& input, trajectory_t& result)
        {
            result.clear();

            foreach(state_t* state, input)
            {
                if( !is_valid(state) )
                    return;
                result.copy_onto_back(state);
            }
        }

        void validity_checker_t::link_distance_function(util::distance_t dist)
        {
            if(cost_function_name=="default")
            {
                cost_function_name = "default_uniform";
            }
            cost_function = cost_function_t::get_loader().createUnmanagedInstance("prx_simulation/"+cost_function_name);
            cost_function->link_distance_function(dist);
            state_cost = cost_function->cost_of_state;
            trajectory_cost = cost_function->cost_of_trajectory;
            heuristic = cost_function->heuristic;
        }

        void validity_checker_t::valid_trajectory_prefix(const space_t* space, const trajectory_t& input, trajectory_t& result, unsigned int size)
        {
            unsigned int traj_size = 0;
            result.clear();

            foreach(state_t* state, input)
            {
                if( !is_valid(state) || traj_size == size )
                    return;
                result.copy_onto_back(state);
                traj_size++;
            }
        }

        void validity_checker_t::link_model(world_model_t* model)
        {
            world_model = model;
        }

        void validity_checker_t::valid_trajectory(trajectory_t& input)
        {

            unsigned int traj_size = 0;

            foreach(state_t* state, input)
            {
                if( !is_valid(state) )
                    break;
                traj_size++;
            }
            input.resize(traj_size);
        }

        pluginlib::ClassLoader<validity_checker_t>& validity_checker_t::get_loader()
        {
            return loader;
        }

    }
}

