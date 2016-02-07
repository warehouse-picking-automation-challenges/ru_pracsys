/**
 * @file multigoal_controller.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Poorva Sampat, Meera Murti, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/systems/controllers/multigoal_controller.hpp"
//#include "simulation/controllers/VO_controller.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::multigoal_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

       multigoal_controller_t::multigoal_controller_t()
        {
            simtime = 0.0;
            goal_radius = 0.0;
            goal_index = 0;
            current_state = NULL;
        }

        multigoal_controller_t::~multigoal_controller_t() 
        { 
            if (current_state != NULL)
            {
                child_state_space->free_point(current_state);
            }
        }

        void multigoal_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            std::vector<const parameter_reader_t*> multigoal_readers;
            std::vector<double> wpmem;
            space_point_t *state;

           // PRX_DEBUG_COLOR("Starting multigoal_controller_t::init()...", PRX_TEXT_GREEN);

            controller_t::init(reader, template_reader);
            child_system = subsystems.begin()->second;
            child_state_space = child_system->get_state_space();

            if( parameters::has_attribute("goal_radius", reader, template_reader) )
                goal_radius = parameters::get_attribute_as<double>("goal_radius", reader, template_reader);
            
            if( parameters::has_attribute("multigoals", reader, template_reader) )
                multigoal_readers = parameters::get_list("multigoals", reader, template_reader);
            else
                PRX_FATAL_S("No list of multigoals found!");

            foreach(const parameter_reader_t* r, multigoal_readers)
            {
                wpmem = r->get_attribute_as< std::vector<double> >("state");
                state = child_state_space->alloc_point();
                for(unsigned i=0;i<wpmem.size();++i)
                {
                    PRX_DEBUG_COLOR(i<<"::"<<wpmem[i],PRX_TEXT_CYAN);
                }
                PRX_ASSERT(state!=NULL);
                child_state_space->set_from_vector(wpmem, state);
                multigoals.push_back(state);
               // PRX_DEBUG_COLOR("Multigoal: " << child_state_space->print_point(state), PRX_TEXT_BLUE);
            }
            //goal_index=multigoals.size();
            //
//            p.link_control_space(output_control_space);
//           // loop.link_control_space(output_control_space);
//
//           // plan_length = loop.length();
            current_state = child_state_space->alloc_point();   
            current_control = output_control_space->alloc_point();
            
            std::vector<double> state_vec;
            state_vec.resize(child_state_space->get_dimension());
            child_state_space->copy_point_to_vector(multigoals.front(), state_vec);
            output_control_space->copy_vector_to_point(state_vec,current_control);
            output_control_space->copy_from_point(current_control);
        }

//        void multigoal_controller_t::propagate(const double simulation_step)
//        {
//            
//            PRX_WARN_S ("Multigoal controller propagate ");
//            controller_t::propagate(simulation_step);
//            //subsystems.begin()->second->propagate(simulation_step);
//           
//            
//        }

        void multigoal_controller_t::compute_control()
        {
         //   PRX_WARN_S ("Multigoal controller compute");
            space_point_t *goal_state;
            child_state_space->copy_to_point(current_state);
            goal_state = multigoals.at(goal_index);    
            std::vector<double> state_vec;
            if(goal_index<multigoals.size()-1)
            {
              //  PRX_DEBUG_COLOR("goal "<<child_state_space->print_point(goal_state)<<" current state "<<child_state_space->print_point(current_state), PRX_TEXT_LIGHTGRAY);
                
                double dist = child_state_space->distance(current_state, goal_state);
                if(dist<goal_radius)
                {
                    goal_index++;
                    goal_state = multigoals.at(goal_index);
                    
                    state_vec.resize(child_state_space->get_dimension());
                    child_state_space->copy_point_to_vector(goal_state, state_vec);
                    output_control_space->copy_vector_to_point(state_vec, current_control);
                    output_control_space->copy_from_point(current_control);
                    PRX_DEBUG_S(output_control_space->print_point(current_control));
                }
            }
            else if (goal_index==multigoals.size()-1)
            {
                double dist = child_state_space->distance(current_state, goal_state);
                if(dist<goal_radius)
                {
                    
                    goal_state = multigoals.at(goal_index);
                    state_vec.resize(child_state_space->get_dimension());
                    state_vec.assign(2, 9999999);
                    child_state_space->copy_point_to_vector(goal_state, state_vec);
                    output_control_space->copy_vector_to_point(state_vec, current_control);
                    output_control_space->copy_from_point(current_control);
                    PRX_DEBUG_S(output_control_space->print_point(current_control));
                }
            }
           // current_control.
            subsystems.begin()->second->compute_control();
        }
    }
}
