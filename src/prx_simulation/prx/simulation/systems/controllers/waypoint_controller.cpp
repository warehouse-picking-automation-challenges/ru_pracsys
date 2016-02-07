/**
 * @file waypoint_controller.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/systems/controllers/waypoint_controller.hpp"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(prx::sim::waypoint_controller_t, prx::sim::system_t);


namespace prx
{
    using namespace util;

    namespace sim
    {

        waypoint_controller_t::waypoint_controller_t()
        {
            simtime = 0.0;
            mode = 0;
            state_memory = boost::assign::list_of(&simtime);
            controller_state_space = new space_t("Time", state_memory);
        }

        waypoint_controller_t::~waypoint_controller_t() { }

        void waypoint_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            std::vector<const parameter_reader_t*> waypoint_readers;
            std::vector<double> wpmem;
            space_point_t *state;
            plan_t p;

            PRX_DEBUG_COLOR("Starting waypoint_controller_t::init()...", PRX_TEXT_GREEN);

            controller_t::init(reader, template_reader);
            child_system = subsystems.begin()->second;
            child_state_space = child_system->get_state_space();

            mode = parameters::get_attribute_as<int> ("mode", reader, template_reader, 0);

            if( parameters::has_attribute("waypoints", reader, template_reader) )
                waypoint_readers = parameters::get_list("waypoints", reader, template_reader);
            else
                PRX_FATAL_S("No list of waypoints found!");

            foreach(const parameter_reader_t* r, waypoint_readers)
            {
                wpmem = r->get_attribute_as< std::vector<double> >("state");
                state = child_state_space->alloc_point();
                child_state_space->set_from_vector(wpmem, state);
                waypoints.push_back(state);
            }

            //TODO: Make sure the child system really is on the first waypoint?

            p.link_control_space(output_control_space);
            loop.link_control_space(output_control_space);

            switch( mode )
            {
                case 0: //forward loop
                    for( size_t i = 0; i < waypoints.size() - 1; i++ )
                    {
                        p.clear();
                        child_system->steering_function(waypoints[i], waypoints[i + 1], p);
                        PRX_DEBUG_COLOR("Plan " << i << " in forward: " << p.print(), PRX_TEXT_CYAN);
                        loop += p;
                    }
                    p.clear();
                    child_system->steering_function(waypoints.back(), waypoints.front(), p);
                    PRX_DEBUG_COLOR("Loopback plan: " << p.print(), PRX_TEXT_CYAN);
                    loop += p;
                    break;

                case 1: //forward reverse
                    for( size_t i = 0; i < waypoints.size() - 1; i++ )
                    {
                        p.clear();
                        child_system->steering_function(waypoints[i], waypoints[i + 1], p);
                        PRX_DEBUG_COLOR("Plan " << i << " in forward: " << p.print(), PRX_TEXT_CYAN);
                        loop += p;
                    }

                    for( size_t i = waypoints.size() - 1; i > 0; i-- )
                    {
                        p.clear();
                        child_system->steering_function(waypoints[i], waypoints[i - 1], p);
                        PRX_DEBUG_COLOR("Plan " << i << " in reverse: " << p.print(), PRX_TEXT_CYAN);
                        loop += p;
                    }
                    break;

                default:
                    PRX_FATAL_S("Invalid mode set on waypoint controller!");
            }

            plan_length = loop.length();

            PRX_DEBUG_COLOR("Done with waypoint_controller_t::init()!", PRX_TEXT_LIGHTGRAY);
        }

        void waypoint_controller_t::propagate(const double simulation_step)
        {
            controller_t::propagate(simulation_step);
            simtime += simulation_step;
        }

        void waypoint_controller_t::compute_control()
        {
            control_t *control;

            control = loop.get_control_at(fmod(simtime, plan_length));
            output_control_space->copy_from_point(control);

            subsystems.begin()->second->compute_control();
        }

        void waypoint_controller_t::verify() const
        {
            controller_t::verify();
        }
    }
}
