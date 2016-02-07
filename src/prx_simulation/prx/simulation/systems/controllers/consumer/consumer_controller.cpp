/**
 * @file consumer_controller.cpp 
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

#include "prx/simulation/systems/controllers/consumer/consumer_controller.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

//DEBUG
#include "prx/utilities/math/2d_geometry/angle.hpp"

PLUGINLIB_EXPORT_CLASS(prx::sim::consumer_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        consumer_controller_t::consumer_controller_t()
        {
            _time = 0.0;
            state_memory = boost::assign::list_of(&_time);
            controller_state_space = new space_t("Time", state_memory);
            queried_planner = false;
            last_control = NULL;
            goal_index = 0;
        }

        consumer_controller_t::~consumer_controller_t()
        {

            foreach(radial_goal_region_t* goal, this->goal_states)
            {
                delete goal;
            }
            child_state_space->free_point(get_state);
        }

        void consumer_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            PRX_INFO_S("Consumer controller init");
            controller_t::init(reader, template_reader);

            // Assigns this consumer to a specific planning node
            planning_node = parameters::get_attribute_as<std::string > ("planning_node", reader, template_reader);

            // Set up communication with paired planning node
            ((comm::planning_comm_t*)comm::plan_comm)->create_new_query_topic(planning_node);
            ((comm::planning_comm_t*)comm::plan_comm)->create_planning_subscription(planning_node);

            // Booleans to determine behavior of the consumer
            deserialize_plan = parameters::get_attribute_as<std::string > ("deserialize_plan", reader, template_reader, "");
            keep_last_control = parameters::get_attribute_as<bool>("keep_last_control", reader, template_reader, true);
            keep_last_state = parameters::get_attribute_as<bool>("keep_last_state", reader, template_reader, false);
            active_queries = parameters::get_attribute_as<bool>("active_queries", reader, template_reader, false);
            smooth = parameters::get_attribute_as<bool>("smooth", reader, template_reader, false);
            homogeneous_setup = parameters::get_attribute_as<bool>("homogeneous_setup", reader, template_reader, true);
            if( homogeneous_setup )
            {
                PRX_DEBUG_COLOR("HOMOGENEOUS SETUP", PRX_TEXT_CYAN);
            }
            else
            {
                PRX_DEBUG_COLOR("NON-HOMOGENEOUS SETUP", PRX_TEXT_BLUE);
            }

            child_state_space = subsystems.begin()->second.get()->get_state_space();
            get_state = child_state_space->alloc_point();
            // Consumers that are active must read in a set of goals
            if( active_queries )
            {
                read_in_goals(reader, template_reader);
            }


            //    PRX_DEBUG_S (reader->trace() << " " <<  template_reader->trace());
            //    PRX_LOG_ERROR ("read in planning node: %s", planning_node.c_str());

            init_state = state_space->alloc_point();
            // create contingency plan
            contingency_plan.link_control_space(output_control_space);
            subsystems.begin()->second->append_contingency(contingency_plan, 0.0);
            //state_space->copy_to_point(init_state);

            // Reads in a plan from file (if specified)
            if( !deserialize_plan.empty() )
            {
                PRX_INFO_S("CONSUMER IS READING A PLAN");
                plan.link_control_space(output_control_space);
                std::ifstream fin;
                fin.open(deserialize_plan.c_str());
                plan.read_from_stream(fin);
            }

        }

        void consumer_controller_t::propagate(const double simulation_step)
        {

            controller_t::propagate(simulation_step);
            if( keep_last_state )
            {
                child_state_space->copy_to_point(get_state);
            }

        }

        //void consumer_controller_t::add_state_space_embedding( composite_space_t* sp )
        //{
        //    if( active && !dynamic_obstacle )
        //        subsystem->add_state_space_embedding( sp );
        //}
        //
        //void consumer_controller_t::add_control_space_embedding( composite_space_t* sp )
        //{
        //    if( active && !dynamic_obstacle )
        //        subsystem->add_control_space_embedding( sp );
        //}

        void consumer_controller_t::verify() const
        {
            PRX_ASSERT(controller_state_space->get_dimension() == 1);

            controller_t::verify();
        }

        void consumer_controller_t::copy_plan(const plan_t& inplan)
        {
            //            control_t* control = sys->pull_control_space()->alloc_point();
            //        for( unsigned int i = 0; i < control_msg.control.size(); ++i )
            //            sys->pull_control_space()->set_element(control, i, control_msg.control[i]);

            PRX_DEBUG_COLOR("Received plan", PRX_TEXT_CYAN);
            plan = inplan;
            queried_planner = false;
            //    state_space->copy_from_point(init_state);
            //    push_state(init_state);
        }

        void consumer_controller_t::compute_control()
        {

            //    PRX_DEBUG_COLOR ("Begin compute control", PRX_TEXT_BLUE);
            control_t* new_control = plan.get_next_control(simulation::simulation_step);
            if( active_queries && goal_index < (int)goal_states.size() && new_control == NULL && !queried_planner )
            {
                PRX_DEBUG_COLOR("Query planner", PRX_TEXT_MAGENTA);
                query_planner();
            }

            // Case 1: Our plan has given us a non-NULL control
            if( new_control != NULL )
            {
                //        PRX_DEBUG_COLOR("New control not NULL", PRX_TEXT_CYAN);
                //        PRX_INFO_S("********consumer control : " << output_control_space->print_point(new_control));
                output_control_space->copy_point(computed_control, new_control);

                //        PRX_WARN_S("keep_last_control : " << keep_last_control << "  c size:" << plan.steps.size());
                if( plan.size() == 0 )
                {
                    last_control = new_control;
                    //            PRX_LOG_INFO("Plan size : %u || Holding control", plan.steps.size() );
                }

                _time = _time + simulation::simulation_step;
            }
                // Case 2: Last control has been set to a valid control, and we want to keep our last control
            else if( last_control != NULL && keep_last_control )
            {
                //        PRX_DEBUG_COLOR("Use last control", PRX_TEXT_LIGHTGRAY);
                new_control = last_control;
            }
            else if( keep_last_state )
            {
                new_control = get_state;
            }
                // Case 3: If all else fails, use the contingency plan (system specific)
            else
            {
                //        PRX_DEBUG_COLOR("Contingency plan", PRX_TEXT_BROWN);
                new_control = contingency_plan.get_control_at(0);
            }

            //    if (queried_planner)
            //    {
            //        PRX_DEBUG_COLOR("Queried planner true!", PRX_TEXT_RED);
            //    }
            //    else
            //    {
            //        PRX_DEBUG_COLOR("Queried planner false!", PRX_TEXT_GREEN);
            //    }
            // Check if we still need to query planner


            //PRX_DEBUG_COLOR("New control: " << this->output_control_space->print_point(new_control), PRX_TEXT_MAGENTA);
            output_control_space->copy_from_point(new_control);
            subsystems.begin()->second->compute_control();

            //    PRX_DEBUG_COLOR("End control", PRX_TEXT_BLUE);
        }

        const space_t* consumer_controller_t::get_output_control_space() const
        {
            return output_control_space;
        }

        const hash_t<std::string, system_ptr_t >* consumer_controller_t::get_subsystems() const
        {
            return &subsystems;
        }

        void consumer_controller_t::query_planner()
        {
            //    PRX_DEBUG_S("Query planner");
            PRX_DEBUG_COLOR("Query planner", PRX_TEXT_BLUE);
            // If we haven't reached our final destination
            if( active_queries && goal_index < (int)goal_states.size() )
            {
                //        PRX_DEBUG_S("We're not done yet");
                PRX_DEBUG_COLOR("Goal_index: " << goal_index, PRX_TEXT_LIGHTGRAY);
                // Get the current state
                child_state_space->copy_to_point(get_state);
                //        state_space->copy_to_point(init_state);
                // Publish a ground truth message
                ((comm::simulation_comm_t*)comm::sim_comm)->publish_ground_truth((*state_memory[0]), planning_node, this->get_pathname());

                // Make a start state vector 
                std::vector<double> start_state;
                for( unsigned i = 0; i < child_state_space->get_dimension(); i++ )
                {
                    start_state.push_back(get_state->at(i));
                }

                // Query planning
                current_goal_state = goal_states[goal_index]->get_goal_points().front();
                ((comm::planning_comm_t*)comm::plan_comm)->publish_planning_query(
                                                                                  start_state, goal_states[goal_index]->get_goal_vec(), goal_states[goal_index]->get_radius(), pathname, planning_node, true, smooth, true, homogeneous_setup);
                goal_index++;
                queried_planner = true;
            }
            PRX_DEBUG_COLOR("END QUery planner", PRX_TEXT_BLUE);
        }

        // Private functions

        void consumer_controller_t::read_in_goals(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            std::vector<const parameter_reader_t*> goal_readers, template_goal_readers;

            if( reader->has_attribute("goals") )
                goal_readers = reader->get_list("goals");
            else
            {
                PRX_FATAL_S("No goals to read!");
            }

            std::string template_name;
            parameter_reader_t* child_template_reader = NULL;

            foreach(const parameter_reader_t* r, goal_readers)
            {
                PRX_DEBUG_COLOR("Goal reading!", PRX_TEXT_CYAN);
                radial_goal_region_t* goal_region = new radial_goal_region_t();
                if( r->has_attribute("goal/template") )
                {
                    template_name = r->get_attribute("goal/template");
                    // TODO: Find a way to load templates under namespaces more cleanly.
                    child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);

                }
                goal_region->init(r->get_child("goal").get(), child_template_reader);
                goal_region->link_space(child_state_space);
                goal_states.push_back(goal_region);


                if( child_template_reader == NULL )
                {
                    delete child_template_reader;
                    child_template_reader = NULL;
                }
            }

            if( template_reader )
            {
                if( template_reader->has_attribute("goals") )
                {
                    template_goal_readers = template_reader->get_list("goals");

                    foreach(const parameter_reader_t* r, goal_readers)
                    {
                        radial_goal_region_t* goal_region = new radial_goal_region_t();
                        if( r->has_attribute("goal/template") )
                        {
                            template_name = r->get_attribute("goal/template");
                            // TODO: Find a way to load templates under namespaces more cleanly.
                            child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);

                        }
                        goal_region->init(r->get_child("goal").get(), child_template_reader);
                        goal_region->link_space(child_state_space);
                        goal_states.push_back(goal_region);
                        if( child_template_reader == NULL )
                        {
                            delete child_template_reader;
                            child_template_reader = NULL;
                        }
                    }

                }
            }
        }

    }
}
