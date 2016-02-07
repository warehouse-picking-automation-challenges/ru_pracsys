/**
 * @file rrt.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 **/

#include "prx/planning/motion_planners/rrtc/rrtc.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/utilities/goals/goal_state.hpp"
#include "prx/planning/modules/stopping_criteria/element/goal_criterion.hpp"
#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include "prx/utilities/distance_metrics/ann_metric/ann_distance_metric.hpp"
#include "prx/utilities/distance_metrics/graph_metric/graph_metric.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>

PLUGINLIB_EXPORT_CLASS(prx::plan::rrtc_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        rrtc_t::rrtc_t()
        {
            rrt[0] = new rrt_t;
            rrt[1] = new rrt_t;
            tree_a = 0;
            tree_b = 1;

            trees_met = false;

            rrt[0]->input_query = new motion_planning_query_t();
            rrt[1]->input_query = new motion_planning_query_t();
            tree_goals[0] = new goal_state_t();
            tree_goals[1] = new goal_state_t();
            rrt[0]->input_query->set_goal(tree_goals[0]);
            rrt[1]->input_query->set_goal(tree_goals[1]);
            rrt[0]->input_specification = new motion_planning_specification_t();
            rrt[1]->input_specification = new motion_planning_specification_t();

            crits[0] = new stopping_criteria_t();
            crits[1] = new stopping_criteria_t();

            rrt[0]->input_specification->set_stopping_criterion(crits[0]);
            rrt[1]->input_specification->set_stopping_criterion(crits[1]);

            rrt[1]->input_specification->metric = NULL;
        }

        rrtc_t::~rrtc_t()
        {
            delete rrt[0]->input_query;
            delete rrt[1]->input_query;

            delete tree_goals[0];
            delete tree_goals[1];

            if( rrt[1]->input_specification->metric != NULL )
            {
                delete rrt[1]->input_specification->metric;
            }

            delete crits[0];
            delete crits[1];

            delete rrt[0]->input_specification;
            delete rrt[1]->input_specification;

            delete rrt[0];
            delete rrt[1];
        }

        void rrtc_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            motion_planner_t::init(reader, template_reader);

            bool visualize_tree = parameters::get_attribute_as<bool>("visualize_tree", reader, template_reader, true);
            rrt[0]->visualize_tree = visualize_tree;
            rrt[1]->visualize_tree = visualize_tree;
            bool visualize_solution = parameters::get_attribute_as<bool>("visualize_solution", reader, template_reader, true);
            rrt[0]->visualize_solution = visualize_solution;
            rrt[1]->visualize_solution = visualize_solution;
            std::string visualization_solution_name = parameters::get_attribute_as<std::string > ("visualization_solution_name", reader, template_reader, ros::this_node::getName() + "/rrt/solutions");
            rrt[0]->visualization_solution_name = visualization_solution_name + "a";
            rrt[1]->visualization_solution_name = visualization_solution_name + "b";
            std::string graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "red");
            rrt[0]->graph_color = graph_color;
            rrt[1]->graph_color = "blue";
            std::string solution_color = parameters::get_attribute_as<std::string > ("solution_color", reader, template_reader, "green");
            rrt[0]->solution_color = solution_color;
            rrt[1]->solution_color = solution_color;

            std::string visualization_body = parameters::get_attribute_as<std::string > ("visualization_body", reader, template_reader, "");
            rrt[0]->visualization_body = visualization_body;
            rrt[1]->visualization_body = visualization_body;

            bool collision_checking = parameters::get_attribute_as<bool>("collision_checking", reader, template_reader, true);
            rrt[0]->collision_checking = collision_checking;
            rrt[1]->collision_checking = collision_checking;
            int max_points = parameters::get_attribute_as<int>("max_points", reader, template_reader, 100000);
            rrt[0]->max_points = max_points / 2;
            rrt[1]->max_points = max_points / 2;
            bool use_boost_random = parameters::get_attribute_as<bool>("use_boost_random", reader, template_reader, false);
            rrt[0]->use_boost_random = use_boost_random;
            rrt[1]->use_boost_random = use_boost_random;

            std::string visualization_tree_name = parameters::get_attribute_as<std::string> ("visualization_tree_name", reader, template_reader, ros::this_node::getName());
            rrt[0]->visualization_tree_name = visualization_tree_name + "/rrta/graph";
            rrt[1]->visualization_tree_name = visualization_tree_name + "/rrtb/graph";

            rrt[0]->radius_solution = false;
            rrt[1]->radius_solution = false;
        }

        void rrtc_t::reset()
        {
            rrt[0]->reset();
            rrt[1]->reset();

            trees_met = false;

            rrt[0]->input_specification->metric->clear();
            rrt[1]->input_specification->metric->clear();

            tree_a = 0;
            tree_b = 1;
        }

        void rrtc_t::setup()
        {
            //Here, we need to transform our input specification/query into specifications/queries for each RRT... tree 0 is simple, but...
            rrt[0]->input_specification->validity_checker = input_specification->validity_checker;
            rrt[0]->input_specification->sampler = input_specification->sampler;
            rrt[0]->input_specification->metric = input_specification->metric;
            rrt[0]->input_specification->local_planner = input_specification->local_planner;
            rrt[0]->input_specification->state_space = state_space;
            rrt[0]->input_specification->control_space = control_space;

            //Setting up tree 1 is, on the other hand...
            rrt[1]->input_specification->validity_checker = input_specification->validity_checker;
            rrt[1]->input_specification->sampler = input_specification->sampler;
            rrt[1]->input_specification->local_planner = input_specification->local_planner;
            rrt[1]->input_specification->state_space = state_space;
            rrt[1]->input_specification->control_space = control_space;

            //If we have never allocated a distance metric, let's go ahead and allocate some for the RRT's
            if( rrt[0]->input_specification->metric == NULL || rrt[1]->input_specification->metric == NULL )
            {
                //Have to make one of the same type, with basically the same parameters
                //This is a dirty way to do it, but I need to see what type of metric we are using here.
                if( dynamic_cast<linear_distance_metric_t*>(input_specification->metric) )
                {
                    //Using a linear metric, create a linear metric for tree 1
                    rrt[0]->input_specification->metric = new linear_distance_metric_t();
                    rrt[1]->input_specification->metric = new linear_distance_metric_t();
                }
                else if( dynamic_cast<ann_distance_metric_t*>(input_specification->metric) )
                {
                    //Using an ANN metric, create an ANN metric for tree 1
                    rrt[0]->input_specification->metric = new ann_distance_metric_t();
                    rrt[1]->input_specification->metric = new ann_distance_metric_t();
                }
                else if( dynamic_cast<graph_distance_metric_t*>(input_specification->metric) )
                {
                    //Using a graph metric, create a graph metric for tree 1
                    rrt[0]->input_specification->metric = new graph_distance_metric_t();
                    rrt[1]->input_specification->metric = new graph_distance_metric_t();
                }
                else
                {
                    PRX_ERROR_S("Somehow, an unknown distance metric (or NULL) has been provided, abort.");
                }
            }

            //Give the distance function to the newly generated metric as well.
            rrt[0]->input_specification->metric->distance_function = input_specification->metric->distance_function;
            rrt[1]->input_specification->metric->distance_function = input_specification->metric->distance_function;

            //Things..
            tree_goals[0]->link_metric(input_specification->metric); //THIS is the line that's breaking...? o_O
            tree_goals[0]->link_space(state_space);
            rrt[0]->link_specification(rrt[0]->input_specification);

            //The metric needs a distance function.... Chuples, you did not think this through I think.
            tree_goals[1]->link_metric(rrt[1]->input_specification->metric);
            tree_goals[1]->link_space(state_space);
            rrt[1]->link_specification(rrt[1]->input_specification);
        }

        void rrtc_t::link_query(query_t* new_query)
        {
            //Make sure we do the motion planning query linking...
            motion_planner_t::link_query(new_query);

            rrt[0]->input_query->link_spaces(state_space, control_space);
            //Tree 0 starts at the actual start
            space_point_t* start = input_query->get_start_state();
            rrt[0]->input_specification->clear_seeds();
            rrt[0]->input_specification->add_seed(start);
            rrt[0]->input_query->set_start(start);

            //Try to be safe in case someone has changed the statespace on us
            rrt[0]->input_specification->metric->link_space(input_query->state_space);

            rrt[1]->input_query->link_spaces(state_space, control_space);
            //Tree 1 starts at the goal state instead
            space_point_t* goal = input_query->get_goal()->get_goal_points()[0];
            rrt[1]->input_specification->clear_seeds();
            rrt[1]->input_specification->add_seed(goal);
            rrt[1]->input_query->set_start(goal);

            //Try to be safe in case someone has changed the statespace on us
            rrt[1]->input_specification->metric->link_space(input_query->state_space);

            tree_goals[0]->set_goal_state(goal);
            tree_goals[1]->set_goal_state(start);

            //Make sure we call the setup for the rrts. (NOTE we cannot do this in setup() as we want, because RRTc requires a query before it can be fully set-up)
            rrt[0]->setup();
            rrt[1]->setup();
        }

        void rrtc_t::step()
        {
            //Stack variables
            bool node_added = true;
            unsigned last_num = 0;
            //PRX_DEBUG_COLOR("RRTc step() " << tree_a << ", " << tree_b, PRX_TEXT_MAGENTA);
            //If the trees have already met, rrtc has no defined behavior, so abort
            if( trees_met )
            {
                return;
            }

            //Set up tree_a to have no goal bias
            rrt[tree_a]->goal_bias_rate = 0;
            //Step tree_a
            rrt[tree_a]->step();

            //If the tree has yet to add any points, we sha'n't continue
            if( rrt[tree_a]->point_number == 0 )
            {
                tree_a = tree_b;
                tree_b = (tree_b + 1) % 2;
                return;
            }

            //Set up tree_b to have 100% goal bias
            rrt[tree_b]->goal_bias_rate = 1;
            //Set tree_b's goal to tree_a's newest addition.
            space_point_t* new_state = rrt[tree_a]->pre_alloced_points[rrt[tree_a]->point_number - 1];
            tree_goals[tree_b]->set_goal_state(new_state); //Tree B has the appropriate goal now
            //PRX_DEBUG_COLOR("Tree[" << tree_a << "] added state:   " << state_space->print_point(new_state, 9), PRX_TEXT_MAGENTA);
            //While new nodes keep getting added
            last_num = rrt[tree_b]->point_number;
            while( node_added )
            {
                //Step tree_b
                rrt[tree_b]->step();
                //If nothing was added
                if( rrt[tree_b]->point_number == last_num )
                {
                    //Abort out, the tree has stopped making progress
                    //PRX_DEBUG_COLOR("Nothing got added by " << tree_b, PRX_TEXT_BROWN);
                    node_added = false;
                    if( rrt[tree_b]->point_number > 0 )
                    {
                        //PRX_DEBUG_COLOR(tree_b<< " tree has point_number > 0 as "<<( rrt[tree_b]->point_number > 0 )<<" truth value",PRX_TEXT_BROWN);
                        space_point_t* reached_state = rrt[tree_b]->pre_alloced_points[rrt[tree_b]->point_number - 1];
                        //Then, check here if the trees have met
                        if( state_space->equal_points(new_state, reached_state) )
                        {
                            trees_met = true;
                            //And if they have met, update the goals appropriately
                            tree_goals[tree_a]->set_goal_state(reached_state); //Now Tree A has the appropriate goal
                        }
                    }
                }
                    //else, update our count and continue
                else
                {
                    //PRX_DEBUG_COLOR("Something got added by " << tree_b, PRX_TEXT_CYAN);
                    //                    space_point_t* reached_state = rrt[tree_b]->pre_alloced_points[rrt[tree_b]->point_number - 1];
                    //                    PRX_DEBUG_COLOR("Inside else " << state_space->print_point(reached_state, 4) << " reaching " << state_space->print_point(new_state, 4), PRX_TEXT_LIGHTGRAY);
                    last_num = rrt[tree_b]->point_number;
                }
            }
            //if( rrt[tree_b]->point_number > 0 )
            //{
                //PRX_DEBUG_COLOR("Tree[" << tree_b << "] reached state: " << state_space->print_point(rrt[tree_b]->pre_alloced_points[rrt[tree_b]->point_number - 1], 9), PRX_TEXT_CYAN);
            //}
            //else
            //{
                //PRX_DEBUG_COLOR("End of step with " << tree_b << "  adding 0 points", PRX_TEXT_CYAN);
            //}
            //Finally, swap the roles of the trees
            tree_a = tree_b;
            tree_b = (tree_b + 1) % 2;
        }

        bool rrtc_t::succeeded() const
        {
            return trees_met;
        }

        void rrtc_t::resolve_query()
        {
            tree_vertex_index_t new_v;
            tree_vertex_index_t parent_v;
            double cost = PRX_INFINITY;
            bool found = false;

            plan_t plan(input_query->plan);
            trajectory_t trajectory(input_query->path);
            plan.clear();
            trajectory.clear();

            state_t* last_state;

            if( !trees_met )
            {
                PRX_WARN_S("Resolving query when the trees have not connected.  No path to return...");
                return;
            }

            //Now, both trees should have goals, so run resolve query to generate the paths and plans
            rrt[0]->resolve_query();
            rrt[1]->resolve_query();

            //Add rrt[0]'s plan and trajectory
            input_query->path = rrt[0]->input_query->path;
            input_query->plan = rrt[0]->input_query->plan;
            last_state = input_query->path.back();

            //Then, we need to get the path out of rrt[1] but in reverse
            trajectory_t rev_traj;
            rev_traj.reverse_trajectory(rrt[1]->input_query->path);

            //Append rrt[1]'s trajectory
            input_query->path += rev_traj;

            //Now, we have to re-construct a plan through the second tree
            std::vector<space_point_t*> goals = rrt[1]->input_query->get_goal()->get_goal_points();
            std::vector<tree_vertex_index_t> v_goals;
            v_goals.clear();

            foreach(space_point_t* g, goals)
            {
                tree_vertex_index_t source = rrt[1]->nearest_vertex(g);
                v_goals.push_back(source);
            }

            foreach(tree_vertex_index_t v, v_goals)
            {
                if( rrt[1]->input_query->get_goal()->satisfied(rrt[1]->tree[v]->point) && rrt[1]->get_vertex(v)->cost < cost )
                {
                    found = true;
                    cost = rrt[1]->get_vertex(v)->cost;
                    new_v = v;
                }
            }
            v_goals.clear();
            v_goals.push_back(new_v);
            if( !found )
            {
                PRX_ERROR_S("Error, tree tracing for the reverse tree failed to find a goal");
            }
            new_v = v_goals[0];
            parent_v = rrt[1]->get_vertex(new_v)->get_parent();

            while( parent_v != new_v )
            {
                //Clear out the things
                plan.clear();
                trajectory.clear();
                //Do a steer
                local_planner->steer(rrt[1]->tree[new_v]->point, rrt[1]->tree[parent_v]->point, plan, trajectory, true);
                //And then append onto our plan
                input_query->plan += plan;

                new_v = rrt[1]->get_vertex(new_v)->get_parent();
                parent_v = rrt[1]->get_vertex(new_v)->get_parent();
            }
        }

        void rrtc_t::update_vis_info() const
        {
            //Assuming both trees have been set up with appropriate names, just call their update
            rrt[0]->update_vis_info();
            rrt[1]->update_vis_info();
        }

        bool rrtc_t::serialize()
        {
            PRX_WARN_S("Serialization not implemented for RRTc!");
            return false;
        }

        bool rrtc_t::deserialize()
        {
            PRX_WARN_S("De-serialization not implemented for RRTc!");
            return false;
        }

        const statistics_t* rrtc_t::get_statistics()
        {
            return rrt[0]->get_statistics();
        }

    }
}



