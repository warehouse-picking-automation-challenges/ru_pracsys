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
 */

#include "prx/planning/motion_planners/rrt/rrt.hpp"
#include "prx/planning/motion_planners/rrt/rrt_statistics.hpp"

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


#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>

PLUGINLIB_EXPORT_CLASS( prx::plan::rrt_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        rrt_t::rrt_t()
        {
            iteration_count = 0;
            color_map = boost::assign::list_of("red")("blue")("green")("yellow")("black")("white");
            solution_number = 0;
        }

        rrt_t::~rrt_t()
        {
            state_space->free_point(sample_point);
            //    delete statistics;
            for( unsigned i = 0; i < max_points; i++ )
            {
                state_space->free_point(pre_alloced_points[i]);
            }
            reset();
        }

        void rrt_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            motion_planner_t::init(reader, template_reader);

            visualize_tree = parameters::get_attribute_as<bool>("visualize_tree", reader, template_reader, true);
            visualize_solution = parameters::get_attribute_as<bool>("visualize_solution", reader, template_reader, true);
            visualization_tree_name = parameters::get_attribute_as<std::string > ("visualization_tree_name", reader, template_reader, ros::this_node::getName() + "/rrt/graph");
            visualization_solution_name = parameters::get_attribute_as<std::string > ("visualization_solution_name", reader, template_reader, ros::this_node::getName() + "/rrt/solutions");
            graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "white");
            solution_color = parameters::get_attribute_as<std::string > ("solution_color", reader, template_reader, "green");
            visualization_body = parameters::get_attribute_as<std::string > ("visualization_body", reader, template_reader, "");
            radius_solution = parameters::get_attribute_as<bool>("radius_solution", reader, template_reader, false);
            collision_checking = parameters::get_attribute_as<bool>("collision_checking", reader, template_reader, true);
            max_points = parameters::get_attribute_as<int>("max_points", reader, template_reader, 100000);
            goal_bias_rate = parameters::get_attribute_as<double>("goal_bias_rate", reader, template_reader, 0.0);
            use_boost_random = parameters::get_attribute_as<bool>("use_boost_random", reader, template_reader, false);
        }

        void rrt_t::reset()
        {
            if( metric->get_nr_points() != 0 )
            {
                metric->clear();
            }
            tree.clear();
            iteration_count = 0;

        }

        void rrt_t::setup()
        {
            states_to_check.clear();
            trajectory.link_space(state_space);
            trajectory.resize(500);
            tree.pre_alloc_memory<rrt_node_t, rrt_edge_t > (max_points);
            states_to_check.push_back(state_space->clone_point(input_specification->get_seeds()[0]));
            sample_point = state_space->alloc_point();
            start_vertex = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
            tree[start_vertex]->point = state_space->clone_point(input_specification->get_seeds()[0]);
            PRX_DEBUG_S("Start state: " << state_space->print_point(tree[start_vertex]->point, 3));
            metric->add_point(tree[start_vertex]);
            clock.reset();
            if( pre_alloced_points.empty() )
            {
                for( unsigned i = 0; i < max_points; i++ )
                {
                    pre_alloced_points.push_back(state_space->alloc_point());
                }
            }
            point_number = 0;
            input_specification->get_stopping_criterion()->reset();

        }

        void rrt_t::step()
        {
            //    PRX_WARN_S ("RRT Step " << iteration_count);
            //sample a state
            //    if(goal_biased)
            {
                // We roll a weighted die to find out whether to bias towards the goal.
                // If you want replicatable results, set the second parameter to false.
                if( roll_weighted_die(boost::assign::list_of(goal_bias_rate)(1 - goal_bias_rate), use_boost_random) == 0 )
                {

                    // This will ONLY check the first goal. 
                    // TODO: Multiple goals?
                    state_space->copy_point(sample_point, input_query->get_goal()->get_goal_points().front());
                }
                else
                {
                    // Otherwise, our dice said to randomly sample
                    sampler->sample(state_space, sample_point);
                }
            }
            //    else
            //    {
            //        sampler->sample(state_space,sample_point);
            //    }
            //get the nearest state
            tree_vertex_index_t nearest = nearest_vertex(sample_point);
            if( state_space->equal_points(sample_point,tree[nearest]->point))
            {
                iteration_count++;
                return;
            }
            //propagate toward the sampled point
            plan_t plan;
            plan.link_control_space(control_space);
            //    plan.link_state_space(state_space);
            local_planner->steer(tree[nearest]->point, sample_point, plan, trajectory, false);
            //check if the trajectory is valid
            if( !collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size() > 1) )
            {
                tree_vertex_index_t v = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
                state_space->copy_point(pre_alloced_points[point_number], trajectory[trajectory.size() - 1]);
                tree[v]->point = pre_alloced_points[point_number]; //state_space->clone_point(traj.states.back());
                metric->add_point(tree[v]);
                tree.get_vertex_as<rrt_node_t>(v)->cost = tree.get_vertex_as<rrt_node_t>(nearest)->cost + validity_checker->trajectory_cost(trajectory);

                state_space->copy_point(states_to_check[0], tree[v]->point);

                tree_edge_index_t e = tree.add_edge<rrt_edge_t > (nearest, v);
                get_edge(e)->plan = plan;

                if( visualize_tree )
                {
                    get_edge(e)->trajectory = trajectory;
                }
                point_number++;
                if(point_number == max_points)
                {
                    for(unsigned i=0;i<max_points;i++)
                    {
                        pre_alloced_points.push_back(state_space->alloc_point());
                    }
                    max_points*=2;
                }
            }
            iteration_count++;
        }

        bool rrt_t::succeeded() const
        {
            if( input_specification->get_stopping_criterion()->satisfied() )
                return true;
            return false;
        }

        void rrt_t::resolve_query()
        {
            // solution_number++;
            std::vector<space_point_t*> goals = input_query->get_goal()->get_goal_points();

            std::vector<tree_vertex_index_t> v_goals;
            v_goals.clear();

            input_query->clear();
            if( radius_solution )
            {
                radial_goal_region_t* region = dynamic_cast<radial_goal_region_t*>(input_query->get_goal());

                foreach(space_point_t* g, goals)
                {
                    std::vector<const abstract_node_t*> source = metric->radius_query(g, region->get_radius());

                    foreach(const abstract_node_t* node, source)
                    {
                        v_goals.push_back(((const tree_node_t*)node)->get_index());
                    }
                }
            }
            else
            {

                foreach(space_point_t* g, goals)
                {
                    tree_vertex_index_t source = nearest_vertex(g);
                    v_goals.push_back(source);
                }
            }

            //At this point, the list of vertices that are candidate solutions have been found.
            //  Now, depending on the type of query requested, perform collision checking on the paths.
            
            if(input_query->q_collision_type == motion_planning_query_t::PRX_NO_COLLISIONS)
            {
                // PRX_WARN_S("RRT is resolving query without doing collision checking.");
                tree_vertex_index_t new_v;
                double cost = PRX_INFINITY;
                bool found = false;

                foreach(tree_vertex_index_t v, v_goals)
                {
                    if( input_query->get_goal()->satisfied(tree[v]->point) && get_vertex(v)->cost < cost )
                    {
                        found = true;
                        cost = get_vertex(v)->cost;
                        new_v = v;
                    }
                }
                v_goals.clear();
                v_goals.push_back(new_v);
                if( !found )
                    return;
                new_v = v_goals[0];

                while( get_vertex(new_v)->get_parent() != new_v )
                {
                    tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(), new_v);
                    rrt_edge_t* edge = get_edge(e);
                    
                    plan_t hold_plan(edge->plan);
                    hold_plan += input_query->plan;
                    input_query->plan = hold_plan;
                    new_v = get_vertex(new_v)->get_parent();
                }
                local_planner->propagate(tree[start_vertex]->point, input_query->plan, input_query->path);
            }
            else 
            {

            }
        }

        void rrt_t::update_vis_info() const
        {
            std::vector<geometry_info_t> geoms;
            std::vector<config_t> configs;
            hash_t<std::string, std::vector<double> > map_params;
            std::vector<double> params;
            PRX_WARN_S("Vis sol name: " << visualization_solution_name << " , tree name: " << visualization_tree_name);

            int count;
            if( visualize_tree )
            {
                PRX_WARN_S("Visualizing tree! " << visualization_body);
                count = 0;
                std::vector<std::string> system_names;
                system_names.push_back(visualization_body);

                foreach(tree_edge_t* e, tree.edges())
                {
                    std::string name = visualization_tree_name + "/edge_" + int_to_str(count);
                    params.clear();

                    foreach(state_t* state, get_edge(e->get_index())->trajectory)
                    {
                        map_params.clear();
                        ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(state, system_names, map_params);
                        params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                    }

                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, graph_color));
                    configs.push_back(config_t());

                    count++;
                }
                ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map[visualization_tree_name] = geoms;
                ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map[visualization_tree_name] = configs;
                geoms.clear();
                configs.clear();
            }


            if( visualize_solution )
            {
                PRX_WARN_S("Visualizing solution! " << visualization_body);
                std::vector<std::string> system_names;
                system_names.push_back(visualization_body);
                if( input_query->path.size() < 2 )
                {
                    //            return;
                }
                else
                {
                    params.clear();
                    for( size_t i = 0; i < input_query->path.size(); i++ )
                    {
                        map_params.clear();
                        ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(input_query->path[i], system_names, map_params);
                        params.insert(params.end(), map_params[visualization_body].begin(), map_params[visualization_body].end());
                        //params.back() += 3; //this is so the solution will be above    
                    }

                    std::string name = visualization_solution_name + "/" + visualization_body + "/path";//_" + int_to_str(solution_number);
                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, color_map[solution_number % color_map.size()]));
                    configs.push_back(config_t());
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map[visualization_solution_name] = geoms;
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map[visualization_solution_name] = configs;
                    geoms.clear();
                    configs.clear();
                }

            }
        }

        rrt_node_t* rrt_t::get_vertex(tree_vertex_index_t v) const
        {
            return tree.get_vertex_as<rrt_node_t > (v);
        }

        rrt_edge_t* rrt_t::get_edge(tree_edge_index_t e) const
        {
            return tree.get_edge_as<rrt_edge_t > (e);
        }

        tree_vertex_index_t rrt_t::nearest_vertex(state_t* state) const
        {
            return metric->single_query(state)->as<rrt_node_t > ()->get_index();
        }

        bool rrt_t::serialize()
        {
            PRX_WARN_S("Serialize in RRT is unimplemented");
            return false;
        }

        bool rrt_t::deserialize()
        {
            PRX_WARN_S("Deserialize in RRT is unimplemented");
            return false;
        }

        const statistics_t* rrt_t::get_statistics()
        {
            statistics = new rrt_statistics_t();
            statistics->as<rrt_statistics_t > ()->num_vertices = tree.num_vertices();
            statistics->as<rrt_statistics_t > ()->solution_quality = validity_checker->trajectory_cost(input_query->path);
            statistics->time = clock.measure();
            statistics->steps = iteration_count;

            return statistics;
        }

    }
}
