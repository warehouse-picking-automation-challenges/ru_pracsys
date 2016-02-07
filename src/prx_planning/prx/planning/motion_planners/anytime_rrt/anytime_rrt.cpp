/**
 * @file anytime_rrt.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/motion_planners/anytime_rrt/anytime_rrt.hpp"
#include "prx/planning/motion_planners/rrt/rrt_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/utilities/statistics/svg_image.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>

PLUGINLIB_EXPORT_CLASS( prx::plan::anytime_rrt_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace svg;
    namespace plan
    {

        anytime_rrt_t::anytime_rrt_t()
        {
            img_count = 0;
            time_elapsed = 0;
        }

        anytime_rrt_t::~anytime_rrt_t() { }

        void anytime_rrt_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            rrt_t::init(reader, template_reader);
        }

        void anytime_rrt_t::resolve_query()
        {
            if(input_query->q_collision_type == motion_planning_query_t::PRX_NO_COLLISIONS)
            {
                input_query->path = solution_trajectory;
                input_query->plan = solution_plan;
                input_query->solution_cost = solution_cost;
            }
            else
            {
                
            }
        }

        void anytime_rrt_t::setup()
        {
            states_to_check.clear();
            trajectory.link_space(state_space);
            plan.link_control_space(control_space);
            //output variables
            solution_trajectory.link_space(state_space);
            solution_plan.link_control_space(control_space);
            solution_cost = PRX_INFINITY;
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

        void anytime_rrt_t::step()
        {
            //sample a state
            if(!choose_target())
                return;
            extend_to_target();
            iteration_count++;

            if(point_number > 10000)
            {
                purge_tree(start_vertex);
                point_number = 0;
            }
        }

        bool anytime_rrt_t::choose_target()
        {
            double goal_bias_rate = .1;
            if( roll_weighted_die(boost::assign::list_of(goal_bias_rate)(1 - goal_bias_rate), false) == 0 )
            {

                // This will ONLY check the first goal. 
                // TODO: Multiple goals?
                state_space->copy_point(sample_point, input_query->get_goal()->get_goal_points().front());
            }
            else
            {
                // Otherwise, our dice said to randomly sample
                sampler->sample(state_space, sample_point);
                int max_attempts = 100;
                int attempts = 0;
                while(cost_to_come(sample_point) + cost_to_go(sample_point) > solution_cost)
                {
                    sampler->sample(state_space, sample_point);
                    attempts++;
                    if(attempts>max_attempts)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
        void anytime_rrt_t::extend_to_target()
        {
            std::vector<tree_vertex_index_t> Z_near;
            Z_near = neighbors(sample_point);
            std::vector<double> sel_costs;
            foreach(tree_vertex_index_t v, Z_near)
            {
                sel_costs.push_back(sel_cost(v));
            }

            bool swap_happened = true;
            unsigned iters = 0;
            while(swap_happened)
            {
                swap_happened = false;
                for(unsigned i=0;i<sel_costs.size()-1-iters;i++)
                {
                    if(sel_costs[i] > sel_costs[i+1])
                    {
                        tree_vertex_index_t temp_v = Z_near[i];
                        double temp_cost = sel_costs[i];
                        Z_near[i] = Z_near[i+1];
                        sel_costs[i] = sel_costs[i+1];
                        Z_near[i+1] = temp_v;
                        sel_costs[i+1] = temp_cost;
                        swap_happened = true;
                    }
                }
                iters++;
            }

            foreach(tree_vertex_index_t z_near, Z_near)
            {
                plan.clear();
                trajectory.clear();
                tree_vertex_index_t nearest = z_near;
                local_planner->steer(tree[nearest]->point, sample_point, plan, trajectory, false);
                double traj_cost = validity_checker->trajectory_cost(trajectory);

                if(get_vertex(nearest)->cost + traj_cost + cost_to_go(trajectory.back()) < solution_cost &&
                    (!collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size() > 1)) )
                {
                    tree_vertex_index_t v = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
                    get_vertex(v)->point = pre_alloced_points[point_number];
                    point_number++;

                    state_space->copy_point(get_vertex(v)->point, trajectory[trajectory.size() - 1]);
                    get_vertex(v)->cost = get_vertex(nearest)->cost + traj_cost;

                    tree_edge_index_t e = tree.add_edge<rrt_edge_t > (nearest, v);
                    get_edge(e)->plan = plan;
                    get_edge(e)->trajectory = trajectory;

                    metric->add_point(tree[v]);
                    state_space->copy_point(states_to_check[0], tree[v]->point);
                    if( input_query->get_goal()->satisfied(states_to_check[0]) )
                    {
                        post_solution();
                        return;
                    }

                    if(point_number == max_points)
                    {
                        for(unsigned i=0;i<max_points;i++)
                        {
                            pre_alloced_points.push_back(state_space->alloc_point());
                        }
                        max_points*=2;
                    }
                    return;
                }
            }
        }

        void anytime_rrt_t::post_solution()
        {
            tree_vertex_index_t best_goal = nearest_vertex(input_query->get_goal()->get_goal_points()[0]);
            solution_cost = get_vertex(best_goal)->cost;
            solution_trajectory.clear();
            solution_plan.clear();

            plan_t hold_plan;
            hold_plan.link_control_space(control_space);
            tree_vertex_index_t new_v = best_goal;
            while( get_vertex(new_v)->get_parent() != new_v )
            {
                tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(), new_v);
                rrt_edge_t* edge = get_edge(e);
                
                hold_plan = edge->plan;
                hold_plan += solution_plan;
                solution_plan = hold_plan;
                new_v = get_vertex(new_v)->get_parent();
            }
            local_planner->propagate(tree[start_vertex]->point, solution_plan, solution_trajectory);
            time_elapsed+=clock.measure();
            clock.reset();
            PRX_INFO_S("New Solution: "<<solution_cost<<" at "<<time_elapsed);
            purge_tree(start_vertex);
            point_number = 0;

        }
        void anytime_rrt_t::purge_tree(util::tree_vertex_index_t v)
        {
            if(get_vertex(v)->get_children().size()!=0)
            {
                std::vector<tree_vertex_index_t> vs;
                for(std::list<tree_vertex_index_t>::const_iterator i=get_vertex(v)->get_children().begin();
                    i!=get_vertex(v)->get_children().end();
                    i++)
                {
                    vs.push_back(*i);
                }
                foreach(tree_vertex_index_t child, vs)
                {
                    purge_tree(child);
                }
            }
            prune_node(v);
        }

        void anytime_rrt_t::prune_node(util::tree_vertex_index_t v)
        {
            if(v!=start_vertex)
            {
                metric->remove_point(tree[v]);
                if(get_vertex(v)->get_parent()!=v)
                {
                    rrt_edge_t* edge = get_edge(tree.edge(get_vertex(v)->get_parent(),v));  
                    edge->plan.clear();
                }
                tree.remove_vertex(v);
            }

        }

        double anytime_rrt_t::sel_cost(util::tree_vertex_index_t v)
        {
            double Db = 1;
            double Cb = 0;
            return Db*metric->distance_function(get_vertex(v)->point,sample_point)+Cb*get_vertex(v)->cost;
        }
        double anytime_rrt_t::cost_to_go(sim::state_t* state)
        {
            return validity_checker->heuristic(state,input_query->get_goal()->get_goal_points()[0]);
        }
        double anytime_rrt_t::cost_to_come(sim::state_t* state)
        {
            return validity_checker->heuristic(tree[start_vertex]->point,state);
        }

        std::vector<tree_vertex_index_t> anytime_rrt_t::neighbors(state_t* state)
        {
            std::vector<const abstract_node_t*> nodes = metric->multi_query(state, 30);
            std::vector<tree_vertex_index_t> vertices;

            foreach(const abstract_node_t* node, nodes)
            {
                vertices.push_back(node->as<rrt_node_t > ()->get_index());
            }

            return vertices;
        }


        const statistics_t* anytime_rrt_t::get_statistics()
        {
            time_elapsed+=clock.measure();
            statistics = new rrt_statistics_t();
            statistics->as<rrt_statistics_t > ()->num_vertices = tree.num_vertices();
            statistics->as<rrt_statistics_t > ()->solution_quality = solution_cost;
            statistics->time = time_elapsed;//clock.measure();
            statistics->steps = iteration_count;

            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            std::stringstream s1;
            s1 << "/prx_output/" << this->path << "/";
            dir += (s1.str());
            boost::filesystem::path output_dir(dir);
            if( !boost::filesystem::exists(output_dir) )
            {
                boost::filesystem::create_directories(output_dir);
            }
            // double avg_cost = 0;
            // int count = 0;
            // foreach(tree_node_t* v, tree.vertices())
            // {
            //     avg_cost += get_vertex(v->get_index())->cost;
            //     count++;
            // }
            // avg_cost /= count;

            // ((rrt_star_statistics_t*)(statistics))->average_cost = avg_cost;
            PRX_INFO_S("Stats: "<<iteration_count<<" "<<tree.num_vertices()<<" "<<solution_cost );
            clock.reset();
            return statistics;
        }

    }
}
