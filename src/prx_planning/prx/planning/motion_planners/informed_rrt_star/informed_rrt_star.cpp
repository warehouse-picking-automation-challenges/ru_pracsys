/**
 * @file informed_rrt_star.cpp
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

#include "prx/planning/motion_planners/informed_rrt_star/informed_rrt_star.hpp"
#include "prx/planning/motion_planners/rrt_star/rrt_star_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"
#include "prx/utilities/statistics/image.hpp"
#include "prx/utilities/statistics/svg_image.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

PLUGINLIB_EXPORT_CLASS( prx::plan::informed_rrt_star_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace svg;
    namespace plan
    {

        informed_rrt_star_t::informed_rrt_star_t()
        {
            img_count = 0;
            time_elapsed = 0;
            best_solution_cost = PRX_INFINITY;
        }

        informed_rrt_star_t::~informed_rrt_star_t() { }

        void informed_rrt_star_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            rrt_t::init(reader, template_reader);
        }

        void informed_rrt_star_t::hacked_resolve()
        {
                input_query->clear();
                tree_vertex_index_t best_goal = nearest_vertex(input_query->get_goal()->get_goal_points()[0]);
                input_query->solution_cost = get_vertex(best_goal)->cost;
        }
        void informed_rrt_star_t::resolve_query()
        {
            if(input_query->q_collision_type == motion_planning_query_t::PRX_NO_COLLISIONS)
            {
                input_query->clear();
                tree_vertex_index_t best_goal = nearest_vertex(input_query->get_goal()->get_goal_points()[0]);

                input_query->solution_cost = get_vertex(best_goal)->cost;

                local_planner->steer(get_vertex(best_goal)->point,input_query->get_goal()->get_goal_points()[0],input_query->plan,input_query->path,true);
                input_query->path.clear();
                tree_vertex_index_t new_v = best_goal;

                plan_t hold_plan;
                hold_plan.link_control_space(control_space);
                while( get_vertex(new_v)->get_parent() != new_v )
                {
                    tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(), new_v);
                    rrt_edge_t* edge = get_edge(e);
                    
                    hold_plan = edge->plan;
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

        void informed_rrt_star_t::step()
        {
            new_traj.link_space(state_space);
            //sample a state
            sampler->sample(state_space, sample_point);
            //get the nearest state
            tree_vertex_index_t nearest = nearest_vertex(sample_point);
            //propagate toward the sampled point

            plan_t plan;
            plan.link_control_space(control_space);
            local_planner->steer(tree[nearest]->point, sample_point, plan, trajectory, true);
            // PRX_INFO_S("Steering done: "<<iteration_count);
            //check if the trajectory is valid
            if( !collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size() > 1) )
            {
                tree_vertex_index_t v = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
                get_vertex(v)->point = pre_alloced_points[point_number];
                point_number++;
                tree_vertex_index_t z_min = nearest;
                double best_cost = get_vertex(z_min)->cost + validity_checker->trajectory_cost(trajectory);

                std::vector<tree_vertex_index_t> Z_near;
                Z_near = neighbors(trajectory[trajectory.size() - 1]);

                plan_t new_plan;
                new_plan.link_control_space(control_space);
                new_traj.clear();

                foreach(tree_vertex_index_t z_near, Z_near)
                {
                    new_plan.clear();
                    local_planner->steer(tree[z_near]->point, trajectory[trajectory.size() - 1], new_plan, new_traj);
                    if( get_vertex(z_near)->cost + validity_checker->trajectory_cost(new_traj) < best_cost &&
                        (!collision_checking || (validity_checker->is_valid(new_traj) && new_traj.size() > 1)) )
                    {
                        best_cost = get_vertex(z_near)->cost + validity_checker->trajectory_cost(new_traj);
                        plan = new_plan;
                        trajectory = new_traj;
                        z_min = z_near;
                    }
                }
                //now traj, control, and z_min should be correct

                state_space->copy_point(get_vertex(v)->point, trajectory[trajectory.size() - 1]);
                metric->add_point(tree[v]);
                get_vertex(v)->cost = best_cost;
                tree_edge_index_t e = tree.add_edge<rrt_edge_t > (z_min, v);
                get_edge(e)->plan = plan;
                state_space->copy_point(states_to_check[0], tree[v]->point);
                // if( visualize_tree )
                // {
                    get_edge(e)->trajectory = trajectory;
                // }

                foreach(tree_vertex_index_t z_near, Z_near)
                {
                    new_plan.clear();
                    local_planner->steer(tree[v]->point, tree[z_near]->point, new_plan, new_traj);
                    if( get_vertex(v)->cost + validity_checker->trajectory_cost(new_traj) < get_vertex(z_near)->cost &&
                        (!collision_checking || (validity_checker->is_valid(new_traj) && new_traj.size() > 1)) )
                    {
                        //remove old parent edge of z_near
                        tree_edge_index_t edge = tree.edge(get_vertex(z_near)->get_parent(), z_near);
                        get_edge(edge)->trajectory.clear();
                        tree.transplant(z_near, v);
                        edge = tree.edge(get_vertex(z_near)->get_parent(), z_near);
                        get_edge(edge)->plan = new_plan;
                        get_vertex(z_near)->cost = get_vertex(v)->cost + validity_checker->trajectory_cost(new_traj);
                        // if( visualize_tree )
                        // {
                            get_edge(edge)->trajectory = new_traj;
                        // }
                        update_costs(z_near);

                    }
                }

                if( input_query->get_goal()->satisfied(states_to_check[0]) )
                {
                    best_solution_cost = best_cost;
                    possible_goals.push_back(v);
                }

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

        void informed_rrt_star_t::update_costs(tree_vertex_index_t v)
        {

            foreach(tree_vertex_index_t child, tree[v]->get_children())
            {
                rrt_edge_t* edge = get_edge(tree.edge(v, child));
                get_vertex(child)->cost = get_vertex(v)->cost + validity_checker->trajectory_cost(edge->trajectory);
                update_costs(child);
            }
        }

        void informed_rrt_star_t::update_k()
        {
            unsigned dim = state_space->get_dimension();
            double constant = pow(2 * (1 + 1.0 / dim), 1.0 / dim) * pow(state_space->space_size() / state_space->n_ball_measure(1.0), 1.0 / dim);

            k = constant * pow((log10(tree.num_vertices()) / tree.num_vertices()), 1.0 / dim);
        }

        std::vector<tree_vertex_index_t> informed_rrt_star_t::neighbors(state_t* state)
        {
            update_k();

            std::vector<const abstract_node_t*> nodes = metric->radius_query(state, k);
            std::vector<tree_vertex_index_t> vertices;

            foreach(const abstract_node_t* node, nodes)
            {
                vertices.push_back(node->as<rrt_node_t > ()->get_index());
            }

            return vertices;
        }

        const statistics_t* informed_rrt_star_t::get_statistics()
        {
            time_elapsed+=clock.measure();
            statistics = new rrt_star_statistics_t();
            statistics->as<rrt_statistics_t > ()->num_vertices = tree.num_vertices();
            statistics->as<rrt_statistics_t > ()->solution_quality = validity_checker->trajectory_cost(input_query->path);
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
            double avg_cost = 0;
            int count = 0;
            foreach(tree_node_t* v, tree.vertices())
            {
                avg_cost += get_vertex(v->get_index())->cost;
                count++;
            }
            avg_cost /= count;
            
            // {
            //     std::stringstream s;
            //     s<<std::setfill('0') << std::setw(8)<<iteration_count<<".svg";
            //     img_count++;
            //     dir = dir+s.str();
            //     Dimensions dimensions(1000, 1000);
            //     Document doc(dir, Layout(dimensions, Layout::BottomLeft));

            //     foreach(tree_edge_t* e, tree.edges())
            //     {
            //         tree_node_t* v = tree[e->get_source()];
            //         tree_node_t* v2 = tree[e->get_target()];
            //         // if (!v->bridge && !v2->bridge)
            //         {
            //             double x,y;
            //             state_t* state = v->point;
            //             state_t* state2 = v2->point;
            //             doc << (Polyline(Stroke(.5,Color::Blue)) << Point(state->at(0)*50+500,state->at(1)*50+500)
            //                                                      << Point(state2->at(0)*50+500,state2->at(1)*50+500));

            //         }
            //     }

            //     doc << Rectangle(Point(3*50+500-100, 4*50+500+275), 4*50, 11*50, Color::Red);
            //     doc << Rectangle(Point(-4.5*50+500-175,5*50+500+37.5), 7*50, 1.5*50, Color::Red);
            //     doc << Rectangle(Point(7*50+500-100,4*50+500+25), 4*50, 1*50, Color::Red);
            //     doc << Rectangle(Point(-6*50+500-125,-5*50+500+125), 5*50, 5*50, Color::Red);
            //     doc << Rectangle(Point(7*50+500-100,-6.5*50+500+100), 4*50, 4*50, Color::Red);

            //     Polyline solution(Stroke(3, Color::Red));
            //     foreach(state_t* state, input_query->path)
            //     {
            //         solution<<Point(state->at(0)*50+500,state->at(1)*50+500);
            //     }
            //     doc<<solution;
            //     doc.save();
            // }

            ((rrt_star_statistics_t*)(statistics))->average_cost = avg_cost;
            PRX_INFO_S("Stats: "<<iteration_count<<" "<<tree.num_vertices()<<" "<<validity_checker->trajectory_cost(input_query->path) );
            clock.reset();
            return statistics;
        }

    }
}
