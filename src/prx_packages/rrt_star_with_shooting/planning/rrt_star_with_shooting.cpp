/**
 * @file rrt_star_with_shooting.cpp
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

#include "rrt_star_with_shooting.hpp"
#include "rrt_star_with_shooting_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/samplers/sampler.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/utilities/statistics/image.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::rrt_star_with_shooting::rrt_star_with_shooting_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    namespace packages
    {
        namespace rrt_star_with_shooting
        {

            rrt_star_with_shooting_t::rrt_star_with_shooting_t() 
            {
            //    statistics = new rrt_star_with_shooting_statistics_t();
                goal_found = false;
                img_count = 0;
                Z_near.clear();
                to_delete.clear();
            }

            rrt_star_with_shooting_t::~rrt_star_with_shooting_t() 
            { 
            }

            void rrt_star_with_shooting_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
            {
                rrt_t::init( reader, template_reader);
                near_threshold = parameters::get_attribute_as<double>("near_threshold",reader,template_reader);
            }

            void rrt_star_with_shooting_t::setup()
            {
                rrt_t::setup();
                best_goal = start_vertex;
                new_traj.link_space(state_space);
                plan.link_control_space(control_space);
                new_plan.link_control_space(control_space);
                for(unsigned i=0;i<max_points;i++)
                {
                    radial.push_back(new abstract_node_t());
                }
                Z_near.clear();
            }

            void rrt_star_with_shooting_t::resolve_query()
            {
                solution_number++;
                input_query->clear();
                tree_vertex_index_t new_v = best_goal;
                while(get_vertex(new_v)->get_parent() != new_v)
                {                
                    tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(),new_v);
                    rrt_edge_t* edge = get_edge(e);

                    foreach(plan_step_t step, edge->plan)
                    {
                        input_query->plan.copy_onto_front(step.control,step.duration);
                    }
                    new_v = get_vertex(new_v)->get_parent();
                }
                local_planner->propagate(tree[start_vertex]->point,input_query->plan,input_query->path);
            }


            void rrt_star_with_shooting_t::step()
            {        
                to_delete.clear();
                //sample a state
                sampler->sample(state_space,sample_point);
                //get the nearest state
                tree_vertex_index_t nearest = nearest_vertex(sample_point);
                //propagate toward the sampled point

                plan.clear();
                new_plan.clear();
                trajectory.clear();
                new_traj.clear();
                local_planner->steer(tree[nearest]->point,sample_point,plan,trajectory,false);

                //check if the trajectory is valid
                if(!collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size()>1))
                {
                    tree_vertex_index_t v = tree.add_vertex<rrt_node_t,rrt_edge_t>();
                    get_vertex(v)->point = pre_alloced_points[point_number];
                    point_number++;

                    tree_vertex_index_t z_min = nearest;
                    double best_cost = get_vertex(z_min)->cost + plan.length();
                    Z_near.clear();
                    if(goal_found)
                    {
                        neighbors(trajectory[trajectory.size()-1]);
                    }

                    foreach(tree_vertex_index_t z_near, Z_near)
                    {
                        new_traj.clear();
                        new_plan.clear();
                        local_planner->steer(tree[z_near]->point,trajectory[trajectory.size()-1],new_plan,new_traj);
                        if(         get_vertex(z_near)->cost + new_plan.length() < best_cost &&
                                    metric->distance_function(new_traj[new_traj.size()-1],trajectory[trajectory.size()-1]) <= near_threshold &&
                                    (!collision_checking || (validity_checker->is_valid(new_traj) && new_traj.size()>1)))
                        {
                            best_cost = get_vertex(z_near)->cost + new_plan.length();
                            plan = new_plan;      
                            trajectory = new_traj;
                            z_min = z_near;
                        }
                    }
                    //now traj, control, and z_min should be correct

                    state_space->copy_point(get_vertex(v)->point,trajectory[trajectory.size()-1]);
                    metric->add_point(tree[v]);
                    get_vertex(v)->cost = best_cost;   
                    tree_edge_index_t e = tree.add_edge<rrt_edge_t>(z_min,v);
                    get_edge(e)->plan = plan;
                    state_space->copy_point(states_to_check[0],tree[v]->point);
                    if(best_goal == start_vertex && input_query->get_goal()->satisfied(tree[v]->point))
                    {
                        best_goal = v;
                    }
                    else if(get_vertex(best_goal)->cost > get_vertex(v)->cost && input_query->get_goal()->satisfied(tree[v]->point) )
                    {
                        best_goal = v;
                    }  
                    if(visualize_tree)
                    {
                        get_edge(e)->trajectory = trajectory;
                    }        

                    foreach(tree_vertex_index_t z_near, Z_near)
                    {
                        new_traj.clear();
                        new_plan.clear();
                        bool in_delete = std::find(to_delete.begin(),to_delete.end(),z_near)!=to_delete.end();
                        if(!in_delete)
                        {
                            if(get_vertex(v)->cost < get_vertex(z_near)->cost)
                            {
                                local_planner->steer(tree[v]->point,tree[z_near]->point,new_plan, new_traj);
                                if(         get_vertex(v)->cost + new_plan.length() < get_vertex(z_near)->cost &&
                                            metric->distance_function(new_traj[new_traj.size()-1],tree[z_near]->point) <= near_threshold &&
                                            (!collision_checking || (validity_checker->is_valid(new_traj) && new_traj.size()>1)))
                                {
                                    //remove old parent edge of z_near
                                    tree_edge_index_t edge = tree.edge(get_vertex(z_near)->get_parent(),z_near);
                                    get_edge(edge)->trajectory.clear();
                                    get_edge(edge)->plan.clear();
                                    tree.transplant(z_near,v);
                                    edge = tree.edge(get_vertex(z_near)->get_parent(),z_near);
                                    get_edge(edge)->plan = new_plan;
                                    get_vertex(z_near)->cost = get_vertex(v)->cost+new_plan.length();

                                    //need to treat the node's point and metric separately
                                    metric->remove_point(tree[z_near]);
                                    state_space->copy_point(tree[z_near]->point,new_traj[new_traj.size()-1]);
                                    metric->add_point(tree[z_near]);

                                    if(visualize_tree)
                                    {
                                        get_edge(edge)->trajectory = new_traj;
                                    }

            //                        PRX_INFO_S("Repropagating");
                                    //now that this edge has been created, we need to re-simulate the subtree
                                    repropagate(z_near);
                                }
                            }
                        }
                    }        
                }

                if(!goal_found && input_query->get_goal()->satisfied(states_to_check[0]))
                {
                    PRX_INFO_S("We found the goal! Iteration: "<<iteration_count);
                    goal_found = true;
                }
                iteration_count++;

            }

            void rrt_star_with_shooting_t::repropagate(tree_vertex_index_t v)
            {
            //    PRX_INFO_S("R: "<<v);
                std::vector<tree_vertex_index_t> to_be_deleted;
                foreach(tree_vertex_index_t child, tree[v]->get_children())
                {
                    new_traj.clear();
                    rrt_edge_t* edge = get_edge(tree.edge(v,child));     
                    get_vertex(child)->cost = get_vertex(v)->cost+edge->plan.length();
                    local_planner->propagate(tree[v]->point,edge->plan,new_traj);
                    if(!collision_checking || (validity_checker->is_valid(new_traj) && new_traj.size()>1))
                    {
                        bool flag=false;
                        if(best_goal == child)
                        {
                            best_goal = start_vertex;
                            flag =true;
                        }
                        metric->remove_point(tree[child]);
                        state_space->copy_point(tree[child]->point,new_traj[new_traj.size()-1]);
                        metric->add_point(tree[child]);
                        if(best_goal == start_vertex && input_query->get_goal()->satisfied(tree[child]->point))
                        {
                            best_goal = child;
                        }
                        else if(flag)
                        {
                            goal_found = false;
                            PRX_INFO_S("Lost goal!");
                        }

                        if(visualize_tree)
                        {
                            edge->trajectory = new_traj;
                        }            
                        repropagate(child);
                    }
                    else
                    {
                        to_be_deleted.push_back(child);
                    }
                } 
                foreach(tree_vertex_index_t child, to_be_deleted)
                {
                    rrt_edge_t* edge = get_edge(tree.edge(v,child)); 
                    edge->trajectory.clear();
                    edge->plan.clear();
                    delete_children_and_self(child);
                }
            }

            void rrt_star_with_shooting_t::delete_children_and_self(tree_vertex_index_t v)
            {
                std::vector<tree_vertex_index_t> vertices;
                foreach(tree_vertex_index_t child, tree[v]->get_children())
                {
                    vertices.push_back(child);
                }
                foreach(tree_vertex_index_t child, vertices)
                {
                    rrt_edge_t* edge = tree.get_edge_as<rrt_edge_t>(v,child);  
                    edge->trajectory.clear();
                    edge->plan.clear();
                    delete_children_and_self(child);
                }
                metric->remove_point(tree[v]);
                if(best_goal == v)
                {
                    PRX_INFO_S("Lost goal!");
                    goal_found = false;
                    best_goal = start_vertex;
                }
                to_delete.push_back(v);
                if(tree.is_leaf(v))
                {
                    tree.remove_vertex(v);
                }
            }

            void rrt_star_with_shooting_t::update_k()
            {    
                unsigned dim = state_space->get_dimension();
                double constant = pow(2*(1+1.0/dim),1.0/dim) * pow( state_space->space_size()/state_space->n_ball_measure(1.0) ,1.0/dim);

                k = constant*pow((log10( tree.num_vertices() )/tree.num_vertices()),1.0/dim);

            //    k = (log10( tree.num_vertices() )*2.7182818284*(1 + (1.0/(double)state_space->get_dimension())))+0.5000000;
            }

            void rrt_star_with_shooting_t::neighbors(state_t* state)
            {
                update_k();
                unsigned num = metric->radius_query(state,k,radial);

                unsigned other = (log10( tree.num_vertices() )*2.7182818284*(1 + (1.0/(double)state_space->get_dimension())))+0.5000000;
                if (num > other)
                    num = other;

                std::vector<const abstract_node_t*>::const_iterator iter,iter_end;
                iter = radial.begin();
                iter_end = iter;
                std::advance(iter_end,num);
                for(iter = radial.begin();iter!=iter_end;iter++)
                {
                    Z_near.push_back(((const tree_node_t*)(*iter))->get_index());
                }

            }

            const statistics_t* rrt_star_with_shooting_t::get_statistics()
            {    

                statistics_t* statistics = new rrt_star_with_shooting_statistics_t();
                statistics->as<rrt_statistics_t>()->num_vertices = tree.num_vertices();
                statistics->as<rrt_statistics_t>()->solution_quality = input_query->plan.length();
                statistics->time = clock.measure();
                statistics->steps = iteration_count;
            //    char* w = std::getenv("PRACSYS_PATH");
            //    std::string dir(w);
            //    std::stringstream s1;
            //    s1<<"/prx_output/"<<this->path<<"/";
            //    dir += (s1.str());
            //    boost::filesystem::path output_dir (dir);
            //    if (!boost::filesystem::exists(output_dir))
            //    {
            //        boost::filesystem::create_directories( output_dir );
            //    }
            //    double thresh = 20;    
            //    image_t image(400,400);
                double avg_cost=0;
                int count=0;
            //    foreach(tree_node_t* v, tree.vertices())
            //    {
            ////        directed_vertex_index_t v = start_vertex;
            //        double x,y;
            //        state_t* state = v->point;
            //        x = ((state->at(0)) / state_space->get_bounds()[0]->get_upper_bound())*200;
            //        y = -1*(state->at(1) / state_space->get_bounds()[1]->get_upper_bound())*200;
            //        unsigned char color;
            //        avg_cost += get_vertex(v->get_index())->cost;
            //        count++;
            //        if(get_vertex(v->get_index())->cost <= thresh)
            //            color = 254*(get_vertex(v->get_index())->cost/thresh) + 1;
            //        else
            //            color = 255;
            //        for(double i=x-2;i<=x+2;i++)
            //        {
            //            for(double j=y-2;j<=y+2;j++)
            //            {
            //                int i_pixel = floor(i+.5);
            //                int j_pixel = floor(j+.5);
            //                image.color_pixel(i_pixel,j_pixel,color,color,color);
            //            }            
            //        }
            //    }
            //
            //    
            //    foreach(state_t* state, input_query->path)
            //    {
            //        double x,y;
            //        x = ((state->at(0)) / state_space->get_bounds()[0]->get_upper_bound())*200;
            //        y = -1*(state->at(1) / state_space->get_bounds()[1]->get_upper_bound())*200;
            //        unsigned char color;
            //        for(double i=x-1;i<=x+1;i++)
            //        {
            //            for(double j=y-1;j<=y+1;j++)
            //            {
            //                int i_pixel = floor(i+.5);
            //                int j_pixel = floor(j+.5);
            //                image.color_pixel(i_pixel,j_pixel,0,255,0);
            //            }            
            //        }
            //    }


                foreach(tree_node_t* v, tree.vertices())
                {
                    avg_cost += get_vertex(v->get_index())->cost;
                    count++;
                }

            //    
                avg_cost/=count;
            //    std::stringstream s;
            //    s<<img_count<<".png";
            //    img_count++;
            //    dir += s.str();
            //    image.encode(dir.c_str());

                statistics->as<rrt_star_with_shooting_statistics_t>()->average_cost = avg_cost;

                return statistics;
            }
        }
    }
}
