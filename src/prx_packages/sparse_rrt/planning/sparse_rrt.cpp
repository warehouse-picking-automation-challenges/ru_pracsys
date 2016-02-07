/**
 * @file sparse_rrt.cpp
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

#include "sparse_rrt.hpp"
#include "sparse_rrt_statistics.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/statistics/image.hpp"
#include "prx/utilities/statistics/svg_image.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/utilities/spaces/embedded_space.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::sparse_rrt::sparse_rrt_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    using namespace svg;
    namespace packages
    {
        namespace sparse_rrt
        {
        
            sparse_rrt_t::sparse_rrt_t() 
            {
                statistics = new sparse_rrt_statistics_t();
                drain = true;
                radius_nearest = true;
                count_of_failure = 0;
                img_count=0;
                time_elapsed = 0;
            }

            sparse_rrt_t::~sparse_rrt_t() 
            { 
            }

            void sparse_rrt_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
            {
                rrt_t::init( reader, template_reader);
                delta_drain = parameters::get_attribute_as<double>("delta_drain",reader,template_reader,0);
                delta_near = parameters::get_attribute_as<double>("delta_near",reader,template_reader,4);
                max_attempts = parameters::get_attribute_as<int>("max_attempts",reader,template_reader,100);
                drain = (delta_drain>PRX_ZERO_CHECK);
                radius_nearest = (delta_near>PRX_ZERO_CHECK);
                thresh = parameters::get_attribute_as<double>("cost_threshold",reader,template_reader);
                steering = parameters::get_attribute_as<bool>("steering",reader,template_reader);
                alpha = parameters::get_attribute_as<double>("alpha",reader,template_reader,1);
                sprint_iterations = initial_sprint_iterations = parameters::get_attribute_as<int>("sprint_iterations",reader,template_reader,0);
                // if(sprint_iterations==0)
                //     sprint_iterations = std::numeric_limits<int>::max();

                PRX_WARN_S("Drain: "<<drain<<" "<<delta_drain<<" BestNearest: "<<radius_nearest<<" "<<delta_near);
                PRX_WARN_S("Alpha: "<<alpha<<" Sprint_iterations: "<<sprint_iterations);

            }

            void sparse_rrt_t::setup()
            {
                point_number=0;
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                std::stringstream s1;
                s1<<"/prx_output/"<<this->path<<"/";
                dir += (s1.str());
                boost::filesystem::path output_dir (dir);
                if (!boost::filesystem::exists(output_dir))
                {
                    boost::filesystem::create_directories( output_dir );
                }
                output_directory = dir;
                tree.pre_alloc_memory<sparse_rrt_node_t,sparse_rrt_edge_t>(max_points);
                trajectory.link_space(state_space);
                states_to_check.clear();
                states_to_check.push_back(state_space->clone_point(input_query->get_start_state()));
                sample_point = state_space->alloc_point();
                start_vertex = tree.add_vertex<sparse_rrt_node_t,sparse_rrt_edge_t>();
                get_vertex(start_vertex)->time_stamp = iteration_count;
                tree[start_vertex]->point = state_space->clone_point(input_query->get_start_state());
                PRX_DEBUG_S ("Start state: " << state_space->print_point(tree[start_vertex]->point,3));
                metric->add_point(tree[start_vertex]);
                clock.reset();   
                for(unsigned i=0;i<max_points;i++)
                {
                    pre_alloced_points.push_back(state_space->alloc_point());
                }
                for(unsigned i=0;i<max_points;i++)
                {
                    radial.push_back(new abstract_node_t());
                }
                best_goal = start_vertex;
                real_solution = false;
                sprint_iteration_counter = 0;
                sprint_count = 0;
                last_stat = 0;

            }

            void sparse_rrt_t::resolve_query()
            {
                solution_number++;
                input_query->clear();
                tree_vertex_index_t new_v = best_goal;
                std::deque<tree_vertex_index_t> path_v;
                while(get_vertex(new_v)->get_parent() != new_v)
                {                
                    path_v.push_front(new_v);
                    tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(),new_v);
                    rrt_edge_t* edge = get_edge(e);

                    foreach(plan_step_t step, edge->plan)
                    {
                        input_query->plan.copy_onto_front(step.control,step.duration);
                    }
                    new_v = get_vertex(new_v)->get_parent();
                }
                // if(path_v.size()!=0)
                // {
                //     for(unsigned i=0;i<path_v.size()-1;i++)
                //     {
                //         tree_edge_index_t e = tree.edge(path_v[i],path_v[i+1]);
                //         rrt_edge_t* edge = get_edge(e);
                //         input_query->path+=edge->trajectory;
                //     }
                // }
                // else
                    local_planner->propagate(tree[start_vertex]->point,input_query->plan,input_query->path);


                // trajectory_t traj2;
                // traj2.link_space(state_space);

                // if(best_goal!=start_vertex)
                // {
                //     for(int i=0;i<100;i++)
                //     {
                //         traj2.clear();
                //         local_planner->propagate(tree[start_vertex]->point,input_query->plan,traj2);
                //         PRX_ERROR_S(state_space->print_point(traj2.back()));
                //     }
                // }
                // PRX_ERROR_S(((embedded_space_t*)state_space)->get_preimage_space()->print_point(((embedded_point_t*)tree[start_vertex]->point)->link,30));

                if(!state_space->equal_points(tree[best_goal]->point,input_query->path.back() ) )
                {
                    PRX_ERROR_S(state_space->print_point(tree[best_goal]->point));
                    PRX_ERROR_S(((embedded_space_t*)state_space)->get_preimage_space()->print_point(((embedded_point_t*)tree[best_goal]->point)->link));
                    PRX_ERROR_S(state_space->print_point(input_query->path.back()));
                    PRX_ERROR_S(((embedded_space_t*)state_space)->get_preimage_space()->print_point(((embedded_point_t*)input_query->path.back())->link));
                }
            }

            void sparse_rrt_t::step()
            {                   
                // bool send_new_geoms=false;
                //sample a state
                sampler->sample(state_space,sample_point);  

                //get the nearest state    
                tree_vertex_index_t nearest = nearest_vertex(sample_point);


                // PRX_ERROR_S(state_space->print_point(tree[nearest]->point));
                // PRX_ERROR_S(((embedded_space_t*)state_space)->get_preimage_space()->print_point(((embedded_point_t*)tree[nearest]->point)->link));

                //propagate toward the sampled point
                plan_t plan;
                plan.link_control_space(control_space);
                state_t* end_state = pre_alloced_points[point_number];
                double prop_length=0;
                int attempts=0;
                trajectory.clear();
                do
                {
                    if(collision_checking)
                    {
                        plan.clear();
                        local_planner->steer(tree[nearest]->point,sample_point,plan,trajectory,steering);
                        state_space->copy_point(end_state,trajectory[trajectory.size()-1]);
                        prop_length = plan.length();         
                    }
                    else
                    {
                        plan.clear();            
                        local_planner->steer(tree[nearest]->point,sample_point,plan,end_state,steering);            
                        prop_length = plan.length();          
                    }
                    attempts++;
                }
                while(drain && attempts < max_attempts && ( metric->distance_function(tree[nearest]->point,end_state) < delta_drain || metric->distance_function(tree[nearest]->point,end_state) == PRX_INFINITY));
                count_of_failure += attempts-1;  

                // PRX_DEBUG_COLOR(state_space->print_point(end_state),PRX_TEXT_CYAN);
                // PRX_DEBUG_COLOR(*state_space,PRX_TEXT_CYAN);

                //check if the trajectory is valid
                if(!collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size()>1))
                {
                    std::vector<tree_vertex_index_t> X_near;
                    double new_cost = get_vertex(nearest)->cost + prop_length;
                    bool better_node = false;
                    //Check if a node inside the radius_nearest region
                    if(drain)
                    {
                        X_near = neighbors(end_state);
                        foreach(tree_vertex_index_t x_near, X_near)
                        {
                            if(get_vertex(x_near)->cost <= new_cost )
                            {
                                better_node = true;
                            }
                        }
                    }
                    if(!better_node)
                    {      
                        tree_vertex_index_t v = tree.add_vertex<sparse_rrt_node_t,sparse_rrt_edge_t>();
                        sparse_rrt_node_t* node = get_vertex(v);
                        node->point = end_state;
                        node->time_stamp = iteration_count;

                        point_number++;
                        node->bridge = true;
                        node->cost = get_vertex(nearest)->cost + prop_length;   
                        tree_edge_index_t e = tree.add_edge<sparse_rrt_edge_t>(nearest,v);
                        get_edge(e)->plan = plan;            
                        state_space->copy_point(states_to_check[0],end_state);
                        //check if better goal node
                        if(!real_solution && input_query->get_goal()->satisfied(end_state))
                        {
                            best_goal = v;
                            real_solution = true;
                        }
                        else if(real_solution && get_vertex(best_goal)->cost > get_vertex(v)->cost && input_query->get_goal()->satisfied(end_state) )
                        {
                            best_goal = v;
                        }

                        //check if we need to store trajectories for visualization
                        if(collision_checking && visualize_tree)
                        {
                            get_edge(e)->trajectory = trajectory;
                        }
                        else if(!collision_checking && visualize_tree)
                        {
                            if(trajectory.size()==0)
                                local_planner->propagate(get_vertex(nearest)->point,plan,trajectory);
                            get_edge(e)->trajectory = trajectory;
                        }   

                        if(drain)
                        {
                            // PRX_INFO_S("pruning");
                            foreach(tree_vertex_index_t x_near, X_near)
                            {
                                // remove_subtree(x_near);
                                if(!get_vertex(x_near)->bridge )
                                {
                                    metric->remove_point(tree[x_near]);
                                    get_vertex(x_near)->bridge = true;
                                }
                                tree_vertex_index_t iter = x_near;
                                while( is_leaf(iter) && get_vertex(iter)->bridge && !is_best_goal(iter))
                                {
                                    tree_vertex_index_t next = get_vertex(iter)->get_parent();
                                    remove_leaf(iter);
                                    iter = next;
                                }             
                            }
                        }
                        get_vertex(v)->bridge = false;
                        metric->add_point(tree[v]);
                    }
                }  
                iteration_count++;
                if(point_number == max_points)
                {
                    for(unsigned i=0;i<max_points;i++)
                    {
                        pre_alloced_points.push_back(state_space->alloc_point());
                        radial.push_back(new abstract_node_t());
                    }
                    max_points*=2;
                }
                sprint_iteration_counter++;
                alpha = .95;
                if(sprint_iteration_counter==sprint_iterations)
                {
                    sprint_count++;
                    sprint_iterations = (1+log(sprint_count))*1.0/pow(alpha,(state_space->get_dimension()+control_space->get_dimension()+1)*(sprint_count-1))*initial_sprint_iterations;
                    PRX_INFO_S(sprint_iterations);
                    sprint_iteration_counter=0;
                    delta_near*=alpha;
                    delta_drain*=alpha;
                }
            }

            void sparse_rrt_t::remove_subtree(util::tree_vertex_index_t v)
            {
                bool new_goal=(v==best_goal);
                if(get_vertex(v)==NULL)
                    return;
                if(get_vertex(v)->get_children().size()!=0)
                {
                    std::vector<tree_vertex_index_t> vs;
                    // PRX_INFO_S("NOT LEAF");
                    for(std::list<tree_vertex_index_t>::const_iterator i=get_vertex(v)->get_children().begin();
                        i!=get_vertex(v)->get_children().end();
                        i++)
                    {
                        vs.push_back(*i);
                        // PRX_ERROR_S("Parent: "<<v<<" Child: "<<*i);
                    }
                    foreach(tree_vertex_index_t child, vs)
                    {
                        remove_subtree(child);
                    }
                }
                // else
                // {
                //     PRX_INFO_S("LEAF "<<v);
                // }
                // PRX_INFO_S("--- deleting "<<v);
                remove_leaf(v);
                if(new_goal)
                {
                    real_solution = false;
                    best_goal = start_vertex;
                    foreach(tree_node_t* node, tree.vertices())
                    {
                        tree_vertex_index_t iter = node->get_index();
                        if(!real_solution && input_query->get_goal()->satisfied(get_vertex(iter)->point))
                        {
                            best_goal = iter;
                            real_solution = true;
                        }
                        else if(real_solution && get_vertex(best_goal)->cost > get_vertex(iter)->cost && input_query->get_goal()->satisfied(get_vertex(iter)->point) )
                        {
                            best_goal = iter;
                        }
                    }
                }
            }

            bool sparse_rrt_t::is_best_goal(tree_vertex_index_t v)
            {
                tree_vertex_index_t new_v = best_goal;

                while(get_vertex(new_v)->get_parent()!=new_v)
                {
                    if(new_v == v)
                        return true;

                    new_v = get_vertex(new_v)->get_parent();
                }
                return false;

            }


            bool sparse_rrt_t::is_leaf(tree_vertex_index_t v)
            {
                return (get_vertex(v)->get_children().size() == 0);
            }

            void sparse_rrt_t::remove_leaf(tree_vertex_index_t v)
            {
                if(get_vertex(v)->get_parent()!=v)
                {
                    rrt_edge_t* edge = get_edge(tree.edge(get_vertex(v)->get_parent(),v));  
                    edge->plan.clear();
                }
                if(!get_vertex(v)->bridge)
                {
                    metric->remove_point(tree[v]);
                }
                tree.remove_vertex(v);
            }    

            inline sparse_rrt_node_t* sparse_rrt_t::get_vertex(tree_vertex_index_t v) const
            {
                return tree.get_vertex_as<sparse_rrt_node_t>(v);
            }
            inline sparse_rrt_edge_t* sparse_rrt_t::get_edge(tree_edge_index_t e) const
            {
                return tree.get_edge_as<sparse_rrt_edge_t>(e);
            }

            tree_vertex_index_t sparse_rrt_t::nearest_vertex(state_t* state)
            {    
                if(radius_nearest)
                {
                    unsigned val = metric->radius_and_closest_query(state,delta_near, radial);
                    if(val==0)
                    {
                        return ((const tree_node_t*)metric->single_query(state))->get_index();
                    }
                    else
                    {
                        tree_vertex_index_t ret_val;
                        double length = PRX_INFINITY;
                        std::vector<const abstract_node_t*>::const_iterator iter,iter_end;
                        iter = radial.begin();
                        iter_end = iter;
                        std::advance(iter_end,val);
                        for(iter = radial.begin();iter!=iter_end;iter++)
                        {
                            tree_vertex_index_t v = ((const tree_node_t*)(*iter))->get_index();
                            double temp = get_vertex(v)->cost ;
                            if( temp < length)
                            {
                                length = temp;
                                ret_val = v;
                            }
                        }
                        return ret_val;
                    }
                }
                else
                {
                    return ((const tree_node_t*)metric->single_query(state))->get_index();
                }
            }
            std::vector<tree_vertex_index_t> sparse_rrt_t::neighbors(state_t* state)
            {
                unsigned num = metric->radius_query(state,delta_drain,radial);
                // PRX_INFO_S(num);
                std::vector<tree_vertex_index_t> vertices;
                std::vector<const abstract_node_t*>::const_iterator iter,iter_end;
                iter = radial.begin();
                iter_end = iter;
                std::advance(iter_end,num);
                for(iter = radial.begin();iter!=iter_end;iter++)
                {
                    vertices.push_back((*iter)->as<rrt_node_t>()->get_index());
                }

                return vertices;
            }
            void sparse_rrt_t::update_vis_info() const
            {
                rrt_t::update_vis_info();
                // std::string plant_name, subpath;
                // boost::tie(plant_name, subpath) = reverse_split_path(visualization_body);
                // subpath = "world_model/";
                // plant_name = subpath + plant_name;
                // std::vector<std::string> ghost_names;
                // std::vector<config_t> ghost_configs;

                // state_t* start = input_query->path[0];
                // config_t base = config_t(vector_t(start->at(0), start->at(1),start->at(2)),quaternion_t(0,0,0,1));
                // for(unsigned i=0;i<input_query->path.size();i+=150)
                // {
                //     state_t* state = input_query->path[i];
                //     ghost_names.push_back(plant_name);
                //     ghost_configs.push_back(config_t(vector_t(state->at(0), state->at(1),state->at(2)),quaternion_t(0,0,0,1)));
                //     ghost_configs.back() = ghost_configs.back() - base;
                // }
                // state_t* state = input_query->path.back();
                // ghost_names.push_back(plant_name);
                // ghost_configs.push_back(config_t(vector_t(state->at(0), state->at(1),state->at(2)),quaternion_t(0,0,0,1)));
                // ghost_configs.back() = ghost_configs.back() - base;


                // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->visualize_ghost_plant(ghost_names, ghost_configs);

            }
            const statistics_t* sparse_rrt_t::get_statistics()
            {      
                time_elapsed+=clock.measure();

                double avg_cost=0;
                int count=0;
                // {
                //     std::stringstream s;
                //     s<<std::setfill('0') << std::setw(8)<<iteration_count<<".svg";
                //     img_count++;
                //     std::string dir(output_directory+s.str());
                //     Dimensions dimensions(1000, 1000);
                //     Document doc(dir, Layout(dimensions, Layout::BottomLeft));

                //     if(visualize_tree)
                //     {
                //         foreach(tree_edge_t* e, tree.edges())
                //         {
                //             sparse_rrt_edge_t* edge = get_edge(e->get_index());
                //             Polyline traj_line(Stroke(.5, Color::Blue));
                //             foreach(state_t* state, edge->trajectory)
                //             {
                //                 traj_line << Point(state->at(0)*50+500,state->at(1)*50+500);

                //             }
                //             doc<<traj_line;
                //         }
                //     }
                //     else
                //     {
                //         foreach(tree_edge_t* e, tree.edges())
                //         {
                //             sparse_rrt_edge_t* edge = get_edge(e->get_index());
                //             Polyline traj_line(Stroke(.5, Color::Blue));
                //             edge->trajectory.link_space(state_space);
                //             local_planner->propagate(get_vertex(edge->get_source())->point,edge->plan,edge->trajectory);
                //             foreach(state_t* state, edge->trajectory)
                //             {
                //                 traj_line << Point(state->at(0)*50+500,state->at(1)*50+500);

                //             }
                //             doc<<traj_line;
                //         }
                //     }

                //     // For car
                //     // doc << Rectangle(Point(0,975), 1000,50, Color::Red);
                //     // doc << Rectangle(Point(0,-25), 1000,50, Color::Red);
                //     // doc << Rectangle(Point(0,-25), 50,1000, Color::Red);
                //     // doc << Rectangle(Point(950,-25), 50,1000, Color::Red);

                //     // doc << Rectangle(Point(700,275), 50,50, Color::Red);
                //     // doc << Rectangle(Point(550,275), 50,50, Color::Red);
                //     // doc << Rectangle(Point(400,275), 50,50, Color::Red);
                //     // doc << Rectangle(Point(250,275), 50,50, Color::Red);
                //     // doc << Rectangle(Point(700,675), 50,50, Color::Red);
                //     // doc << Rectangle(Point(550,675), 50,50, Color::Red);
                //     // doc << Rectangle(Point(400,675), 50,50, Color::Red);
                //     // doc << Rectangle(Point(250,675), 50,50, Color::Red);

                //     // doc << Rectangle(Point(50,475), 350,50, Color::Red);
                //     // doc << Rectangle(Point(600,475), 350,50, Color::Red);


                //     // for point system
                //     // doc << Rectangle(Point(-4.5*50+500-175,5*50+500+37.5), 7*50, 1.5*50, Color::Red);
                //     // doc << Rectangle(Point(7*50+500-100,4*50+500+25), 4*50, 1*50, Color::Red);
                //     // doc << Rectangle(Point(-6*50+500-125,-5*50+500+125), 5*50, 5*50, Color::Red);
                //     // doc << Rectangle(Point(7*50+500-100,-6.5*50+500+100), 4*50, 4*50, Color::Red);
                    
                //     doc << Rectangle(Point(3*50+500-100, 4*50+500+275), 4*50, 11*50, Color::Red);
                //     doc << Rectangle(Point(-4.5*50+500-175,5*50+500+37.5), 7*50, 1.5*50, Color::Red);
                //     doc << Rectangle(Point(7*50+500-100,4*50+500+25), 4*50, 1*50, Color::Red);
                //     doc << Rectangle(Point(-6*50+500-125,-5*50+500+125), 5*50, 5*50, Color::Red);
                //     doc << Rectangle(Point(7*50+500-100,-6.5*50+500+100), 4*50, 4*50, Color::Red);

                //     if(best_goal!=start_vertex)
                //     {
                //         Polyline solution(Stroke(3, Color::Red));
                //         foreach(state_t* state, input_query->path)
                //         {
                //             solution<<Point(state->at(0)*50+500,state->at(1)*50+500);
                //         }
                //         doc<<solution;
                //     }
                //     doc.save();
                // }
                double avg_time_stamp=0;
                double max_time=0;
                foreach(tree_node_t* v, tree.vertices())
                {
                    if (!v->as<sparse_rrt_node_t>()->bridge)
                    {
                        avg_cost += get_vertex(v->get_index())->cost;
                        count++;
                        if(v->get_index()!=start_vertex)
                        {
                            double temp = iteration_count - get_vertex(v->get_index())->time_stamp;
                            avg_time_stamp+= temp;
                            if(temp > max_time)
                                max_time = temp;
                        }
                    }
                }
                
                avg_cost/=count;
                avg_time_stamp/=count;

                statistics = new sparse_rrt_statistics_t();
                statistics->as<sparse_rrt_statistics_t>()->failure_count = count_of_failure;
                statistics->as<sparse_rrt_statistics_t>()->num_vertices = tree.num_vertices();
                statistics->as<sparse_rrt_statistics_t>()->solution_quality = input_query->plan.length();
                statistics->as<sparse_rrt_statistics_t>()->average_cost = avg_cost;
                statistics->as<sparse_rrt_statistics_t>()->best_near = delta_near;
                statistics->as<sparse_rrt_statistics_t>()->drain = delta_drain;
                statistics->as<sparse_rrt_statistics_t>()->avg_lifespan = avg_time_stamp;


                sparse_rrt_node_t* start_node = get_vertex(start_vertex);
                double avg=0;
                int child_count=0;
                foreach(tree_vertex_index_t v, get_vertex(start_vertex)->get_children())
                {
                    if(!get_vertex(start_vertex)->bridge)
                    {
                        avg+=metric->distance_function(start_node->point,get_vertex(v)->point);
                        child_count++;
                    }
                }
                avg/=child_count;
                statistics->as<sparse_rrt_statistics_t>()->children_of_root = child_count;
                statistics->as<sparse_rrt_statistics_t>()->avg_distance_from_root = avg;
                statistics->time = time_elapsed;
                statistics->steps = iteration_count;
                count=0;
                PRX_INFO_S("Stats: "<<iteration_count<<" "<<tree.num_vertices()<<" "<<input_query->plan.length()<<" "<<avg_cost<<" "<<delta_near<<" "<<delta_drain<<" "<<avg_time_stamp<<" "<<get_vertex(start_vertex)->get_children().size()<<" "<<avg);

                clock.reset();
                last_stat = iteration_count;
                return statistics;
            }
        }
    }
}
