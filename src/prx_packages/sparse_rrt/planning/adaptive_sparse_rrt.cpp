/**
 * @file adaptive_sparse_rrt.cpp
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

#include "adaptive_sparse_rrt.hpp"
#include "adaptive_sparse_rrt_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::sparse_rrt::adaptive_sparse_rrt_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    namespace packages
    {
        namespace sparse_rrt
        {
        
            adaptive_sparse_rrt_t::adaptive_sparse_rrt_t() 
            {
                statistics = new adaptive_sparse_rrt_statistics_t();
                drain = true;
                radius_nearest = true;
                count_of_failure = 0;
                img_count=0;
            }

            adaptive_sparse_rrt_t::~adaptive_sparse_rrt_t() 
            { 
            }

            void adaptive_sparse_rrt_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
            {
                rrt_t::init( reader, template_reader);
                delta_drain = parameters::get_attribute_as<double>("delta_drain",reader,template_reader,0);
                delta_near = parameters::get_attribute_as<double>("delta_near",reader,template_reader,4);
                max_attempts = parameters::get_attribute_as<int>("max_attempts",reader,template_reader,100);
                drain = (delta_drain>PRX_ZERO_CHECK);
                radius_nearest = (delta_near>PRX_ZERO_CHECK);
                thresh = parameters::get_attribute_as<double>("cost_threshold",reader,template_reader);
                
                vis_drain = parameters::get_attribute_as<bool>("vis_drain",reader,template_reader,true);
                vis_near = parameters::get_attribute_as<bool>("vis_near",reader,template_reader,true);
                vis_cost = parameters::get_attribute_as<bool>("vis_cost",reader,template_reader,true);

                PRX_WARN_S("Drain: "<<drain<<" "<<delta_drain<<" BestNearest: "<<radius_nearest<<" "<<delta_near);

            }

            void adaptive_sparse_rrt_t::setup()
            {
                point_count=0;
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
                tree.pre_alloc_memory<adaptive_sparse_rrt_node_t,adaptive_sparse_rrt_edge_t>(max_points);
                trajectory.link_space(state_space);
                states_to_check.clear();
                states_to_check.push_back(state_space->clone_point(input_query->get_start_state()));
                sample_point = state_space->alloc_point();
                start_vertex = tree.add_vertex<adaptive_sparse_rrt_node_t,adaptive_sparse_rrt_edge_t>();
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
                best_cost = PRX_INFINITY;
                real_solution = false;

            }

            void adaptive_sparse_rrt_t::resolve_query()
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

                if(!state_space->equal_points(tree[best_goal]->point,input_query->path.back() ) )
                {
                    PRX_ERROR_S(state_space->print_point(tree[best_goal]->point));
                    PRX_ERROR_S(state_space->print_point(input_query->path.back()));
                }
            }

            void adaptive_sparse_rrt_t::step()
            {   
                //sample a state
                sampler->sample(state_space,sample_point);  

                //get the nearest state    
                tree_vertex_index_t nearest = nearest_vertex(sample_point);

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
                        local_planner->steer(tree[nearest]->point,sample_point,plan,trajectory,false);
                        state_space->copy_point(end_state,trajectory[trajectory.size()-1]);
                        prop_length = plan.length();         
                    }
                    else
                    {
                        plan.clear();            
                        local_planner->steer(tree[nearest]->point,sample_point,plan,end_state,false);            
                        prop_length = plan.length();          
                    }
                    attempts++;
                }
                while(drain && attempts < max_attempts && ( metric->distance_function(tree[nearest]->point,end_state) < delta_drain || metric->distance_function(tree[nearest]->point,end_state) == PRX_INFINITY));
                count_of_failure += attempts-1;  

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
                        tree_vertex_index_t v = tree.add_vertex<adaptive_sparse_rrt_node_t,adaptive_sparse_rrt_edge_t>();
                        adaptive_sparse_rrt_node_t* node = get_vertex(v);
                        node->point = end_state;

            //            PRX_INFO_S("POINT: "<< state_space->print_point(end_state));
                        point_number++;
                        node->bridge = true;
                        node->cost = get_vertex(nearest)->cost + prop_length;   
                        tree_edge_index_t e = tree.add_edge<adaptive_sparse_rrt_edge_t>(nearest,v);
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
                            foreach(tree_vertex_index_t x_near, X_near)
                            {
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
            }
            
            double adaptive_sparse_rrt_t::calculate_cost(state_t* start, state_t* goal, double default_cost)
            {

                double H = goal->at(0) - start->at(0);
                return state_space->distance(start, goal) + (H<0?0:H);

            }

            void adaptive_sparse_rrt_t::branch_and_bound()
            {
                std::vector<tree_vertex_index_t> X;
                
                foreach(tree_node_t* v, tree.vertices())
                {
                    if(v->as<adaptive_sparse_rrt_node_t>()->cost > best_cost)
                        X.push_back(v->get_index());
                }
                
                foreach(tree_vertex_index_t x_near, X)
                {
                    if(!get_vertex(x_near)->bridge)
                    {
                        metric->remove_point(tree[x_near]);
                        get_vertex(x_near)->bridge = true;
                    }
                    tree_vertex_index_t iter = x_near;
                    while( is_leaf(iter) && get_vertex(iter)->bridge)
                    {
                        tree_vertex_index_t next = get_vertex(iter)->get_parent();
                        remove_leaf(iter);
                        iter = next;
                    }             
                }
            }
            

            bool adaptive_sparse_rrt_t::is_best_goal(tree_vertex_index_t v)
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


            bool adaptive_sparse_rrt_t::is_leaf(tree_vertex_index_t v)
            {
                return (get_vertex(v)->get_children().size() == 0);
            }

            void adaptive_sparse_rrt_t::remove_leaf(tree_vertex_index_t v)
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

            inline adaptive_sparse_rrt_node_t* adaptive_sparse_rrt_t::get_vertex(tree_vertex_index_t v) const
            {
                return tree.get_vertex_as<adaptive_sparse_rrt_node_t>(v);
            }
            inline adaptive_sparse_rrt_edge_t* adaptive_sparse_rrt_t::get_edge(tree_edge_index_t e) const
            {
                return tree.get_edge_as<adaptive_sparse_rrt_edge_t>(e);
            }

            
            void adaptive_sparse_rrt_t::sort_neighbors(unsigned val)
            {
                unsigned pos = 1;
                unsigned last = 0;
                while(pos < val)
                {
                    if(get_vertex(((const tree_node_t*)(radial[pos]))->get_index())->cost >= get_vertex(((const tree_node_t*)(radial[pos-1]))->get_index())->cost)
                    {
                        if(last != 0)
                        {
                            pos = last;
                            last = 0;
                        }
                        pos+=1;
                    }
                    else
                    {
                        const abstract_node_t* temp = radial[pos];
                        radial[pos] = radial[pos-1];
                        radial[pos-1] = temp;
                        if(pos>1)
                        {
                            if(last==0)
                                last = pos;
                            pos-=1;
                        }
                        else
                            pos+=1;
                    }
                }
            }
            
            tree_vertex_index_t adaptive_sparse_rrt_t::nearest_vertex(state_t* state)
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
            std::vector<tree_vertex_index_t> adaptive_sparse_rrt_t::neighbors(state_t* state)
            {
                unsigned num = metric->radius_query(state,delta_drain,radial);

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

            const statistics_t* adaptive_sparse_rrt_t::get_statistics()
            {      
                images.push_back(new image_t(400,400));
                double avg_cost=0;
                int count=0;
                if(visualize)
                {
                    images.back()->clear();
                    near_images.back()->clear();
                    drain_images.back()->clear();
                    foreach(tree_node_t* v, tree.vertices())
                    {
                        if (!v->as<adaptive_sparse_rrt_node_t>()->bridge)
                        {
                            double x,y;
                            state_t* state = v->point;
                            x = ((state->at(0)) / state_space->get_bounds()[0]->get_upper_bound())*200;
                            y = -1* (state->at(1) / state_space->get_bounds()[1]->get_upper_bound())*200;
                            unsigned char color;
                            avg_cost += get_vertex(v->get_index())->cost;
                            count++;
                            if(get_vertex(v->get_index())->cost <= thresh)
                                color = 254*(get_vertex(v->get_index())->cost/thresh) + 1;
                            else
                                color = 255;
                            for(double i=x-2;i<=x+2;i++)
                            {
                                for(double j=y-2;j<=y+2;j++)
                                {
                                    int i_pixel = floor(i+.5);
                                    int j_pixel = floor(j+.5);
                                    images.back()->color_pixel(i_pixel,j_pixel,color,color,color);
                                }            
                            }
                        }
                    }


                    foreach(state_t* state, input_query->path)
                    {
                        double x,y;
                        x = ((state->at(0)) / state_space->get_bounds()[0]->get_upper_bound())*200;
                        y =  -1*(state->at(1) / state_space->get_bounds()[1]->get_upper_bound())*200;
                        
                        for(double i=x-1;i<=x+1;i++)
                        {
                            for(double j=y-1;j<=y+1;j++)
                            {
                                int i_pixel = floor(i+.5);
                                int j_pixel = floor(j+.5);
                                images.back()->color_pixel(i_pixel,j_pixel,0,255,0);
                            }            
                        }
                    }
                    std::stringstream s;
                    std::stringstream s2;
                    std::stringstream s3;
                    if(img_count<10)
                    {
                        s<<"cost_00"<<img_count<<".ppm";
                    }
                    else if(img_count<100)
                    {
                        s<<"cost_0"<<img_count<<".ppm";
                    }
                    else
                    {
                        s<<"cost_"<<img_count<<".ppm";                      
                    }
                    std::string dir(output_directory+s.str());
                    images.back()->encode(dir.c_str());
                    img_count++;
                }
                // else
                // {

                //     std::ofstream fout;
                //     fout.open("/Users/zlittlefield/Desktop/cost_ratio.txt");
                //     foreach(tree_node_t* v, tree.vertices())
                //     {
                //         if (!v->as<adaptive_sparse_rrt_node_t>()->bridge)
                //         {
                //             fout<<get_vertex(v->get_index())->cost
                //                 <<" "<<calculate_cost(get_vertex(start_vertex)->point,get_vertex(v->get_index())->point,metric->distance_function(get_vertex(v->get_index())->point,get_vertex(start_vertex)->point)/175.0)
                //                 <<" "<<get_depth(v->get_index())
                //                 <<" "<<state_space->print_point(get_vertex(v->get_index())->point)
                //                 <<"\n";
                //             avg_cost += get_vertex(v->get_index())->cost;
                //             count++;
                //         }
                //     }
                //     fout.close();
                // }

                avg_cost/=count;

                statistics = new adaptive_sparse_rrt_statistics_t();
                statistics->as<adaptive_sparse_rrt_statistics_t>()->failure_count = count_of_failure;
                statistics->as<adaptive_sparse_rrt_statistics_t>()->num_vertices = tree.num_vertices();
                statistics->as<adaptive_sparse_rrt_statistics_t>()->solution_quality = get_vertex(best_goal)->cost;//input_query->plan.length();
                statistics->as<adaptive_sparse_rrt_statistics_t>()->average_cost = avg_cost;
                statistics->time = clock.measure();
                statistics->steps = iteration_count;
                count=0;
                PRX_INFO_S("Stats: "<<iteration_count<<" "<<tree.num_vertices()<<" "<<get_vertex(best_goal)->cost<<" "<<avg_cost);

                return statistics;
            }
            int adaptive_sparse_rrt_t::get_depth(tree_vertex_index_t v)
            {
                int count = 0;
                tree_vertex_index_t iter = v;
                while(iter!=start_vertex)
                {
                    iter = get_vertex(iter)->get_parent();
                    count++;
                }
                return count;
            }
            
            void adaptive_sparse_rrt_t::update_vis_info() const
            {
                rrt_t::update_vis_info();
                std::vector<geometry_info_t> geoms;
                std::vector<config_t> configs;
                hash_t<std::string, std::vector<double> > map_params;
                std::vector<double> params;

                int count;
                if( true )
                {
                    count = 0;
                    std::vector<std::string> system_names;
                    system_names.push_back(visualization_body);
                    foreach(tree_node_t* v, tree.vertices())
                    {
                        if (!v->as<adaptive_sparse_rrt_node_t>()->bridge)
                        {                            
                            if(vis_cost)
                            {
                                params.clear();
                                std::string name = "cost_" + int_to_str(count);
                                count++;
                                map_params.clear();
                                ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(v->point, system_names, map_params);
                                params.push_back(1);
                                params.push_back(1);
                                params.push_back(1);
                                geoms.push_back(geometry_info_t(visualization_body, name, PRX_BOX, params, vector_t(v->as<adaptive_sparse_rrt_node_t>()->cost/thresh,v->as<adaptive_sparse_rrt_node_t>()->cost/thresh,v->as<adaptive_sparse_rrt_node_t>()->cost/thresh,1)));
                                configs.push_back(config_t());
                                double offset = 0;
                                if(vis_near && !vis_drain)
                                    offset = .5;
                                if(!vis_near && vis_drain)
                                    offset = -.5;
                                configs.back().set_position(map_params[system_names[0]][0]+offset,map_params[system_names[0]][1],map_params[system_names[0]][2]);
                            }
                        }
                    }
                    ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->visualization_geom_map["drains"] = geoms;
                    ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->visualization_configs_map["drains"] = configs;
                    geoms.clear();
                    configs.clear();
                }
            }
            
        }
    }
}
