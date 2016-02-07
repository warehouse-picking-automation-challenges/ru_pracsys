/**
 * @file reachable_sparse_rrt.cpp
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

#include "reachable_sparse_rrt.hpp"
#include "reachable_sparse_rrt_statistics.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/statistics/image.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::sparse_rrt::reachable_sparse_rrt_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    namespace packages
    {
        namespace sparse_rrt
        {
            reachable_sparse_rrt_t::reachable_sparse_rrt_t()
            {
                statistics = new reachable_sparse_rrt_statistics_t();
            }

            reachable_sparse_rrt_t::~reachable_sparse_rrt_t()
            {
            }

            void reachable_sparse_rrt_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
            {
                rrt_t::init( reader, template_reader);
                num_trajs = parameters::get_attribute_as<unsigned >("num_trajectories",reader,template_reader);
                thresh = parameters::get_attribute_as<double >("cost_threshold",reader,template_reader,5);
                intersection_radius = parameters::get_attribute_as<double >("intersection_radius",reader,template_reader);
                cell_sizes = parameters::get_attribute_as<std::vector<double> >("cell_sizes",reader,template_reader);
                min_bound = parameters::get_attribute_as<std::vector<double> >("min_bound",reader,template_reader);
                max_bound = parameters::get_attribute_as<std::vector<double> >("max_bound",reader,template_reader);
                reach_steps = parameters::get_attribute_as<unsigned >("reach_steps",reader,template_reader,200);
                divisions.resize(cell_sizes.size());
                total_weight = 0;
                img_count=0;
                cell_sizes[0]=1;
                cell_sizes[1]=1;

            }

            void reachable_sparse_rrt_t::setup()
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
                tree.pre_alloc_memory<reachable_sparse_rrt_node_t,reachable_sparse_rrt_edge_t>(max_points);
                trajs.resize(num_trajs);
                for(unsigned i=0;i<num_trajs;i++)
                {
                    trajs[i] = new trajectory_t(state_space);
                }
                random_selection.clear();
                trajectory.link_space(state_space);
                additional_traj.link_space(state_space);
                plan.link_control_space(control_space);
                plan2.link_control_space(control_space);
                states_to_check.clear();
                states_to_check.push_back(state_space->clone_point(input_query->get_start_state()));
                sample_point = state_space->alloc_point();
                start_vertex = tree.add_vertex<reachable_sparse_rrt_node_t,reachable_sparse_rrt_edge_t>();
                tree[start_vertex]->point = state_space->clone_point(input_query->get_start_state());
                PRX_INFO_S ("Start state: " << state_space->print_point(tree[start_vertex]->point,3));

                clock.reset();
                for(unsigned i=0;i<max_points;i++)
                {
                    pre_alloced_points.push_back(state_space->alloc_point());
                }
                best_goal = start_vertex;
                real_solution = false;
                compute_reachability(start_vertex);


                //hack to see if things work.
                // for(int i=0;i<10;i++)
                // {
                //     pre_alloced_points[i]->at(0) = -240+i*5;
                //     pre_alloced_points[i]->at(1) = 120;
                //     pre_alloced_points[i]->at(2) = 0;
                //     PRX_INFO_S(pre_alloced_points[i]->at(0)<<" "<<get_vertex(start_vertex)->grid->get(pre_alloced_points[i]).time);
                // }
                // exit(0);



               random_selection.push_front(start_vertex);
               random_selection_cells.push_front(&(get_vertex(start_vertex)->grid->grid_weight));
               // if(get_vertex(start_vertex)->grid->in_bounds(input_query->get_goal()->get_goal_points()[0]) && get_vertex(start_vertex)->grid->get(input_query->get_goal()->get_goal_points()[0]).time != PRX_INFINITY)
               // {
               //     best_goal = tree.add_vertex<reachable_sparse_rrt_node_t,reachable_sparse_rrt_edge_t>();
               //     tree_edge_index_t new_e = tree.add_edge<reachable_sparse_rrt_edge_t>(start_vertex,best_goal);
               //     get_vertex(best_goal)->point = input_query->get_goal()->get_goal_points()[0];
               //     get_vertex(best_goal)->cost = get_vertex(start_vertex)->grid->get(get_vertex(best_goal)->point).time;
               //     get_vertex(start_vertex)->grid->get_controls(get_vertex(best_goal)->point,plan);
               //     get_edge(new_e)->plan = plan;
               //     real_solution = true;
               //     PRX_INFO_S("We found a goal at time: "<<clock.measure());
               // }

            }

            void reachable_sparse_rrt_t::resolve_query()
            {
                solution_number++;
                input_query->clear();
                tree_vertex_index_t new_v = best_goal;

                while(get_vertex(new_v)->get_parent() != new_v)
                {
                    input_query->path.copy_onto_front(get_vertex(new_v)->point);
                    tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(),new_v);
                    rrt_edge_t* edge = get_edge(e);

                    foreach(plan_step_t step, edge->plan)
                    {
                        input_query->plan.copy_onto_front(step.control,step.duration);
                    }
                    new_v = get_vertex(new_v)->get_parent();
                }
                input_query->path.copy_onto_front(get_vertex(new_v)->point);
                // local_planner->propagate(tree[start_vertex]->point,input_query->plan,input_query->path);

                // new_v = best_goal;

                // while(get_vertex(new_v)->get_parent() != new_v)
                // {
                //     input_query->path.copy_onto_front(get_vertex(new_v)->point);
                //     tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(),new_v);
                //     rrt_edge_t* edge = get_edge(e);

                //     foreach(plan_step_t step, edge->plan)
                //     {
                //         input_query->plan.copy_onto_front(step.control,step.duration);
                //     }
                //     new_v = get_vertex(new_v)->get_parent();
                // }

                // if(!state_space->equal_points(tree[best_goal]->point,input_query->path.back() ) )
                // {
                //     PRX_ERROR_S(state_space->print_point(tree[best_goal]->point));
                //     PRX_ERROR_S(state_space->print_point(input_query->path.back()));
                // }
            }

            void reachable_sparse_rrt_t::step()
            {
                tree_vertex_index_t nearest = nearest_vertex();

                //propagate toward the sampled point
                plan.clear();
                plan2.clear();
                state_t* end_state = pre_alloced_points[point_count];
                double prop_length=0;
                trajectory.clear();
                additional_traj.clear();
                if(collision_checking)
                {
                    if(get_vertex(nearest)->grid==NULL)
                    {
                        compute_reachability(nearest);
                    }
                    // get_vertex(nearest)->grid->random_selection(plan);
                    // local_planner->propagate(tree[nearest]->point,plan,trajectory);
                    // local_planner->steer(trajectory[trajectory.size()-1],sample_point,plan2,additional_traj,false);
                    local_planner->steer(tree[nearest]->point,sample_point,plan2,additional_traj,false);
                    trajectory+=additional_traj;
                    plan+=plan2;
                    state_space->copy_point(end_state,trajectory[trajectory.size()-1]);
                    prop_length = plan.length();
                }
                else
                {
                    //this branch is not correctly using controls from space_grid
                    // if(get_vertex(nearest)->grid==NULL)
                    // {
                    //     compute_reachability(nearest);
                    // }
                    // get_vertex(nearest)->grid->random_selection(plan);
                    // state_space->copy_point(end_state,plan.get_end_state());
                    // // local_planner->propagate_step(tree[nearest]->point,plan,end_state);
                    // local_planner->steer(end_state,sample_point,plan2,end_state,false);
                    // plan+=plan2;
                    // prop_length = plan.length();
                }

                //check if the trajectory is valid
                if( (!collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size()>1)) 
                    && (!real_solution || (real_solution && get_vertex(nearest)->cost + prop_length < get_vertex(best_goal)->cost) 
                        )
                    )
                {
                    tree_vertex_index_t v = tree.add_vertex<reachable_sparse_rrt_node_t,reachable_sparse_rrt_edge_t>();
                    tree_edge_index_t e = tree.add_edge<reachable_sparse_rrt_edge_t>(nearest,v);
                    reachable_sparse_rrt_node_t* node = get_vertex(v);
                    node->point = end_state;
                    // PRX_INFO_S("Added point: "<<state_space->print_point(end_state));
                    double new_cost = get_vertex(nearest)->cost + prop_length;
                    node->cost = new_cost;
                    get_edge(e)->plan = plan;
                    state_space->copy_point(states_to_check[0],end_state);
                    //check if better goal node
                    if(!real_solution && input_query->get_goal()->satisfied(end_state))
                    {
                        best_goal = v;
                        real_solution = true;
                        PRX_INFO_S("We found a goal at time: "<<clock.measure());
                        initialize_goal_path();
                    }
                   // else if(real_solution && get_vertex(best_goal)->cost > get_vertex(v)->cost && input_query->get_goal()->satisfied(end_state) )
                   // {
                   //     best_goal = v;
                   // }

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
                    point_count++;

                    compute_reachability(v);
                    // create_reach_region(v);
                    if(node->grid->grid_weight>PRX_ZERO_CHECK)
                    {
                        //metric->add_point(tree[v]);
                        random_selection.push_front(v);
                        random_selection_cells.push_front(&(get_vertex(v)->grid->grid_weight));
                        insert_into_list();
                        if(get_vertex(v)->grid->in_bounds(input_query->get_goal()->get_goal_points()[0]) && get_vertex(v)->grid->get(input_query->get_goal()->get_goal_points()[0])!=NULL)
                        {
                            if(!real_solution)
                            {
                                best_goal = tree.add_vertex<reachable_sparse_rrt_node_t,reachable_sparse_rrt_edge_t>();
                                tree_edge_index_t new_e = tree.add_edge<reachable_sparse_rrt_edge_t>(v,best_goal);
                                get_vertex(best_goal)->point = input_query->get_goal()->get_goal_points()[0];
                                get_vertex(best_goal)->cost = get_vertex(v)->grid->get(get_vertex(best_goal)->point)->time;
                                get_vertex(v)->grid->get_controls(get_vertex(best_goal)->point,plan);
                                get_edge(new_e)->plan = plan;
                                real_solution = true;

                                PRX_INFO_S("We found a goal at time: "<<clock.measure());
                                initialize_goal_path();
                            }
                            else if(real_solution && get_vertex(best_goal)->cost > get_vertex(v)->grid->get(input_query->get_goal()->get_goal_points()[0])->time )
                            {
                                best_goal = tree.add_vertex<reachable_sparse_rrt_node_t,reachable_sparse_rrt_edge_t>();
                                tree_edge_index_t new_e = tree.add_edge<reachable_sparse_rrt_edge_t>(v,best_goal);
                                get_vertex(best_goal)->point = input_query->get_goal()->get_goal_points()[0];
                                get_vertex(best_goal)->cost = get_vertex(v)->grid->get(get_vertex(best_goal)->point)->time;
                                get_vertex(v)->grid->get_controls(get_vertex(best_goal)->point,plan);
                                get_edge(new_e)->plan = plan;
                                initialize_goal_path();
                            }
                        }
                    }
                }
                iteration_count++;
                if(point_count == max_points)
                {
                    for(unsigned i=0;i<max_points;i++)
                    {
                        pre_alloced_points.push_back(state_space->alloc_point());
                    }
                    max_points*=2;
                }
            }

            void reachable_sparse_rrt_t::recompute_subtree(tree_vertex_index_t v, double update_val)
            {
                if(get_vertex(v)->grid!=NULL)
                {
                    get_vertex(v)->grid->recompute_grid();
                }
                foreach(tree_vertex_index_t child, tree[v]->get_children())
                {
                    if(get_vertex(v)->grid!=NULL)
                    {
                        get_vertex(child)->cost += update_val;
                    }
                    else
                    {
                        get_vertex(child)->cost = get_vertex(v)->cost + tree.get_edge_as<reachable_sparse_rrt_edge_t>(v,child)->plan.length();
                    }
                    recompute_subtree(child,update_val);
                } 
            }

            void reachable_sparse_rrt_t::compute_reachability(tree_vertex_index_t v)
            {
                reachable_sparse_rrt_node_t* node = get_vertex(v);
                state_t* init_state = node->point;
                plan.clear();
                plan.append_onto_front(reach_steps*simulation::simulation_step);
                plan[0].control->at(0) = control_space->get_bounds()[0]->get_upper_bound();
                plan[0].control->at(1) = control_space->get_bounds()[1]->get_lower_bound();
                double increment = (control_space->get_bounds()[1]->get_upper_bound() -control_space->get_bounds()[1]->get_lower_bound() )/num_trajs;
                for(unsigned i=0;i<num_trajs;i++)
                {
                    local_planner->propagate(init_state,plan,*trajs[i]);
                    plan[0].control->at(1) += increment;
                    if(collision_checking)
                        validity_checker->valid_trajectory(*trajs[i]);
                }
                //compute random trajectories
                init_grid(v);
                std::deque<unsigned> to_be_updated;
                unsigned remove_count=0;
                to_be_updated.clear();
                for(unsigned i=0;i<random_selection.size();i++)
                {
                    tree_vertex_index_t vertex = random_selection[i];
                    if(vertex!=v)
                    {
                        reachable_sparse_rrt_node_t* node2 = get_vertex(vertex);
                        double temp_value = node->grid->grid_weight;
                        double temp_value2 = node2->grid->grid_weight;
                        if(temp_value > PRX_ZERO_CHECK && temp_value2 > PRX_ZERO_CHECK && metric->distance_function(node->point,node2->point)<intersection_radius)
                        {
                            node->grid->intersect(node2->grid);
                            total_weight = total_weight + node->grid->grid_weight + node2->grid->grid_weight - temp_value2 - temp_value ;
                            if(fabs(node2->grid->grid_weight-temp_value2) > PRX_ZERO_CHECK)
                            {
                                to_be_updated.push_front(i);
                            }
                            if(node2->grid->grid_weight<PRX_ZERO_CHECK)
                            {
                                remove_count++;
                            }
                        }
                    }
                }
                foreach(unsigned val, to_be_updated)
                {
                    update_list(val);
                }
                for(unsigned i=0;i<remove_count;i++)
                {
                    if(*random_selection_cells[random_selection_cells.size()-1-i]>PRX_ZERO_CHECK)
                        PRX_ERROR_S(*random_selection_cells[random_selection_cells.size()-1-i]);
                }
                if(remove_count!=0)
                {
                    random_selection_cells.resize(random_selection_cells.size()-remove_count);
                    random_selection.resize(random_selection.size()-remove_count);
                }

                //rewiring code!!!
                // for(unsigned i=0;i<random_selection.size();i++)
                // {
                //     tree_vertex_index_t vertex = random_selection[i];
                //     if(vertex!=v)
                //     {
                //         reachable_sparse_rrt_node_t* node2 = get_vertex(vertex);
                //         double temp_value = node->grid->grid_weight;
                //         double temp_value2 = node2->grid->grid_weight;
                //         if(    temp_value > PRX_ZERO_CHECK 
                //             && temp_value2 > PRX_ZERO_CHECK 
                //             && metric->distance_function(node->point,node2->point)<intersection_radius
                //             && node->grid->in_bounds(node2->point)
                //             && node->grid->get(node2->point).node !=NULL
                //             && node->grid->get(node2->point).time < node2->cost
                //           )
                //         {
                //             double update_val = node->grid->get(node2->point).time - node2->cost ;
                //             node2->cost = node->grid->get(node2->point).time;
                //             tree.transplant(vertex,v);
                //             recompute_subtree(vertex,update_val);
                //         }
                //     }
                // }

            }

            void reachable_sparse_rrt_t::init_grid(tree_vertex_index_t v)
            {
                std::vector<bounds_t*> new_bounds;
                for(unsigned i=0;i<state_space->get_dimension();i++)
                    new_bounds.push_back(new bounds_t(get_vertex(v)->point->at(i)+min_bound[i],get_vertex(v)->point->at(i)+max_bound[i]));     

                foreach(trajectory_t* traj, trajs)
                {
                    unsigned size = traj->size();
                    for(unsigned i=0;i<size;i++)
                    {
                        state_t* state = (*traj)[i];
                        for(unsigned j=0;j<state_space->get_dimension();j++)
                            new_bounds[j]->expand(state->at(j));
                    }
                }
                for(unsigned i=0;i<state_space->get_dimension();i++)
                {
                    new_bounds[i]->set_bounds(  std::max(floor(new_bounds[i]->get_lower_bound()/cell_sizes[i])*cell_sizes[i] , state_space->get_bounds()[i]->get_lower_bound()),
                                                std::min((floor(new_bounds[i]->get_upper_bound()/cell_sizes[i])+1)*cell_sizes[i],state_space->get_bounds()[i]->get_upper_bound() ));

                    divisions[i] = (new_bounds[i]->get_upper_bound() - new_bounds[i]->get_lower_bound())/cell_sizes[i];
                }
                get_vertex(v)->grid = new space_grid_t(new_bounds,divisions);
                get_vertex(v)->grid->give_validity_checker(validity_checker,sample_point,collision_checking);
                get_vertex(v)->grid->add_node(tree.get_vertex_as<reachable_sparse_rrt_node_t>(v),control_space,simulation::simulation_step,reach_steps);
                total_weight+=get_vertex(v)->grid->grid_weight;

            }

            bool reachable_sparse_rrt_t::is_best_goal(tree_vertex_index_t v)
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


            bool reachable_sparse_rrt_t::is_leaf(tree_vertex_index_t v)
            {
                return (get_vertex(v)->get_children().size() == 0);
            }

            void reachable_sparse_rrt_t::remove_leaf(tree_vertex_index_t v)
            {
                if(get_vertex(v)->get_parent()!=v)
                {
                    rrt_edge_t* edge = get_edge(tree.edge(get_vertex(v)->get_parent(),v));
                    edge->plan.clear();
                }
                if(get_vertex(v)->grid!=NULL)
                {
                    delete get_vertex(v)->grid;
                    get_vertex(v)->grid = NULL;
                }
                tree.remove_vertex(v);
            }

            inline reachable_sparse_rrt_node_t* reachable_sparse_rrt_t::get_vertex(tree_vertex_index_t v) const
            {
                return tree.get_vertex_as<reachable_sparse_rrt_node_t>(v);
            }
            inline reachable_sparse_rrt_edge_t* reachable_sparse_rrt_t::get_edge(tree_edge_index_t e) const
            {
                return tree.get_edge_as<reachable_sparse_rrt_edge_t>(e);
            }

            void reachable_sparse_rrt_t::insert_into_list()
            {
                if(random_selection_cells.size()>=2)
                {
                    update_list(0);
                }
            }

            void reachable_sparse_rrt_t::update_list(unsigned index)
            {
                for(unsigned i=index;i<random_selection.size()-1;i++)
                {
                    if(*random_selection_cells[i] < *random_selection_cells[i+1])
                    {
                        int val1 = i;
                        int val2 = i+1;
                        double * temp_cells = random_selection_cells[val1];
                        tree_vertex_index_t temp = random_selection[val1];
                        random_selection[val1] = random_selection[val2];
                        random_selection_cells[val1] = random_selection_cells[val2];
                        random_selection[val2] = temp;
                        random_selection_cells[val2] = temp_cells;
                    }
                    else
                    {
                        break;
                    }
                }

            }

            void reachable_sparse_rrt_t::initialize_goal_path()
            {
                PRX_INFO_S("The goal changes: "<<best_goal);
                double cost = get_vertex(best_goal)->cost;
                std::vector<tree_vertex_index_t> leaves;
                foreach(tree_vertex_index_t v, random_selection)
                {
                    if(is_leaf(v))
                    {
                        leaves.push_back(v);
                    }
                }
                foreach(tree_vertex_index_t v, leaves)
                {
                    tree_vertex_index_t new_v = v;
                    tree_vertex_index_t temp;
                    while(is_leaf(new_v) && get_vertex(new_v)->cost > cost)
                    {
                        temp = get_vertex(new_v)->get_parent();
                        remove_leaf(new_v);
                        new_v = temp;
                    }
                }

                random_selection.clear();
                random_selection_cells.clear();

                PRX_WARN_S(tree.num_vertices());
                total_weight = 0;
                foreach(tree_node_t* node, tree.vertices())
                {
                    if((get_vertex(node->get_index())->grid)!=NULL)
                    {
                        random_selection.push_front(node->get_index());
                        random_selection_cells.push_front(&(get_vertex(random_selection[0])->grid->grid_weight));
                        total_weight += *random_selection_cells[0];
                        insert_into_list();
                    }
                }


            }

            tree_vertex_index_t reachable_sparse_rrt_t::nearest_vertex()
            {
                double val = uniform_random(0,total_weight);
                tree_vertex_index_t v;
                unsigned iter=0;
                while(val>0)
                {
                    v = random_selection[iter];
                    val -= *random_selection_cells[iter];
                    iter++;
                }
                return v;
            }

            const statistics_t* reachable_sparse_rrt_t::get_statistics()
            {
                PRX_INFO_S(total_weight);
                // image_t image(400,400);
                // foreach(tree_vertex_index_t v, random_selection)
                // {
                //     double x,y;
                //     state_t* state = get_vertex(v)->point;
                //     x = ((state->at(0)) / state_space->get_bounds()[0]->get_upper_bound())*200;
                //     y = -1* (state->at(1) / state_space->get_bounds()[1]->get_upper_bound())*200;
                //     unsigned char color;
                //     if(get_vertex(v)->cost <= thresh)
                //         color = 254*(get_vertex(v)->cost/thresh) + 1;
                //     else
                //         color = 255;
                //     for(double i=x-2;i<=x+2;i++)
                //     {
                //         for(double j=y-2;j<=y+2;j++)
                //         {
                //             int i_pixel = floor(i+.5);
                //             int j_pixel = floor(j+.5);
                //             image.color_pixel(i_pixel,j_pixel,color,color,color);
                //         }
                //     }
                // }


                // foreach(state_t* state, input_query->path)
                // {
                //     double x,y;
                //     x = ((state->at(0)) / state_space->get_bounds()[0]->get_upper_bound())*200;
                //     y =  -1*(state->at(1) / state_space->get_bounds()[1]->get_upper_bound())*200;
                //     unsigned char color;
                //     for(double i=x-1;i<=x+1;i++)
                //     {
                //         for(double j=y-1;j<=y+1;j++)
                //         {
                //             int i_pixel = floor(i+.5);
                //             int j_pixel = floor(j+.5);
                //             image.color_pixel(i_pixel,j_pixel,0,255,0);
                //         }
                //     }
                // }
                // std::stringstream s;
                // s<<img_count<<".pgm";
                // img_count++;
                // std::string dir(output_directory+s.str());
                // image.encode(dir.c_str());


                double avg_cost=0;
                int count = 0;
                foreach(tree_vertex_index_t v, random_selection)
                {
                    avg_cost += get_vertex(v)->cost;
                    count++;
                }

                avg_cost/=count;

                statistics = new reachable_sparse_rrt_statistics_t();
                statistics->as<reachable_sparse_rrt_statistics_t>()->solution_quality = get_vertex(best_goal)->cost;//input_query->plan.length();
                statistics->as<reachable_sparse_rrt_statistics_t>()->num_vertices = count;
                statistics->as<reachable_sparse_rrt_statistics_t>()->average_cost = avg_cost;
                statistics->time = clock.measure();
                statistics->steps = iteration_count;
                PRX_INFO_S("Stats: "<<clock.measure()<<" "<<iteration_count<<" "<<count<<" "<<get_vertex(best_goal)->cost<<" "<<avg_cost);

                return statistics;
            }

            void reachable_sparse_rrt_t::update_vis_info() const
            {
                rrt_t::update_vis_info();
                std::vector<geometry_info_t> geoms;
                std::vector<config_t> configs;
                hash_t<std::string, std::vector<double> > map_params;
                std::vector<double> params;
                // state_t* temp_state = state_space->alloc_point();
                int count;
                count = 0;
                std::vector<std::string> system_names;
                system_names.push_back(visualization_body);
                // foreach(tree_node_t* v, tree.vertices())
                // {
                //     if (v->as<reachable_sparse_rrt_node_t>()->num_cells!=0)
                //     {
                //         params.clear();
                //         std::string name = "cost_" + int_to_str(count);
                //         count++;
                //         map_params.clear();
                //         std::vector<util::bounds_t*> bounds = v->as<reachable_sparse_rrt_node_t>()->grid->point_bounds;
                //         double min,max;
                //         bounds[0]->get_bounds(min,max);
                //         temp_state->at(0) = (max+min)/2.0;
                //         params.push_back(max-min);
                //         bounds[1]->get_bounds(min,max);
                //         temp_state->at(1) = (max+min)/2.0;
                //         params.push_back(max-min);
                //         bounds[2]->get_bounds(min,max);
                //         temp_state->at(2) = (max+min)/2.0;
                //         PRX_INFO_S(state_space->print_point(temp_state));
                //         params.push_back(max-min);
                //         ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(temp_state, system_names, map_params);
                //         // params.push_back(1);
                //         geoms.push_back(geometry_info_t(visualization_body, name, PRX_BOX, params, vector_t(1,0,0,1)));
                //         configs.push_back(config_t());
                //         configs.back().set_position(map_params[system_names[0]][0],map_params[system_names[0]][1],0);//map_params[system_names[0]][2]);
                //     }
                // }
                
                // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->visualization_geom_map["drains"] = geoms;
                // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->visualization_configs_map["drains"] = configs;
                // geoms.clear();
                // configs.clear();

                // if( visualize_tree )
                // {
                //     PRX_WARN_S("Visualizing tree! " << visualization_body);
                //     count = 0;
                //     std::vector<std::string> system_names;
                //     system_names.push_back(visualization_body);

                //     foreach(tree_edge_t* e, tree.edges())
                //     {
                //         if(get_vertex(e->get_source())->num_cells!=0 || get_vertex(e->get_target())->num_cells!=0)
                //         {
                //             std::string name = visualization_tree_name + "/edge_" + int_to_str(count);
                //             params.clear();

                //             foreach(state_t* state, get_edge(e->get_index())->trajectory)
                //             {
                //                 map_params.clear();
                //                 ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(state, system_names, map_params);
                //                 params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                //             }

                //             geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, graph_color));
                //             configs.push_back(config_t());

                //             count++;
                //         }
                //     }
                //     ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->visualization_geom_map[visualization_tree_name] = geoms;
                //     ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->visualization_configs_map[visualization_tree_name] = configs;
                //     geoms.clear();
                //     configs.clear();
                // }
            }
        }
    }
}
