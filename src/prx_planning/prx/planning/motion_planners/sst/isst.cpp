/**
 * @file isst.cpp
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

#include "prx/planning/motion_planners/sst/isst.hpp"
#include "prx/planning/motion_planners/sst/sst_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

PLUGINLIB_EXPORT_CLASS( prx::plan::isst_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {
        
        isst_t::isst_t() 
        {
            statistics = new sst_statistics_t();
            drain = true;
            radius_nearest = true;
            count_of_failure = 0;
            img_count=0;
            time_elapsed = 0;
            steering = false;
            current_temp = 1;
            temp_rate = pow(2.0,.01);
            out_drain = 0;
            valid = 0;
            h_remove = 0;
            bnb_remove = 0;
            replaced = 0;
            first_solution_time=0;

        }

        isst_t::~isst_t() 
        { 
        }

        void isst_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
        {
            rrt_t::init( reader, template_reader);
            delta_drain = parameters::get_attribute_as<double>("delta_drain",reader,template_reader,0);
            delta_near = parameters::get_attribute_as<double>("delta_near",reader,template_reader,4);
            max_attempts = parameters::get_attribute_as<int>("max_attempts",reader,template_reader,100);
            drain = (delta_drain>PRX_ZERO_CHECK);
            radius_nearest = (delta_near>PRX_ZERO_CHECK);
            sample_metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "sample_metric" ,template_reader,"sample_metric");
            rrt_first = parameters::get_attribute_as<bool>("rrt_first",reader,template_reader,false);
            sst_only = parameters::get_attribute_as<bool>("sst_only",reader,template_reader,false);
            use_heuristic = parameters::get_attribute_as<bool>("use_heuristic",reader,template_reader,false);
            complex_heuristic = parameters::get_attribute_as<bool>("complex_heuristic",reader,template_reader,false);
            branch_and_bound = parameters::get_attribute_as<bool>("branch_and_bound",reader,template_reader,false);
            use_trrt_heuristic = parameters::get_attribute_as<bool>("use_trrt_heuristic",reader,template_reader,false);
            if(!rrt_first && !sst_only)
            {
                radius_nearest = false;
            }

        }

        void isst_t::setup()
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
            tree.pre_alloc_memory<sst_node_t,sst_edge_t>(max_points);
            trajectory.link_space(state_space);
            states_to_check.clear();
            states_to_check.push_back(state_space->clone_point(input_specification->get_seeds()[0]));
            sample_point = state_space->alloc_point();
            start_vertex = tree.add_vertex<sst_node_t,sst_edge_t>();
            get_vertex(start_vertex)->time_stamp = iteration_count;

            tree[start_vertex]->point = state_space->clone_point(input_specification->get_seeds()[0]);
            PRX_DEBUG_S ("Start state: " << state_space->print_point(tree[start_vertex]->point,3));
            min_cost = validity_checker->state_cost(tree[start_vertex]->point);
            max_cost = min_cost+1;
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
            //create the sample set
            sample_graph.pre_alloc_memory<informed_sample_point_t,sst_edge_t>(max_points);
            informed_sample_point_t* sample;
            tree_vertex_index_t v = sample_graph.add_vertex<informed_sample_point_t,sst_edge_t>();
            sample = sample_graph.get_vertex_as<informed_sample_point_t>(v);
            sample->point = state_space->clone_point(input_specification->get_seeds()[0]);
            PRX_DEBUG_S ("Start state: " << state_space->print_point(sample->point,3));
            sample->set = true;
            sample->memory = start_vertex;
            sample_metric->link_space(state_space);
            sample_metric->add_point(sample_graph[v]);

            //things for solution gathering
            best_goal = start_vertex;
            real_solution = false;
            last_stat = 0;

            PRX_INFO_S("state_space size: "<<state_space->get_dimension());
            PRX_INFO_S("control_space size: "<<control_space->get_dimension());

        }
        double isst_t::compute_cost()
        {
            tree_vertex_index_t best_goal = ((const tree_node_t*)metric->single_query(input_query->get_goal()->get_goal_points()[0]))->get_index();
            return get_vertex(best_goal)->cost;
        }

        void isst_t::resolve_query()
        {
            // solution_number++;
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
                // PRX_INFO_S("cost: "<<get_vertex(new_v)->cost<<" h_val_graph: "<<get_vertex(new_v)->h_value<<" h_val_function: "<<metric->distance_function(tree[new_v]->point,input_query->get_goal()->get_goal_points()[0]));
            }
                // PRX_INFO_S("----------");
            local_planner->propagate(tree[start_vertex]->point,input_query->plan,input_query->path);
        }

        void isst_t::step()
        {            
            if(iteration_count==0)
                clock.reset();  

            //sample a state
            if( !sst_only && roll_weighted_die(boost::assign::list_of(.1)(1 - .1), false) == 0 )
            {
                state_space->copy_point(sample_point, input_query->get_goal()->get_goal_points().front());
            }
            else
            {
                sampler->sample(state_space, sample_point);
            }

            //get the nearest state  
            tree_vertex_index_t nearest = nearest_vertex(sample_point);

            double best_val = PRX_INFINITY;
            //propagate toward the sampled point

            plan_t plan,plan2;
            trajectory_t traj2;
            plan.link_control_space(control_space);
            plan2.link_control_space(control_space);
            traj2.link_space(state_space);
            state_t* end_state = pre_alloced_points[point_number];
            double prop_length=0;
            int upper_val = 20;
            // if(!use_heuristic)
                upper_val = 1;
            for(int i=0;i<upper_val;i++)
            {
                traj2.clear();
                plan2.clear();
                local_planner->steer(tree[nearest]->point,sample_point,plan2,traj2,steering);
                double h_val = heuristic(traj2.back());
                if( h_val < best_val)
                {
                    plan = plan2;
                    trajectory = traj2;
                    state_space->copy_point(end_state,trajectory[trajectory.size()-1]);
                    prop_length = plan.length(); 
                    best_val = h_val;
                }  
            }  

            //from Simeon paper
            bool good_traj_cost = true;
            if(real_solution && use_trrt_heuristic)//!rrt_first)
            {
                trajectory_t::const_iterator i = trajectory.begin();
                trajectory_t::const_iterator j = trajectory.begin();
                j++;
                double start_cost;
                double end_cost;
                for ( ; j != trajectory.end(); ++i,++j)
                {
                    start_cost = validity_checker->state_cost(*i);
                    end_cost = validity_checker->state_cost(*j);
                    if(end_cost > max_cost)
                        max_cost = end_cost;
                    if(end_cost < min_cost)
                        min_cost = end_cost;
                    if(end_cost > start_cost)
                    {
                        if(exp(-(end_cost-start_cost)/current_temp) < .5)
                        {
                            current_temp *= temp_rate;
                            good_traj_cost = false;
                            break;
                        }
                    }
                }
                if(good_traj_cost)
                {
                    current_temp *= pow(2.0,-(end_cost-start_cost)/((max_cost-min_cost)));
                }
            }

            double traj_cost = validity_checker->trajectory_cost(trajectory);
            //check if the trajectory is valid
            out_drain++;
            if(!rrt_first && good_traj_cost)
            {
                out_drain--;
                double h_value = best_val;
                double new_cost = get_vertex(nearest)->cost + traj_cost;
                bnb_remove++;
                if(
                    (best_goal==start_vertex ||  
                    !branch_and_bound || 
                    new_cost + h_value < get_vertex(best_goal)->cost) 
                  )
                {      
                    bnb_remove--;
                //check for better things
                informed_sample_point_t* sample;
                tree_vertex_index_t sample_vertex = ((tree_node_t*)(sample_metric->single_query(end_state)))->get_index();
                sample = sample_graph.get_vertex_as<informed_sample_point_t>(sample_vertex);

                //if too far from a witness sample
                if(metric->distance_function(sample->point,end_state) > delta_drain)
                {
                    //add into list of graph points
                    tree_vertex_index_t v = sample_graph.add_vertex<informed_sample_point_t,sst_edge_t>();
                    sample = sample_graph.get_vertex_as<informed_sample_point_t>(v);
                    sample->point = state_space->clone_point(end_state);
                    sample_metric->add_point(sample_graph[v]);
                    // sample->h_value = heuristic(end_state);
                }

                //either we made a new witness sample, or this representative is better for the existing witness
                //   added a check for branch and bound
                h_remove++;
                if( (  
                    !(sample->set) || 
                    get_vertex(sample->memory)->cost + heuristic(sample->memory) > new_cost + h_value
                    ))
                {
                    h_remove--;
                valid++;
                    if(!collision_checking || ( traj_cost<PRX_INFINITY && validity_checker->is_valid(trajectory) && trajectory.size()>1))
                    {
                        valid--;

                            tree_vertex_index_t v = tree.add_vertex<sst_node_t,sst_edge_t>();
                            sst_node_t* node = get_vertex(v);
                            node->h_value = h_value;
                            node->point = end_state;
                            node->time_stamp = iteration_count;

                            point_number++;
                            node->bridge = true;
                            node->cost = new_cost;   
                            tree_edge_index_t e = tree.add_edge<sst_edge_t>(nearest,v);
                            get_edge(e)->plan = plan;            
                            state_space->copy_point(states_to_check[0],end_state);
                            //check if better goal node
                            if(!real_solution && input_query->get_goal()->satisfied(end_state))
                            {
                                best_goal = v;
                                real_solution = true;
                                time_elapsed+=clock.measure();
                                first_solution_time = time_elapsed;
                                PRX_INFO_S("First Solution Found at Time: "<<first_solution_time<<" s Iteration: "<<iteration_count);
                                purge_nodes();
                                clock.reset();

                            }
                            else if(real_solution && get_vertex(best_goal)->cost > get_vertex(v)->cost && input_query->get_goal()->satisfied(end_state) )
                            {
                                best_goal = v;
                                purge_nodes();
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
                            //optimization for sparsity
                            if(sample->set)
                            {
                                replaced++;
                                if(!get_vertex(sample->memory)->bridge )
                                {
                                    metric->remove_point(tree[sample->memory]);
                                    get_vertex(sample->memory)->bridge = true;
                                }
                                tree_vertex_index_t iter = sample->memory;
                                while( is_leaf(iter) && get_vertex(iter)->bridge && !is_best_goal(iter))
                                {
                                    tree_vertex_index_t next = get_vertex(iter)->get_parent();
                                    remove_leaf(iter);
                                    iter = next;
                                } 
                            }
                            sample->set = true;
                            sample->memory = v;

                            //add new node to metric
                            get_vertex(v)->bridge = false;
                            metric->add_point(tree[v]);
                        }
                    }
                }
            }
            else if(rrt_first)
            {
                out_drain--;
                if(!collision_checking || ( traj_cost<PRX_INFINITY && validity_checker->is_valid(trajectory) && trajectory.size()>1))
                {
                    double new_cost = get_vertex(nearest)->cost + traj_cost;
                    //check for better things
                    informed_sample_point_t* sample;
                    tree_vertex_index_t sample_vertex = ((tree_node_t*)(sample_metric->single_query(end_state)))->get_index();
                    sample = sample_graph.get_vertex_as<informed_sample_point_t>(sample_vertex);

                    //if too far from a witness sample
                    if(metric->distance_function(sample->point,end_state) > delta_drain)
                    {
                        //add into list of graph points
                        tree_vertex_index_t v = sample_graph.add_vertex<informed_sample_point_t,sst_edge_t>();
                        sample = sample_graph.get_vertex_as<informed_sample_point_t>(v);
                        sample->point = state_space->clone_point(end_state);
                        sample_metric->add_point(sample_graph[v]);
                        // sample->h_value = heuristic(end_state);
                    }

                    tree_vertex_index_t v = tree.add_vertex<sst_node_t,sst_edge_t>();
                    sst_node_t* node = get_vertex(v);
                    node->point = end_state;
                    node->time_stamp = iteration_count;

                    point_number++;
                    node->bridge = true;
                    node->cost = new_cost;   
                    tree_edge_index_t e = tree.add_edge<sst_edge_t>(nearest,v);
                    get_edge(e)->plan = plan;            
                    state_space->copy_point(states_to_check[0],end_state);
                    //check if better goal node
                    if(!real_solution && input_query->get_goal()->satisfied(end_state))
                    {
                        best_goal = v;
                        real_solution = true;
                        time_elapsed+=clock.measure();
                        first_solution_time = time_elapsed;
                        PRX_INFO_S("First Solution Found at Time: "<<first_solution_time<<" s Iteration: "<<iteration_count);
                        clock.reset();
                        purge_nodes();
                        time_elapsed+=clock.measure();
                        PRX_INFO_S("Pruning done at Time: "<<time_elapsed<<" s");
                        clock.reset();
                        sample->set = true;
                        sample->memory = v;

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
                    sample->visiting_nodes.push_back(v);

                    //add new node to metric
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

        void isst_t::remove_subtree(util::tree_vertex_index_t v)
        {
            bool new_goal=(v==best_goal);
            if(get_vertex(v)==NULL)
                return;
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
                    remove_subtree(child);
                }
            }
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


        void isst_t::prune_node(util::tree_vertex_index_t v)
        {
            if(!get_vertex(v)->bridge )
            {
                metric->remove_point(tree[v]);
                get_vertex(v)->bridge = true;
            }
            tree_vertex_index_t iter = v;
            while( is_leaf(iter) && get_vertex(iter)->bridge && !is_best_goal(iter))
            {
                tree_vertex_index_t next = get_vertex(iter)->get_parent();
                remove_leaf(iter);
                iter = next;
            } 
        }


        double isst_t::heuristic(const util::space_point_t* s)
        {
            if(!use_heuristic)
                return 0;

            if(complex_heuristic)
            {
                return h(s);
            }
            else
            {
                return validity_checker->heuristic(s,input_query->get_goal()->get_goal_points()[0]);
            }
        }
        double isst_t::fake_heuristic(const util::space_point_t* s)
        {
            return 0;
            // unsigned val = sample_metric->radius_and_closest_query(s,3*delta_drain, radial);
            // std::vector<double> distances;
            // std::vector<double> hs;

            // std::vector<const abstract_node_t*>::const_iterator iter,iter_end;
            // iter = radial.begin();
            // iter_end = iter;
            // std::advance(iter_end,val);
            // double sum = 0;
            // for(;iter!=iter_end;iter++)
            // {
            //     informed_sample_point_t* sample;
            //     tree_vertex_index_t v = ((const tree_node_t*)(*iter))->get_index();
            //     sample = sample_graph.get_vertex_as<informed_sample_point_t>(v);
            //     distances.push_back(metric->distance_function(s,sample->point));
            //     sum+=distances.back();
            //     hs.push_back(sample->h_value);
            // }
            // double h_val = 0;
            // for(unsigned i=0;i<distances.size();i++)
            // {
            //     h_val += ((sum - distances[i])/sum)*hs[i];
            // }
            // return h_val;
        }
        double isst_t::heuristic(tree_vertex_index_t s)
        {  
            if(!use_heuristic)
            {
                return 0;
            }
            if(complex_heuristic)
            {
                sst_node_t* n = get_vertex(s);
                if(n->h_value < 0)
                {
                    n->h_value = h(tree[s]->point);
                }
                return n->h_value;//metric->distance_function(s,get_vertex(best_goal)->point);
            }
            else
            {
                return validity_checker->heuristic(tree[s]->point,input_query->get_goal()->get_goal_points()[0]);
            }
        }

        void isst_t::purge_nodes()
        {
            if(sst_only)
                return;
            else if(!rrt_first && !sst_only)
            {
                radius_nearest = (delta_near>PRX_ZERO_CHECK);
            }
            if(rrt_first)
            {
                double best_goal_cost = get_vertex(best_goal)->cost;
                foreach(tree_node_t* node, sample_graph.vertices())
                {
                    informed_sample_point_t* sample = dynamic_cast<informed_sample_point_t*>(node);
                    tree_vertex_index_t best_vertex = start_vertex;
                    double best_cost = PRX_INFINITY;
                    for (std::vector<tree_vertex_index_t>::iterator i = sample->visiting_nodes.begin(); i != sample->visiting_nodes.end(); ++i)
                    {
                        double h = 0;// heuristic(*i);
                        if(get_vertex(*i)->cost + h < best_cost && (!branch_and_bound || get_vertex(*i)->cost+h <= best_goal_cost) )
                        {
                            best_vertex = *i;
                            best_cost = get_vertex(*i)->cost + h;
                        }
                    }
                    for (std::vector<tree_vertex_index_t>::iterator i = sample->visiting_nodes.begin(); i != sample->visiting_nodes.end(); ++i)
                    {
                        if(*i != best_vertex)
                        {
                            prune_node(*i);
                        }
                    }
                    if(best_vertex!=start_vertex)
                    {
                        sample->set = true;
                        sample->memory = best_vertex;
                    }
                }
                rrt_first = false;
            }
            else
            {
                double best_goal_cost = get_vertex(best_goal)->cost;
                foreach(tree_node_t* node, sample_graph.vertices())
                {
                    informed_sample_point_t* sample = dynamic_cast<informed_sample_point_t*>(node);
                    if(sample->set)
                    {
                        double h = 0;//heuristic(sample->memory);
                        if( branch_and_bound && get_vertex(sample->memory)->cost+h > best_goal_cost  )
                        {
                            prune_node(sample->memory);
                            sample->set = false;
                        }
                    }
                }

            }
        }

        bool isst_t::is_best_goal(tree_vertex_index_t v) const
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


        bool isst_t::is_leaf(tree_vertex_index_t v)
        {
            return (get_vertex(v)->get_children().size() == 0);
        }

        void isst_t::remove_leaf(tree_vertex_index_t v)
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

        inline sst_node_t* isst_t::get_vertex(tree_vertex_index_t v) const
        {
            return tree.get_vertex_as<sst_node_t>(v);
        }
        inline sst_edge_t* isst_t::get_edge(tree_edge_index_t e) const
        {
            return tree.get_edge_as<sst_edge_t>(e);
        }

        tree_vertex_index_t isst_t::nearest_vertex(state_t* state)
        {    
            if(radius_nearest && (sst_only || !rrt_first)) 
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
                        double temp = get_vertex(v)->cost + heuristic(v);
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

                // const std::vector< const abstract_node_t* > res = metric->multi_query(state, log(metric->get_nr_points())+1 );
                
                // tree_vertex_index_t ret_val;
                // std::vector<const abstract_node_t*>::const_iterator iter,iter_end;
                // iter = res.begin();
                // iter_end = res.end();
                // double length = PRX_INFINITY;
                // for(;iter!=iter_end;iter++)
                // {
                //     tree_vertex_index_t v = ((const tree_node_t*)(*iter))->get_index();
                //     double temp = get_vertex(v)->cost + heuristic(v);
                //     if( temp < length)
                //     {
                //         length = temp;
                //         ret_val = v;
                //     }
                // }


                // return ret_val;
            }
        }
        void isst_t::update_vis_info() const
        {
            rrt_t::update_vis_info();
            // std::vector<geometry_info_t> geoms;
            // std::vector<config_t> configs;
            // hash_t<std::string, std::vector<double> > map_params;
            // std::vector<double> params;

            // int count;
            // count = 0;
            // std::vector<std::string> system_names;
            // system_names.push_back(visualization_body);

            // foreach(tree_node_t* node, sample_graph.vertices())
            // {
            //     informed_sample_point_t* sample = dynamic_cast<informed_sample_point_t*>(node);
            //     if(sample->set)
            //     {
            //         std::string name = "sst_samples/s_" + int_to_str(count);
            //         params.clear();

            //         state_t* state = sample->point;

            //         map_params.clear();
            //         ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(state, system_names, map_params);
            //         params.push_back(delta_drain);
            //         params.push_back(5);
            //         if(is_best_goal(sample->memory))
            //         {
            //             geoms.push_back(geometry_info_t(visualization_body, name, PRX_OPEN_CYLINDER, params, "green"));
            //             configs.push_back(config_t(vector_t(map_params[system_names[0]][0],map_params[system_names[0]][1],map_params[system_names[0]][2]),quaternion_t()));

            //             count++;
            //         }
            //     }
            // }
            // ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["sst_samples"] = geoms;
            // ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["sst_samples"] = configs;
            // geoms.clear();
            // configs.clear();

        }
        const statistics_t* isst_t::get_statistics()
        {      
            time_elapsed+=clock.measure();

            double avg_cost=0;
            int count=0;
            double avg_time_stamp=0;
            double max_time=0;
            foreach(tree_node_t* v, tree.vertices())
            {
                if (!v->as<sst_node_t>()->bridge)
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

            statistics = new sst_statistics_t();
            statistics->as<sst_statistics_t>()->failure_count = count_of_failure;
            statistics->as<sst_statistics_t>()->num_vertices = tree.num_vertices();
            statistics->as<sst_statistics_t>()->solution_quality = get_vertex(best_goal)->cost;
            statistics->as<sst_statistics_t>()->average_cost = avg_cost;
            statistics->as<sst_statistics_t>()->best_near = delta_near;
            statistics->as<sst_statistics_t>()->drain = delta_drain;
            statistics->as<sst_statistics_t>()->avg_lifespan = avg_time_stamp;
            statistics->as<sst_statistics_t>()->first_solution = first_solution_time;
            statistics->as<sst_statistics_t>()->removed_by_validity = valid;
            statistics->as<sst_statistics_t>()->removed_by_drain = h_remove;
            statistics->as<sst_statistics_t>()->removed_by_bnb = bnb_remove;
            statistics->as<sst_statistics_t>()->removed_by_losing = replaced;


            sst_node_t* start_node = get_vertex(start_vertex);
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
            statistics->as<sst_statistics_t>()->children_of_root = child_count;
            statistics->as<sst_statistics_t>()->avg_distance_from_root = avg;
            statistics->time = time_elapsed;
            statistics->steps = iteration_count;
            count=0;

            std::stringstream out(std::stringstream::out);

            out<<"Stats from SST: \n-- Time Elapsed: "<<time_elapsed
                    <<" s \n-- Iteration Count: "<<iteration_count
                    <<" \n-- Number of Vertices: "<<tree.num_vertices()
                    <<" \n-- Solution Length: "<<get_vertex(best_goal)->cost<<" "<<heuristic(start_vertex)
                    <<" \n-- Average Node Cost: "<<avg_cost
                    <<" \n-- Delta_near: "<<delta_near
                    <<" \n-- Delta_drain: "<<delta_drain
                    <<" \n-- Average Lifespan (iterations): "<<avg_time_stamp
                    <<" \n-- Removed by distance: "<<out_drain
                    <<" \n-- Removed by validity: "<<valid
                    <<" \n-- Removed   heuristic: "<<h_remove
                    <<" \n-- Removed by BranchNB: "<<bnb_remove
                    <<" \n-- Removed by Replacem: "<<replaced
                    <<" \n-- Children of Root: "<<get_vertex(start_vertex)->get_children().size()
                    <<" \n-- Average Valence: "<<avg;
            PRX_INFO_S(out.str());
            clock.reset();
            last_stat = iteration_count;
            return statistics;
        }
    }
}
