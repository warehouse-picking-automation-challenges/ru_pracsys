/**
 * @file sst.cpp
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

#include "prx/planning/motion_planners/sst/sst.hpp"
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

PLUGINLIB_EXPORT_CLASS( prx::plan::sst_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {
        
        sst_t::sst_t() 
        {
            statistics = new sst_statistics_t();
            drain = true;
            radius_nearest = true;
            count_of_failure = 0;
            img_count=0;
            time_elapsed = 0;
            steering = false;
        }

        sst_t::~sst_t() 
        { 
        }

        void sst_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
        {
            rrt_t::init( reader, template_reader);
            delta_drain = parameters::get_attribute_as<double>("delta_drain",reader,template_reader,0);
            delta_near = parameters::get_attribute_as<double>("delta_near",reader,template_reader,4);
            max_attempts = parameters::get_attribute_as<int>("max_attempts",reader,template_reader,100);
            drain = (delta_drain>PRX_ZERO_CHECK);
            radius_nearest = (delta_near>PRX_ZERO_CHECK);
            sample_metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "sample_metric" ,template_reader,"sample_metric");
        }

        void sst_t::setup()
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
            sample_graph.pre_alloc_memory<sample_point_t,sst_edge_t>(max_points);
            sample_point_t* sample;
            tree_vertex_index_t v = sample_graph.add_vertex<sample_point_t,sst_edge_t>();
            sample = sample_graph.get_vertex_as<sample_point_t>(v);
            sample->point = state_space->clone_point(input_specification->get_seeds()[0]);
            PRX_DEBUG_S ("Start state: " << state_space->print_point(sample->point,3));
            sample->set = true;
            sample->memory = start_vertex;
            sample_metric->link_space(state_space);
            sample_metric->add_point(sample_graph[v]);

            //things for solution gathering
            best_goal = start_vertex;
            real_solution = false;
            sprint_iteration_counter = 0;
            sprint_count = 0;
            last_stat = 0;

        }

        void sst_t::hacked_resolve()
        {
                input_query->clear();
                tree_vertex_index_t best_goal = nearest_vertex(input_query->get_goal()->get_goal_points()[0]);
                input_query->solution_cost = get_vertex(best_goal)->cost;
        }
        void sst_t::resolve_query()
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
            }
            local_planner->propagate(tree[start_vertex]->point,input_query->plan,input_query->path);
        }

        void sst_t::step()
        {                   
            //sample a state
            // if( !real_solution && roll_weighted_die(boost::assign::list_of(.1)(1 - .1), false) == 0 )
            // {
            //     state_space->copy_point(sample_point, input_query->get_goal()->get_goal_points().front());
            // }
            // else
            // {
            //     sampler->sample(state_space, sample_point);
            // }
            sampler->sample(state_space,sample_point);

            // //get the nearest state  
            tree_vertex_index_t nearest = nearest_vertex(sample_point);
            //propagate toward the sampled point
            plan_t plan;
            plan.link_control_space(control_space);
            state_t* end_state = pre_alloced_points[point_number];
            double prop_length=0;
            int attempts=0;
            trajectory.clear();
            //small optimization where we try to make sure we leave the drain ball.
            do
            {
                //keep the trajectory if collision checking
                if(collision_checking)
                {
                    plan.clear();
                    local_planner->steer(tree[nearest]->point,sample_point,plan,trajectory,steering);
                    state_space->copy_point(end_state,trajectory[trajectory.size()-1]);
                    prop_length = validity_checker->trajectory_cost(trajectory);         
                }
                //just keep the end state
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

            //**** finding sample node
            double new_cost = get_vertex(nearest)->cost + prop_length;
            //check for better things
            sample_point_t* sample;
            tree_vertex_index_t sample_vertex = ((tree_node_t*)(sample_metric->single_query(end_state)))->get_index();
            sample = sample_graph.get_vertex_as<sample_point_t>(sample_vertex);

            //if too far from a witness sample
            if(metric->distance_function(sample->point,end_state) > delta_drain)
            {
                //add into list of graph points
                tree_vertex_index_t v = sample_graph.add_vertex<sample_point_t,sst_edge_t>();
                sample = sample_graph.get_vertex_as<sample_point_t>(v);
                sample->point = state_space->clone_point(end_state);
                sample_metric->add_point(sample_graph[v]);
            }

            //either we made a new witness sample, or this representative is better for the existing witness
            if((!(sample->set)) || get_vertex(sample->memory)->cost > new_cost )
            {    
                //check if the trajectory is valid
                if(!collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size()>1))
                {  
                    // PRX_INFO_S(5);
                    tree_vertex_index_t v = tree.add_vertex<sst_node_t,sst_edge_t>();
                    sst_node_t* node = get_vertex(v);
                    node->point = end_state;
                    node->time_stamp = iteration_count;

                    point_number++;
                    node->bridge = true;
                    node->cost = get_vertex(nearest)->cost + prop_length;   
                    tree_edge_index_t e = tree.add_edge<sst_edge_t>(nearest,v);
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
                    //optimization for sparsity
                    if(sample->set)
                    {
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

        void sst_t::remove_subtree(util::tree_vertex_index_t v)
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

        bool sst_t::is_best_goal(tree_vertex_index_t v)
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


        bool sst_t::is_leaf(tree_vertex_index_t v)
        {
            return (get_vertex(v)->get_children().size() == 0);
        }

        void sst_t::remove_leaf(tree_vertex_index_t v)
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

        inline sst_node_t* sst_t::get_vertex(tree_vertex_index_t v) const
        {
            return tree.get_vertex_as<sst_node_t>(v);
        }
        inline sst_edge_t* sst_t::get_edge(tree_edge_index_t e) const
        {
            return tree.get_edge_as<sst_edge_t>(e);
        }

        tree_vertex_index_t sst_t::nearest_vertex(state_t* state)
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
        void sst_t::update_vis_info() const
        {
            rrt_t::update_vis_info();

        }
        const statistics_t* sst_t::get_statistics()
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
                    <<" \n-- Solution Length: "<<get_vertex(best_goal)->cost
                    <<" \n-- Average Node Cost: "<<avg_cost
                    <<" \n-- Delta_near: "<<delta_near
                    <<" \n-- Delta_drain: "<<delta_drain
                    <<" \n-- Average Lifespan (iterations): "<<avg_time_stamp
                    <<" \n-- Children of Root: "<<get_vertex(start_vertex)->get_children().size()
                    <<" \n-- Average Valence: "<<avg;
            PRX_INFO_S(out.str());
            clock.reset();
            last_stat = iteration_count;
            return statistics;
        }
    }
}
