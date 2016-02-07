/**
 * @file prm.cpp
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

#include "prx/planning/motion_planners/apc_planner/apc_planner.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>

#define X_IN 0.25
#define X_OUT 0.25
#define X_BIN 0.15
#define OFFSET 0.07

PLUGINLIB_EXPORT_CLASS(prx::plan::apc_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace packages::manipulation;

    namespace plan
    {

        apc_planner_t::apc_planner_t() : prm_star_t()
        {
            PRX_PRINT("In the constructor of APC planner", PRX_TEXT_CYAN);
            global_run_id = -1;

            online_version = false;
            collision_check_time = 0;
            safe_check_time = 0;
            check_count = 0;
            row_heights.resize(4);
            col_widths.resize(3);
            trusted_node = false;
            
            ik_failures = 0;

            the_model = NULL;
        }

        apc_planner_t::~apc_planner_t()
        {
            PRX_PRINT("Destroying APC planner", PRX_TEXT_CYAN);
            state_space->free_point(default_state);
        }

        void apc_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            PRX_PRINT("Initializing APC planner... ", PRX_TEXT_CYAN);
            prm_star_t::init(reader, template_reader);
            inside_samples = parameters::get_attribute_as<int>("inside_samples", reader, template_reader, 500);
            trajectory_subsampling = parameters::get_attribute_as<int>("trajectory_subsampling", reader, template_reader, 10);

            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/../object_models/");
            if( parameters::has_attribute("shelf_description_file", reader, template_reader) )
            {
                std::string file_name = dir + parameters::get_attribute("shelf_description_file", reader, template_reader);

                FILE * pFile;
                pFile = fopen(file_name.c_str(), "r");
                if( pFile != NULL )
                {
                    double trash;
                    int ret = fscanf(pFile, "{'x':%lf, 'y':%lf, 'z':%lf, 'roll':%lf, 'pitch':%lf, 'yaw':%lf, 'width':%lf, 'height':%lf, 'depth':%lf, 'rows':%lf, 'cols':%lf, 'a_col_width':%lf, 'b_col_width':%lf, 'c_col_width':%lf, 'a_row_height':%lf, 'd_row_height':%lf, 'g_row_height':%lf, 'j_row_height':%lf, 'shelfs_lip_height':%lf, 'support_column_width':%lf}"
                                     , &shelf_x, &shelf_y, &shelf_z, &trash, &trash, &trash
                                     , &trash, &trash, &trash, &trash, &trash
                                     , &col_widths[0], &col_widths[1], &col_widths[2]
                                     , &row_heights[3], &row_heights[2], &row_heights[1], &row_heights[0]
                                     , &shelfs_lip_height, &support_column_width

                                     );
                    fclose(pFile);
                }
                else
                {
                    PRX_FATAL_S("I could not read the file : " << file_name << "  for the shelf specification in the apc_planner!");
                }
            }
            else
            {
                PRX_FATAL_S("You need to specify shelf_description_file for the apc_planner in the folder: " << dir);
            }
        }

        void apc_planner_t::reset()
        {
            PRX_PRINT("Reseting APC planner... ", PRX_TEXT_CYAN);
            prm_star_t::reset();
        }

        void apc_planner_t::link_specification(specification_t* new_spec)
        {
            PRX_PRINT("Linking specification in the APC planner... ", PRX_TEXT_CYAN);
            prm_star_t::link_specification(new_spec);
            apc_local_planner = dynamic_cast<apc_local_planner_t*>(local_planner);
            if( apc_local_planner == NULL )
                PRX_FATAL_S("APC planner needs an apc_local_planner to run!");

            apc_sampler = dynamic_cast<apc_sampler_t*>(sampler);
            if( apc_sampler == NULL )
                PRX_FATAL_S("APC planner needs an apc_sampler to run!");
        }

        void apc_planner_t::setup_online_mode()
        {
            online_version = true;
        }

        void apc_planner_t::restart_astar()
        {
            astar->restart();
        }

        void apc_planner_t::setup()
        {
            PRX_PRINT("Setting up the APC planner... ", PRX_TEXT_CYAN);
            prm_star_t::setup();
            PRX_PRINT("Setted up the PRM* part of the APC planner... ", PRX_TEXT_CYAN);
            default_state = state_space->alloc_point();
            state_space->copy_to_point(default_state);
            final_state = state_space->alloc_point();

            PRX_PRINT("default_state: " << state_space->print_point(default_state, 5),PRX_TEXT_CYAN);
        }

        bool apc_planner_t::execute()
        {
            PRX_PRINT("Execute at the APC planner... ", PRX_TEXT_CYAN);
            online_version = false;

            try
            {
                prm_star_t::execute();
            }
            catch( stopping_criteria_t::stopping_criteria_satisfied e )
            {}
//            state_space->copy_from_point(default_state);
//            PRX_PRINT("default_state: " << state_space->print_point(default_state, 5),PRX_TEXT_CYAN);

            vector_t min_bin(3);
            vector_t max_bin(3);
            std::vector<double> y_centers(3);
            y_centers[0] = shelf_y + (col_widths[1] / 2.0) + support_column_width + (col_widths[0] / 2.0);
            y_centers[1] = shelf_y;
            y_centers[2] = shelf_y - (col_widths[1] / 2.0) - support_column_width - (col_widths[2] / 2.0);
            //I have read the z values of the bins from bottom to top
            double z_center = shelf_z;
            for( int r = 0; r < 4; ++r )
            {
                z_center += shelfs_lip_height + row_heights[r] / 2;

                for( int c = 0; c < 3; ++c )
                {
                    min_bin[0] = shelf_x - X_OUT;
                    min_bin[1] = y_centers[c] - OFFSET;
                    min_bin[2] = z_center - OFFSET;

                    max_bin[0] = shelf_x;
                    max_bin[1] = y_centers[c] + OFFSET;
                    max_bin[2] = z_center + OFFSET;
                    PRX_PRINT("BIN CENTER: x:" << shelf_x << "   y:" << y_centers[c] << "   z:" << z_center, PRX_TEXT_BLUE);
                    for( int i = 0; i < inside_samples; ++i )
                    {
                        valid_random_inside_sample(min_bin, max_bin, true);
                        trusted_node = true;
                        // PRX_PRINT("after the random inside sample : " << state_space->print_point(random_point,5) , PRX_TEXT_CYAN);
                        add_node(random_point);
                        // PRX_PRINT("The new node added is  trusted : "<< trusted_node, PRX_TEXT_BROWN);
                        graph.get_vertex_as<prm_star_node_t > (v_new)->trusted = trusted_node;
                        update_k(num_vertices);
                        // PRX_PRINT("PRM STAR outside the BIN iteration " << execute_num, PRX_TEXT_LIGHTGRAY);
                        execute_num++;
                    }

                }
                z_center += row_heights[r] / 2;
            }
            PRX_PRINT("Outside of the bin: " << ((double)ik_failures)/((double)num_generated), PRX_TEXT_MAGENTA );
            ik_failures = num_generated = 0;

//            state_space->copy_from_point(default_state);
//            PRX_PRINT("default_state: " << state_space->print_point(default_state, 5),PRX_TEXT_CYAN);

            z_center = shelf_z;
            for( int r = 0; r < 4; ++r )
            {
                z_center += shelfs_lip_height + row_heights[r] / 2;

                for( int c = 0; c < 3; ++c )
                {
                    min_bin[0] = shelf_x;
                    min_bin[1] = y_centers[c] - OFFSET;
                    min_bin[2] = z_center - OFFSET;

                    max_bin[0] = shelf_x + X_IN;
                    max_bin[1] = y_centers[c] + OFFSET;
                    max_bin[2] = z_center + OFFSET;
                    
                    PRX_PRINT("Sampling inside bin: " << r << " , " << c , PRX_TEXT_RED);
                    
                    for( int i = 0; i < inside_samples; ++i )
                    {
                        valid_random_inside_sample(min_bin, max_bin, false );
                        trusted_node = false;
                        // PRX_PRINT("after the random inside sample : " << state_space->print_point(random_point,5) , PRX_TEXT_CYAN);
                        add_node(random_point);
                        // PRX_PRINT("The new node added is  trusted : "<< trusted_node, PRX_TEXT_BROWN);
                        graph.get_vertex_as<prm_star_node_t > (v_new)->trusted = trusted_node;
                        update_k(num_vertices);
                        // PRX_PRINT("PRM STAR BIN iteration " << execute_num, PRX_TEXT_LIGHTGRAY);
                        execute_num++;
                    }
                }
                z_center += row_heights[r] / 2;
            }

            PRX_PRINT("Inside of the bin: " << ((double)ik_failures)/((double)num_generated), PRX_TEXT_MAGENTA );
            ik_failures = num_generated = 0;

            z_center = shelf_z;
            for( int r = 0; r < 4; ++r )
            {
                z_center += shelfs_lip_height + row_heights[r] / 2;

                for( int c = 0; c < 3; ++c )
                {
                    min_bin[0] = shelf_x - X_BIN;
                    min_bin[1] = y_centers[c] - OFFSET;
                    min_bin[2] = z_center - OFFSET;

                    max_bin[0] = shelf_x + X_BIN;
                    max_bin[1] = y_centers[c] + OFFSET;
                    max_bin[2] = z_center + OFFSET;
                    
                    PRX_PRINT("Sampling inside bin: " << r << " , " << c , PRX_TEXT_GREEN );
                    
                    for( int i = 0; i < inside_samples; ++i )
                    {
                        valid_random_inside_sample(min_bin, max_bin, false );
                        trusted_node = false;
                        // PRX_PRINT("after the random inside sample : " << state_space->print_point(random_point,5) , PRX_TEXT_CYAN);
                        add_node(random_point);
                        // PRX_PRINT("The new node added is  trusted : "<< trusted_node, PRX_TEXT_BROWN);
                        graph.get_vertex_as<prm_star_node_t > (v_new)->trusted = trusted_node;
                        update_k(num_vertices);
                        // PRX_PRINT("PRM STAR BIN iteration " << execute_num, PRX_TEXT_LIGHTGRAY);
                        execute_num++;
                    }
                }
                z_center += row_heights[r] / 2;
            }

            PRX_PRINT("At the entrance of the bin: " << ((double)ik_failures)/((double)num_generated), PRX_TEXT_MAGENTA );
            ik_failures = num_generated = 0;

            PRX_PRINT("Checks Time : " << (safe_check_time / check_count) << "collision_check_time : " << (collision_check_time / check_count), PRX_TEXT_BLUE);

//            state_space->copy_from_point(default_state);
//            PRX_PRINT("default_state: " << state_space->print_point(default_state, 5),PRX_TEXT_CYAN);

            return succeeded();
        }

        bool apc_planner_t::validate_roadmap( world_model_t* model )
        {
            //Let's assume we made something that's not stupid.
            unsigned invalids = 0;
            //Iterate over all the nodes at least
            foreach( undirected_vertex_index_t v, boost::vertices(graph.graph) )
            {
                if( !(model->valid_state( graph[v]->point )) )
                {
                    ++invalids;
                }
            }
            if( invalids > 0 )
            {
                PRX_PRINT("Roadmap has invalid states: " << invalids, PRX_TEXT_RED);
                return false;                
            }
            return true;
        }

        void apc_planner_t::step()
        {
            valid_random_sample();
            //PRX_PRINT("after the random sample : " << state_space->print_point(random_point,5) , PRX_TEXT_BROWN);
 
            add_node(random_point);
            // PRX_PRINT("The new node added is  trusted : "<< trusted_node, PRX_TEXT_BROWN);
            
            graph.get_vertex_as<prm_star_node_t > (v_new)->trusted = trusted_node;
            update_k(num_vertices);
            if( execute_num % 10 == 0 )
                PRX_PRINT("PRM STAR iteration " << execute_num, PRX_TEXT_LIGHTGRAY);
            execute_num++;
        }

        bool apc_planner_t::succeeded() const
        {
            // PRX_PRINT("Did the APC planner succeed?", PRX_TEXT_CYAN);
            return prm_star_t::succeeded();
        }

        void apc_planner_t::change_grasping_state( )
        {
            foreach( undirected_vertex_index_t vi, boost::vertices(graph.graph) )
                graph[vi]->point->at(16) = 1.0;
        }

        void apc_planner_t::validate_precomputation( )
        {
            foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
            {
                if( validity_checker->is_valid( graph[v]->point ) == false ) 
                {
                    graph.get_vertex_as<prm_star_node_t > (v)->trusted = false;
                    foreach(undirected_edge_index_t e, boost::out_edges(v, graph.graph))
                        graph.get_edge_as<prm_star_edge_t > (e)->trusted = false;

                    PRX_PRINT( "This vertex was IN COLLISION", PRX_TEXT_RED );
                }
            }

            foreach(undirected_edge_index_t e, boost::edges(graph.graph))
            {
                if(graph.get_edge_as<prm_star_edge_t > (e)->trusted)
                {
                    new_plan.clear();
                    path1.clear();
                    apc_local_planner->steer( graph[boost::source(e,graph.graph)]->point, graph[boost::target(e,graph.graph)]->point, new_plan, path1 );
                    if( validity_checker->is_valid(path1) == false )
                    {
                        graph.get_edge_as<prm_star_edge_t > (e)->trusted = false;  
                        PRX_PRINT( "This edge was IN COLLISION", PRX_TEXT_RED );
                    }
                }
            }
        }

        bool apc_planner_t::IK_steer_resolution( world_model_t* model, config_t end_config, bool grasping, bool camera_link )
        {
            //Determine the start's end effector configurations
            config_t start_config;
            state_t* prx_start_state = input_query->get_start_state();            
            _manipulator->get_state_space()->copy_from_point( prx_start_state );

            //PRX_PRINT( "copied start state and ready to compute corresponding configuration...", PRX_TEXT_LIGHTGRAY );

            // PRX_PRINT("For Camera Link: " << (camera_link ? "TRUE" : "FALSE"), PRX_TEXT_LIGHTGRAY );
            if( camera_link )
                _manipulator->get_camera_configuration( start_config );
            else
                _manipulator->get_end_effector_configuration( start_config );
            
            //PRX_PRINT( "Configuration is: " << start_config, PRX_TEXT_LIGHTGRAY );

            bool success = false;

            //Begin by assuming we have no plan.
/*            input_query->plan.clear();
            input_query->path.clear();
            
            // NEWER, BETTER (?) METHOD
            //Try to do the IK steering
            if( !( apc_local_planner->IK_steering_general( start_config, end_config, input_query->path, input_query->plan, validity_checker, grasping, camera_link ) ) )
            {
                PRX_PRINT("General IK failed either due to IK failures or collisions", PRX_TEXT_LIGHTGRAY);
                success = false;
            }
            else
            {
                PRX_PRINT("General IK sucessfully created a collision-free path!", PRX_TEXT_LIGHTGRAY);
                model->get_state_space()->copy_point( input_query->get_goal()->get_goal_points()[0], input_query->path.back() );
                //PRX_INFO_S("Computed trajectory: \n"<<input_query->path.print());
                  PRX_PRINT("Zhe STATE:: " << model->get_state_space()->print_point( input_query->path.back(), 4 ), PRX_TEXT_CYAN );
                success = true;
            }
            // PRX_PRINT("NEW METHOD CLOCKED AT: " << clock.measure(), PRX_TEXT_GREEN);
             return success;
*/
             input_query->plan.clear();
             input_query->path.clear();
            
             //clock.reset();
             //Try to do the IK steering
            if( !(_manipulator->IK_steering_general( start_config, end_config, input_query->plan, grasping, camera_link ) ) )
            {
//                  PRX_PRINT( "General Steering failed", PRX_TEXT_LIGHTGRAY );
            //     //If it failed, report such
                 input_query->plan.clear();
                 input_query->path.clear();
                 return false;
            }
            else
            {
                 //otherwise, generate the path that the system will follow
//                 PRX_PRINT( "General Steering succeeded", PRX_TEXT_LIGHTGRAY );
                 model->propagate_plan( prx_start_state, input_query->plan, input_query->path, true );
                 //And if it is in collision
                 if( input_query->path.in_collision() )
                 {
//                     PRX_PRINT( "The path was in collision", PRX_TEXT_LIGHTGRAY );
                     //Report failure
                     input_query->plan.clear();
                     input_query->path.clear();  
                     return false;                  
                 }
                 else
                 {
//                     PRX_PRINT( "The path was collision free", PRX_TEXT_LIGHTGRAY );
                     //PRX_INFO_S("Computed trajectory: \n"<<input_query->path.print());
                     model->get_state_space()->copy_point( input_query->get_goal()->get_goal_points()[0], input_query->path.back() );
                     //PRX_PRINT("STATE:: " << model->get_state_space()->print_point( input_query->path.back(), 4 ), PRX_TEXT_CYAN );
                     return true;
                }
            }
            //PRX_PRINT("OLD METHOD CLOCKED AT: " << clock.measure(), PRX_TEXT_RED);
            
        }

        /***************************************************************
         *****  RESOLVE QUERY
         **************************************************************/
        void apc_planner_t::resolve_query( space_t* object_space, world_model_t* model, bool grasping )
        {
            if( grasping )               
                the_model = model;
            else 
                the_model = NULL;

            global_run_id++;
            online_version = true;

            //PRX_PRINT("Resolving query in the APC planner with grasping " << grasping, PRX_TEXT_LIGHTGRAY);

            double path_length = 0;

            std::vector<space_point_t*> goals = input_query->get_goal()->get_goal_points();
            int nr_goals = ((multiple_goal_states_t*)(input_query->get_goal()))->get_number_of_points();
            std::vector<undirected_vertex_index_t> v_goals;

            no_collision_query_type = false;
            near_query_type = false;
/*
            if( grasping )
            {
                PRX_PRINT( "RELATIVE CONFIG before adding node", PRX_TEXT_GREEN );
                ((manipulation_simulator_t*)model->get_simulator())->print_relative_config();
            }
*/
            undirected_vertex_index_t v_start;
            boost::tie(remove_start, v_start) = add_node( input_query->get_start_state() );
/*
            if( grasping )
            {
                PRX_PRINT( "RELATIVE CONFIG after adding node", PRX_TEXT_GREEN );
                ((manipulation_simulator_t*)model->get_simulator())->print_relative_config();
            }
*/
            if( validity_checker->is_valid(input_query->get_start_state()) == false )
            {
                PRX_WARN_S("The start state is in collision!!");
                input_query->plan.clear();
                input_query->path.clear();
                return;
            }
            
            undirected_vertex_index_t v_g;
            bool remove_goal;

            bool have_goal = false;
            for( int i=0; i<nr_goals; i++ )
            {
                boost::tie(remove_goal, v_g) = add_node( goals[i] );
                if( validity_checker->is_valid( goals[i] ) == true )
                    have_goal = true;
                v_goals.push_back(v_g);
                remove_goals.push_back(remove_goal);
            }
            
            if( !have_goal )
            {
                PRX_WARN_S("All goals are in collision!!");
                input_query->plan.clear();
                input_query->path.clear();
                return;
            }
            
/*
            if( grasping )
            {
                PRX_PRINT( "RELATIVE CONFIG", PRX_TEXT_GREEN );
                ((manipulation_simulator_t*)model->get_simulator())->print_relative_config();
            }
*/

            boost::connected_components(graph.graph,graph.components);
            bool in_diff_components = true;
            int counter = 0;
            foreach( undirected_vertex_index_t v_g, v_goals )
            {
                if( graph.components[v_start] == graph.components[v_g] )
                {
                    in_diff_components = false;
                }
                counter++;
            }

/*            if( in_diff_components )
            {
                PRX_PRINT( "The start and the goal are in different components", PRX_TEXT_LIGHTGRAY );
            }
            else
            {
                PRX_PRINT( "The start and the goal are in the same component", PRX_TEXT_LIGHTGRAY );
            }
*/
            astar->link_graph(&graph);
            input_query->plan.clear();
            input_query->path.clear();

            bool found_path = false;
            int reruns = 0;
            while( !found_path )
            {
                astar->set_astar_mode(astar_module_t::PRX_NO_EDGE_INFO);
                if( !in_diff_components && astar->solve(v_start, v_goals) )
                {
                    std::deque< undirected_vertex_index_t > path_vertices;
                    astar->extract_path(v_start, astar->get_found_goal(), path_vertices);
                    //PRX_PRINT("ASTAR FOUND A PATH size:" << path_vertices.size(), PRX_TEXT_LIGHTGRAY);
                    bool valid_path = true;
                    path_length = 0;

                    for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                    {
                        //PRX_PRINT( "one propagate", PRX_TEXT_BLUE );
                        undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                        prm_star_edge_t* edge = graph.get_edge_as<prm_star_edge_t > (e);

                        path1.clear();
                        new_plan.clear();
                        apc_local_planner->steer( graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, final_state );

                        if( ( grasping || !(edge->trusted) ) && !edge->is_checked(global_run_id) )
                        {
                            model->propagate_plan( graph[path_vertices[i]]->point, new_plan, path1, true );

                            if( path1.in_collision() )
                            {                      
                                astar->block_edge(e);
                                valid_path = false;
                                //PRX_PRINT("Blocked edge in astar :: " << state_space->print_point(graph[path_vertices[i]]->point, 3) << " ], [" << state_space->print_point(graph[path_vertices[i + 1]]->point, 3), PRX_TEXT_LIGHTGRAY);
                            }
                            else
                            {
                                edge->checked = global_run_id;
                                //PRX_PRINT( "NOT in collision", PRX_TEXT_BLUE );
                            }
                        }
                        else if( grasping )
                        {
                            model->propagate_plan(graph[path_vertices[i]]->point, new_plan, path1, false );
                            //PRX_PRINT( "after trusted edge propagation", PRX_TEXT_BLUE );
                        }
                    }

                    found_path = valid_path;                    
                    //PRX_PRINT( "exited the loop with found_path " << found_path, PRX_TEXT_LIGHTGRAY );

                    if( found_path )
                    {
                        // PRX_PRINT("FOUND A PATH! Between " << path_vertices.size() << " states.", PRX_TEXT_GREEN);
                        for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                        {
                            //MOAR DEBUG
                            // PRX_PRINT("Jump between states::: " << path_vertices[i] << "  :  " << path_vertices[i+1], PRX_TEXT_CYAN );
                            // PRX_PRINT("::  " << state_space->print_point(graph[path_vertices[i]]->point, 5), PRX_TEXT_LIGHTGRAY );
                            // PRX_PRINT("::  " << state_space->print_point(graph[path_vertices[i+1]]->point, 5), PRX_TEXT_LIGHTGRAY );
                            // prm_star_edge_t* edge = graph.get_edge_as< prm_star_edge_t >( boost::edge( path_vertices[i], path_vertices[i+1], graph.graph).first );
                            // if( edge != NULL )
                            // {
                            //     PRX_PRINT("Trusted? :  " << edge->trusted << "  Checked? :  " << edge->checked, PRX_TEXT_LIGHTGRAY );
                            // }
                            // else
                            // {
                            //     PRX_PRINT("THERE IS NO FREAKIN' EDGE HERE!!!!!!!!!!!!!!!!!!!!!!!! FIX YOUR STUFF!!!!", PRX_TEXT_RED);
                            // }
                            //NORMAL CODE
                            path1.clear();
                            new_plan.clear();
                            apc_local_planner->steer(graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, path1); 
                            input_query->plan += new_plan;
                            if( i < path_vertices.size() - 2 )
                            {
                                path1.resize( path1.size() - 1 );
                            }
                            input_query->path += path1;

                            path_length += path1.length();
                        }
                    }
                    else
                    {
                        reruns++;
                        input_query->plan.clear();
                        input_query->path.clear();
                    }
                }
                else
                {
                    //There is no path and we have to stop searching
                    //PRX_PRINT("No solution to astar", PRX_TEXT_RED);
                    break;
                }
            }            
            //PRX_PRINT( "After calling A*, the object is at: " << model->get_active_space()->print_memory( 7 ), PRX_TEXT_BROWN );
//            PRX_PRINT( "RELATIVE CONFIG", PRX_TEXT_GREEN );
//            ((manipulation_simulator_t*)model->get_simulator())->print_relative_config();
//PRX_PRINT( "Found path " << found_path << "Start/goal in same component? " << (graph.components[v_start] == graph.components[v_goals[0]]) << " plan of size " << input_query->plan.size() << " and length " << path_length, PRX_TEXT_BROWN);

/*            if( found_path )
            {
                PRX_PRINT ( "Found path!", PRX_TEXT_LIGHTGRAY );
                
                //DEBUG DEBUG DEBUG SANITY CHECKS
                // for( unsigned i=0; i<input_query->path.size(); ++i )
                // {
                //     if( !( validity_checker->is_valid(input_query->path[i]) ) )
                //     {
                //         PRX_PRINT("Computed an invalid state: [" << i << "]:  " << state_space->print_point( input_query->path[i], 6 ), PRX_TEXT_RED );
                //     }
                // }
            }
            else
            {
                PRX_PRINT ( "No path found.", PRX_TEXT_LIGHTGRAY );                
            }
*/

            if( remove_start )
            {
                foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v_start, graph.graph))
                {
                    undirected_edge_index_t e = boost::edge(v_start, u, graph.graph).first;
                    graph.get_edge_as<prm_star_edge_t > (e)->clear(control_space);
                    e = boost::edge(u, v_start, graph.graph).first;
                    graph.get_edge_as<prm_star_edge_t > (e)->clear(control_space);
                }

                metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_start));
                graph.clear_and_remove_vertex(v_start);
                num_vertices--;
            }

            for( size_t i = 0; i < v_goals.size(); ++i )
            {
                if( remove_goals[i] )
                {
                    v_g = v_goals[i];

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v_g, graph.graph))
                    {
                        undirected_edge_index_t e = boost::edge(v_g, u, graph.graph).first;
                        graph.get_edge_as<prm_star_edge_t > (e)->clear(control_space);
                        e = boost::edge(u, v_g, graph.graph).first;
                        graph.get_edge_as<prm_star_edge_t > (e)->clear(control_space);
                    }
                    metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_g));
                    graph.clear_and_remove_vertex(v_g);
                    num_vertices--;
                }
            }

            v_goals.clear();
            remove_goals.clear();

            no_collision_query_type = false;
            near_query_type = false;

        }

        /***************************************************************
         **************************************************************/

        //MOAR DEBUG
        void apc_planner_t::print_graph( std::ostream& stream )
        {
            foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
            {
                stream << state_space->print_point( graph[v]->point, 3 ) << "\n";
            }
        }
        

        void apc_planner_t::resolve_query()
        {
            PRX_FATAL_S( "This should not be called for the APC." );
        }

        void apc_planner_t::update_vis_info() const
        {
            PRX_PRINT("Update vis info in the APC planner", PRX_TEXT_CYAN);
            prm_star_t::update_vis_info();
        }

        void apc_planner_t::valid_random_sample()
        {
            // PRX_PRINT("Generating a valid random sample in the APC planner", PRX_TEXT_CYAN);
            do
            {
                trusted_node = apc_sampler->safe_sample(state_space, random_point);
                impose_hand_state(random_point, true);
                ++num_generated;
            }
            while( !lazy_collision_mode && !(validity_checker->is_valid(random_point)) );
            // PRX_PRINT("sample: " << state_space->print_point(random_point, 5),PRX_TEXT_MAGENTA);
        }

        void apc_planner_t::valid_random_inside_sample(const util::vector_t& min_bounds, const util::vector_t& max_bounds, bool fix_gripper )
        {
            bool succeeded = true;
            do
            {
                succeeded = apc_sampler->sample_inside(random_point, min_bounds, max_bounds);
                impose_hand_state(random_point, fix_gripper );
                ++num_generated; //What for?
                if(!succeeded)
                {
                    ++ik_failures;
                }
            }
            while( !succeeded || ( !lazy_collision_mode && !(validity_checker->is_valid(random_point)) ) );

            // PRX_PRINT("sample: " << state_space->print_point(random_point, 5),PRX_TEXT_BLUE);
        }

        void apc_planner_t::impose_hand_state( space_point_t* target_state, bool fix_gripper )
        {
            // PRX_PRINT("target_state BEFORE: " << state_space->print_point(target_state, 5),PRX_TEXT_CYAN);
            if( is_left_arm )
            {
                for( int i=8; i<15; i++ )
                    target_state->at(i) = default_state->at(i);
                if( fix_gripper )
                    target_state->at(15) = default_state->at(15);
            }
            else
            {
                for( int i=1; i<8; i++ )
                    target_state->at(i) = default_state->at(i);
                target_state->at(15) = default_state->at(15);
            }
            // PRX_PRINT("target_state AFTER: " << state_space->print_point(target_state, 5),PRX_TEXT_GREEN);
       }


        std::pair<bool, util::undirected_vertex_index_t> apc_planner_t::add_node(const space_point_t* n_state)
        {
            return prm_star_t::add_node(n_state);            
        }

        void apc_planner_t::connect_node(undirected_vertex_index_t v)
        {
//            PRX_PRINT("--Connect a node in the APC planner", PRX_TEXT_CYAN);
            prm_star_t::connect_node(v);
        }

        void apc_planner_t::connect_node(undirected_vertex_index_t v, double rad)
        {
//            PRX_PRINT("--Connect a node in the APC planner", PRX_TEXT_CYAN);
            prm_star_t::connect_node(v, rad);
        }

        void apc_planner_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
        {
            const undirected_node_t* node;

            path1.clear();
/*
            if( the_model )
            {
                PRX_PRINT( "RELATIVE CONFIG in beginning of link node", PRX_TEXT_GREEN );
                ((manipulation_simulator_t*)the_model->get_simulator())->print_relative_config();
            }

            if( the_model )
            {
                PRX_PRINT( "From  state: " << state_space->print_point( graph[v]->point ), PRX_TEXT_MAGENTA );
            }
*/                

            int prev_edges = num_edges;
            for( size_t i = 0; i < neighbors.size(); i++ )
            {
                node = neighbors[i]->as< undirected_node_t > ();
                new_plan.clear();

                sys_clock_t checks_clock;
                checks_clock.reset();
                bool safe_path = false;
/*
                if( the_model )
                {
                    PRX_PRINT( "Going to state: " << state_space->print_point( node->point ), PRX_TEXT_MAGENTA );
                }
*/
                if( online_version )
                {
                    apc_local_planner->steer( graph[v]->point, node->point, new_plan, path1 );
                }
                else
                {
                    safe_path = apc_local_planner->safe_steer(graph[v]->point, node->point, new_plan, path1);
                    safe_check_time += checks_clock.measure_reset();
                    check_count++;
                }
/*
            if( the_model )
            {
                PRX_PRINT( "RELATIVE CONFIG -link node: after steering", PRX_TEXT_GREEN );
                ((manipulation_simulator_t*)the_model->get_simulator())->print_relative_config();
            }
*/

                //If the path is valid
                if( (new_plan.size() != 0  && ( lazy_collision_mode || is_valid_trajectory( path1 ) ) ) )
                {
                    if( !online_version )
                        collision_check_time += checks_clock.measure();
                    //Add the edge to the graph
                    // double dist = metric->distance_function(graph[v]->point, node->point);
                    double dist = validity_checker->trajectory_cost(path1);
                    undirected_edge_index_t e = graph.add_edge< prm_star_edge_t > (v, node->index, dist);
                    graph.get_edge_as<prm_star_edge_t > (e)->checked = -1;
                    graph.get_edge_as<prm_star_edge_t > (e)->id = num_edges;
                    graph.get_edge_as<prm_star_edge_t > (e)->trusted = true; //safe_path;
                    num_edges++;
                    if( visualize_graph )
                        graph.get_edge_as< prm_star_edge_t > (e)->path = path1;
                }
/*
            if( the_model )
            {
                PRX_PRINT( "RELATIVE CONFIG -link node: after collision checking", PRX_TEXT_GREEN );
                ((manipulation_simulator_t*)the_model->get_simulator())->print_relative_config();
            }
*/

                path1.clear();
            }
//            if( the_model ) 
//                PRX_PRINT( "at the end of link node, the object is at: " << the_model->get_active_space()->print_memory( 7 ), PRX_TEXT_BROWN );
            //PRX_PRINT( "New edges for added node " << num_edges - prev_edges, PRX_TEXT_MAGENTA );
        }

        bool apc_planner_t::is_valid_trajectory(const sim::trajectory_t& path)
        {
            //            PRX_PRINT("APC planner is checking for a valid trajectory", PRX_TEXT_CYAN);
            return prm_star_t::is_valid_trajectory(path);
        }

        bool apc_planner_t::serialize()
        {
            PRX_PRINT(" Inside the APC planner serialization function ", PRX_TEXT_CYAN);
            return prm_star_t::serialize();
        }

        bool apc_planner_t::deserialize()
        {
            PRX_PRINT(" Inside the APC planner deserialization function ", PRX_TEXT_CYAN);
            return prm_star_t::deserialize();
        }

        void apc_planner_t::link_manipulator(prx::packages::baxter::manipulator_plant_t* manipulator, bool is_left_arm)
        {
            PRX_PRINT("linking the manipulator to the apc_planner!", PRX_TEXT_BROWN);
            _manipulator = manipulator;
            this->is_left_arm = is_left_arm;
        }

        void apc_planner_t::generate_IK_data_base( IK_data_base_t& IK_base, IK_data_base_t& cam_IK_base )
        {
            const space_t* manip_space = _manipulator->get_state_space();
            config_t effector_config;
            unsigned index = 0;

            // unsigned ee_index;
            // unsigned cam_index;
            
            // //Crunch mode hack
            // if( is_left_arm )
            // {
            //     ee_index = 35;
            //     cam_index = 50;
            // }
            // else
            // {
            //     ee_index = 16;
            //     cam_index = 54;
            // }

            PRX_PRINT("Generate IK data base ...   manip_Space: " << manip_space->get_space_name() << "   space_dim: " << manip_space->get_dimension(),PRX_TEXT_CYAN);
            foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
            {
                // PRX_PRINT(manip_space->print_point(graph[v]->point,5),PRX_TEXT_GREEN);
                manip_space->copy_from_point(graph[v]->point);
                // PRX_PRINT("copy to memory",PRX_TEXT_GREEN);
                index = 0;

                _manipulator->get_end_effector_configuration( effector_config );
                IK_base.add_pair(manip_space, effector_config, graph[v]->point);
                _manipulator->get_camera_configuration( effector_config );
                cam_IK_base.add_pair(manip_space, effector_config, graph[v]->point);
            }
            PRX_PRINT("Done with nodes : " << boost::num_vertices(graph.graph), PRX_TEXT_CYAN);


/*            int new_IKs = 0;
              foreach(undirected_edge_index_t e, boost::edges(graph.graph))
              {
                path1.clear();
                new_plan.clear();
                apc_local_planner->steer(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point, new_plan, path1);

                // foreach(state_t* state, path1)
                for(int i=trajectory_subsampling; i<path1.size(); i+=trajectory_subsampling)     
                {
                    manip_space->copy_from_point(path1[i]);
                    index = 0;
                    _manipulator->update_phys_configs( config_list, index );
                    IK_base.add_pair(manip_space, config_list[ee_index].second, path1[i]);
                    cam_IK_base.add_pair(manip_space, config_list[cam_index].second, path1[i]);
                    new_IKs++;
                }
            }
            PRX_PRINT("Done with edges : " << boost::num_edges(graph.graph) << "   added:" << new_IKs << "  new IKs", PRX_TEXT_CYAN);
*/

        }

        const statistics_t* apc_planner_t::get_statistics()
        {
            // PRX_PRINT(" Inside the APC planner's get_statistics function", PRX_TEXT_CYAN);
            return prm_star_t::get_statistics();
        }

        void apc_planner_t::set_param(const std::string& parameter_name, const boost::any& value)
        {

            prm_star_t::set_param(parameter_name, value);
        }
    }
}
