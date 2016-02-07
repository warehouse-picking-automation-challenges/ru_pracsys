/**
 * @file apc_local_planner.cpp
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

#include "prx/planning/modules/local_planners/apc_local_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

#include "prx/simulation/plan.hpp"

#include <pluginlib/class_list_macros.h> 

 PLUGINLIB_EXPORT_CLASS(prx::plan::apc_local_planner_t, prx::plan::local_planner_t)

 namespace prx
 {
    using namespace util;
    using namespace sim;
    namespace plan
    {

        apc_local_planner_t::apc_local_planner_t() { }

        apc_local_planner_t::~apc_local_planner_t()
        {
            if( prev_st != NULL )
            {
                _manipulator->get_state_space()->free_point( prev_st );
                _manipulator->get_state_space()->free_point( inter_st );
            }
        }

        void apc_local_planner_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            local_planner_t::init(reader, template_reader);
            prev_st = NULL;
            inter_st = NULL;
        }
        
        void apc_local_planner_t::link_info(prx::packages::baxter::manipulator_plant_t* manip, const util::vector_t& min_bounds, const util::vector_t& max_bounds)
        {
            _manipulator = manip;
            this->min_bounds = min_bounds;
            this->max_bounds = max_bounds;
        }

        void apc_local_planner_t::steer( const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj, bool connect )
        {
            _manipulator->steering_function( start, goal, plan );
//            PRX_PRINT( "After manipulator's steering function", PRX_TEXT_BROWN );
            if( !connect && plan.length() > 0 )
            {
                plan.trim(max_prop_length);
            }
            world_model->propagate_plan(start, plan, traj);
//            PRX_PRINT( "After world model's propagate function", PRX_TEXT_BROWN );
            // PRX_INFO_S("Result state: "<<world_model->get_state_space()->print_point(traj.back())<<" Actual state: "<<world_model->get_state_space()->print_point(goal));
        }

        void apc_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, state_t* result, bool connect)
        {
//            _manipulator->get_state_space()->copy_point( helpful_state, start );
            _manipulator->steering_function(start, goal, plan);
//            PRX_PRINT( "After manipulator's steering function 2", PRX_TEXT_BROWN );
            if( !connect && plan.length() > 0)
            {
                plan.trim(max_prop_length);
            }
            world_model->propagate_plan(start, plan, result);
//            PRX_PRINT( "After manipulator's steering function 2", PRX_TEXT_BROWN );
        }

        bool apc_local_planner_t::safe_steer(const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj)
        {
            world_model->steering_function(start, goal, plan);
            world_model->propagate_plan(start, plan, traj);
            
            config_list_t conf_list;
            unsigned index =0;
            
            util::config_t effector_config;
            const space_t* manip_space = _manipulator->get_state_space();
            foreach(state_t* state, traj)
            {
                manip_space->copy_from_point(state);
                
                index = 0;
                _manipulator->update_phys_configs( conf_list, index );
                for( unsigned i=0; i<conf_list.size(); ++i )
                {
                    vector_t effector_pos = conf_list[i].second.get_position();
                    if(effector_pos > min_bounds && effector_pos < max_bounds)
                        return false;
                }
            }
            return true;
        }

        bool apc_local_planner_t::IK_steering_general( config_t& start_config, config_t& end_config, trajectory_t& result_path, plan_t& result_plan, validity_checker_t* validity_checker, bool grasping, bool camera_link )
        {
            // PRX_PRINT(" ======================= APC LP IK =======================", PRX_TEXT_CYAN); 
            //Begin by determining a distance between the start and goal configurations
            double distance = start_config.get_position().distance( end_config.get_position() );
            //Set the number of interpolation steps
            double steps = std::ceil(distance / 0.01); //NOTE: we might need a way to read this from the manipulator at some point...
            //Clear out the plan we were given, we will be filling it in ourselves
            result_plan.clear();
            result_path.clear();

            const space_t* manip_space = _manipulator->get_state_space();
            temp_plan.link_control_space( _manipulator->get_control_space() );
            temp_traj.link_space( manip_space );
            temp_plan.clear();
            temp_traj.clear();

            //Make sure we have the point allocated for our previous state
            if( prev_st == NULL )
            {
                prev_st = manip_space->alloc_point();
                inter_st = manip_space->alloc_point();
            }

            //Store the arm's current state as the previous state
            manip_space->copy_to_point(prev_st);

            temp_traj.copy_onto_back( prev_st );

            //So first, let's do the IK for the start configuration
            if( ( camera_link && !_manipulator->get_utility_IK( start_config, prev_st, grasping, prev_st, true ) ) ||
                (!camera_link && !_manipulator->IK_solver( start_config, prev_st, grasping, prev_st, true ) ) )
            {
                PRX_DEBUG_COLOR("IK failed in Steering for the start config.", PRX_TEXT_RED);
                result_plan.clear();
                result_path.clear();
                return false;
            }
                        
            //Storage for the interpolated configuration
            config_t interpolated_config;

            //Then, for each step
            for( double i=1; i<=steps; ++i )
            {
                //Compute the next interpolated configuration
                double t = PRX_MINIMUM( (i/steps), 1.0 );
                interpolated_config.interpolate( start_config, end_config, t );

                //Compute Minimum IK for interpolated configuration (Pose, seed state, soln)
                if( ( camera_link && _manipulator->get_utility_IK( interpolated_config, inter_st, grasping, prev_st, true ) ) ||
                    (!camera_link && _manipulator->IK_solver( interpolated_config, inter_st, grasping, prev_st, true ) ) )
                {
                    temp_plan.clear();
                    temp_traj.clear();

                    steer( prev_st, inter_st, temp_plan, temp_traj );

                    if( validity_checker->is_valid( temp_traj ) )
                    {
                        result_plan += temp_plan;
                        result_path += temp_traj;

                        if( i < steps )
                            result_path.chop( result_path.size() - 1 );

                        manip_space->copy_point(prev_st, inter_st);
                    }
                    else
                    {
                        //Let's go ahead and abort out early
                        PRX_DEBUG_COLOR("IK Steering ran into a collision!", PRX_TEXT_MAGENTA);
                        result_plan.clear();
                        result_path.clear();
                        return false;
                    }
                }
                //If the MinIK fails, then the steering is a failure, abort
                else
                {
                    //Clear out whatever plan we had started making
                    PRX_DEBUG_COLOR("IK failed in Steering", PRX_TEXT_BROWN);
                    result_plan.clear();
                    result_path.clear();
                    return false;
                }
            }
            //Report our Success
            return true;
        }

    }
}

