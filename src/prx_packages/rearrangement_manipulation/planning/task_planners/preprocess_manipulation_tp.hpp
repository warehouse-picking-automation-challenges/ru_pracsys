/**
 * @file preprocess_manipulation_tp_t.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_PREPROCESS_MANIPULATION_TP_HPP
#define PRX_PREPROCESS_MANIPULATION_TP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"

#include "prx/planning/task_planners/task_planner.hpp"

//From different packages
#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "../../../manipulation/simulation/plants/movable_body_plant.hpp"

namespace prx
{

    namespace util
    {
        class bounds_t;
        class goal_state_t;
        class statistics_t;
    }
    
    namespace plan
    {
        class motion_planning_specification_t;
    }

    namespace packages
    {
        namespace baxter
        {
            class manipulator_plant_t;
        }

        namespace manipulation
        {
            class movable_body_plant_t;
            class manip_sampler_t;
            class pose_t;
        }

        namespace rearrangement_manipulation
        {
            class manipulation_mp_specification_t;
            class rearrangement_manipulation_specification_t;            
            class obstacle_aware_astar_t;
            class system_name_validity_checker_t;
            class manipulation_mp_t;

            /**
             * Manipulation task planner. Computes the path for moving an object from an 
             * initial to a target position.             
             * 
             * @autors Athanasios Krontiris
             */
            class preprocess_manipulation_tp_t : public plan::task_planner_t
            {

              public:

                preprocess_manipulation_tp_t();
                virtual ~preprocess_manipulation_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc motion_planner_t::reset() 
                 */
                virtual void reset();
                
                /**
                 * @copydoc planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* new_spec);

                /** 
                 * @copydoc motion_planner_t::setup() 
                 * 
                 * Will occupy memory for the random_open_point and the new_control, after 
                 * planning_query has been linked. 
                 */
                virtual void setup();

                /**
                 * @copydoc motion_planner_t::execute()
                 */
                virtual bool execute();
                
                /**
                 * @copydoc motion_planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();
                
                /**
                 * @copydoc motion_planner_t::succeeded() const 
                 */
                virtual bool succeeded() const;

                /** 
                 * @copydoc motion_planner_t::resolve_query() 
                 *
                 * At the end of the resolve_query the algorithm will remove the vertices 
                 * for the start and the goal for this specific query from the graph.
                 */
                virtual void resolve_query();

                bool serialize();
                bool deserialize();

              protected:

                baxter::manipulator_plant_t* _manipulator;
                manipulation::movable_body_plant_t* _object;
                manipulation::movable_body_plant_t* collision_object;

                /** @brief Reads from input the name of the planning context that is for the manipulator only.*/
                std::string pc_name_manipulator_only;
                /** @brief Plannable manipulator and object.*/
                std::string pc_name_manipulator_with_object;
                /** @brief The planning context that is for the transit state.*/
                std::string pc_name_transit;
                /** @brief The planning context that is for the transfer state.*/
                std::string pc_name_transfer;
                /** @brief A helping planning context that helps to get the spaces for the objects.*/
                std::string pc_name_objects;
                /** @brief planning context only for one object.*/
                std::string pc_name_object_only;

                /** @brief The state space over the manipulator that the manipulation task planner is using. */
                const util::space_t* manip_state_space;
                /** @brief The control space over the manipulator that the manipulation task planner is using. */
                const util::space_t* manip_control_space;
                /** @brief The state space for the manipulator and an object.*/
                const util::space_t* mo_space;
                /** @brief The state space for the object that the manipulator can transfer.*/
                const util::space_t* object_state_space;
                /** @brief The state space for the object that will be used to check collisions to inform the graph.*/
                const util::space_t* collision_object_space;
                /** @brief The space for sampling random poses.*/
                util::space_t* stable_pose_space;

                //Helping variables for state points.
                sim::state_t* manip_state;
                sim::state_t* object_state;
                /** @brief The safe state for the manipulator.*/
                sim::state_t* safe_state;
                /** @brief The random sampled state for a pose.*/
                sim::state_t* stable_pose_state;
                /** @brief Helping variable for checking for collisions between poses while informing the graph.*/
                sim::state_t* collision_check_state;

                //Temporary storages for random samples.
                /** @brief Helping state point for grasped configurations.*/
                sim::state_t* grasped_point; //manip and object state space
                /** @brief Helping state point for released configurations.*/
                sim::state_t* released_point; //manip state space
                /** @brief Helping state point for retracted configurations.*/
                sim::state_t* retracted_point; //manip state space

                //Helping variables for control points
                sim::control_t* manip_ctrl;
                /** @brief The control that will bring the manipulator to the safe state. Because the manipulator is a rigid body the safe_control = safe_state.*/
                sim::control_t* safe_control;

                //Helping vector variables.
                std::vector<double> manip_control_vec;
                std::vector<double*> stable_pose_memory;

                /** @brief Temporary retract path storage. */
                sim::trajectory_t retract_path;
                /** @brief Temporary retract plan storage. */
                sim::plan_t retract_plan;

                /** @brief Random manipulation sampling module. Samples both manipulator's state and object's state */
                manipulation::manip_sampler_t* manip_sampler;

                /** 
                 * @brief A validity checker that is aware of the objects and can detect important collisions. 
                 * Will be used by the Obstacle aware A*
                 */
                system_name_validity_checker_t* system_name_validity_checker;

                std::string transit_motion_planner_name;
                std::string transfer_motion_planner_name;
                std::string transit_manipulation_mp_name;
                std::string transfer_manipulation_mp_name;

                manipulation_mp_t* manip_mp;

                /** @brief The specification for this manipulation problem */
                rearrangement_manipulation_specification_t* specs;
                plan::motion_planning_specification_t* graph_specification;
                manipulation_mp_specification_t* manipulation_specification;

                //===============//
                //   For Poses   //
                //===============//       

                std::vector<manipulation::pose_t> poses_set;
                unsigned poses_set_length;

                /** @brief Fix poses read from the input.*/
                std::vector< std::vector< double > > fix_poses;                

                std::vector< std::pair<unsigned, sim::state_t*> > pose_seeds;
                
                std::string poses_file;
                
                //==============//
                // For Cheating //
                //==============//
                std::vector< std::vector< double > > cheating_poses;
                std::vector< sim::state_t*> cheating_transit_seeds;
                std::vector< sim::state_t*> cheating_transfer_seeds;

                //======================//
                // For de/serialization //
                //======================//   

                std::string transit_graph_file;
                std::string transfer_graph_file;
                std::string informed_transit_graph_file;
                std::string informed_transfer_graph_file;

                //================//
                // For Statistics //
                //================//

                /** @brief A clock to measure time for statistics.*/
                util::sys_clock_t statistics_clock;
                /** @brief The time for constructing a pebble graph.*/
                double rpg_graph_time;
                /** @brief Time that is needed in order to select the poses for the pebble graph.*/
                double select_poses_time;
                /** @brief Time that is needed to grasp the poses.*/
                double grasp_poses_time;
                /** @brief Time that is needed to connect the poses.*/
                double connect_poses_time;

                /** @brief A flag indicating whether PRM should send its computed graph to the visualization node. */
                bool visualize_graph;

                //Files serialization/deserialization variables.
                std::string prx_output_dir;   
                std::string prx_input_dir;

              private:
                util::config_t tmp_config;

                /**
                 * Detects the manipulator and one cup in the full state space.
                 * 
                 * @brief Detects the manipulator and one cup in the full state space.
                 */
                bool detect_plants();

                void compute_random_poses();

                void compute_posible_grasps(manipulation::pose_t& pose, int number_of_grasps, int max_tries);
                
                void compute_cheating_seeds();

                /**
                 * Returns a state point for the manipulator that grasps a movable object
                 * at the position pose_state.
                 * 
                 * @param point The grasped state for the manipulator that will be returned by this function.
                 * @param pose_state The state of the movable object that we want to grasp. 
                 * 
                 * @return True if the function returned a good grasped point, otherwise False. 
                 */
                bool get_grasp(sim::state_t* point, const sim::state_t* pose_state, int max_tries, bool full_motion = true);

                /**
                 * Manipulator steers to compute a plan and then propagate the plan 
                 * for collision checking. The correct planning context has to be set before
                 * call this function. The planning context must correspond to the planning context
                 * of the input variable start.
                 * 
                 * @param plan The plan that the function will return.
                 * @param path The path that the function will return.
                 * @param manip_start The initial position of the manipulator in manipulator_state_space.
                 * @param start The initial position of the execution of the plan.
                 * @param goal_config the configuration of the end effector that we desire to reach. 
                 * 
                 * @return True if the path is valid, otherwise False.
                 */
                bool valid_move(sim::plan_t& plan, sim::trajectory_t& path, const sim::state_t* manip_start, const sim::state_t* start, util::config_t& goal_config);

                /** Checks if the grasp is valid in the environment. 
                 * It also checks if the retract point and retract path are valid.
                 * In the process of validation of the grasp it also builds the retract path and retract point.
                 * 
                 * @brief Checks if the grasp is valid in the environment.
                 * 
                 * @param point Is the grasped point that it checks for validity. 
                 */
                bool is_valid_grasp(sim::state_t* point, bool full_motion);

                int similar_pose(sim::state_t* pose);
            };
        }
    }
}


#endif	
