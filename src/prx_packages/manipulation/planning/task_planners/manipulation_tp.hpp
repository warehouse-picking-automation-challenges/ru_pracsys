/**
 * @file manipulation_tp_t.hpp
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

#ifndef PRX_MANIPULATION_TP_HPP
#define	PRX_MANIPULATION_TP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_graph.hpp"
#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "simulation/plants/movable_body_plant.hpp"

#include "planning/modules/pose.hpp"

namespace prx
{

    namespace util
    {
        class bounds_t;
        class multiple_goal_states_t;
        class statistics_t;

    }

    namespace plan
    {
        class motion_planning_specification_t;
        class motion_planning_query_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            using namespace baxter;
            class manip_sampler_t;
            class manipulation_specification_t;
            class manipulation_query_t;

            /**
             * Manipulation task planner. Computes the path for moving an object from an
             * initial to a target position.
             *
             * @autors Athanasios Krontiris
             */
            class manipulation_tp_t : public plan::task_planner_t
            {

              public:

                manipulation_tp_t();
                virtual ~manipulation_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc motion_planner_t::reset()
                 */
                virtual void reset();

                /**
                 * @copydoc motion_planner_t::link_world_model()
                 */
                virtual void link_world_model(plan::world_model_t * const model);

                /**
                 * @copydoc motion_planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();

                /**
                 * @copydoc planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* new_spec);

                /**
                 * @copydoc motion_planner_t::link_query()
                 */
                virtual void link_query(plan::query_t* new_query);

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

                /**
                 * @copydoc planner_t::set_param( const std::string&, const boost::any& )
                 */
                virtual void set_param(const std::string& parameter_name, const boost::any& value);

                /**
                 * @copydoc planner_t::update_vis_info() const
                 */
                virtual void update_vis_info() const;


                std::string manipulator_path;
                manipulator_plant_t* _manipulator;
                movable_body_plant_t* _object;

                /** @brief Reads from input the name of the planning context that is for the manipulator only.*/
                std::string pc_name_manipulator_only;
                /** @brief Reads from input the name of the planning context that is for the object.*/
                std::string pc_name_object_only;
                /** @brief Reads from input the name of the planning context that is for the manipulator and the object.*/
                std::string pc_name_manipulator_with_object;
                /** @brief Reads from input the name of the planning context that is for the manipulator and an active for collision object.*/
                std::string pc_name_manipulator_with_active_object;
                /** @brief The planning context that is for the transit state (The manipulator and active object).*/
                std::string pc_name_transit_inform;
                /** @brief The planning context that is for the transfer state (The manipulator with plannable object and one more active object.*/
                std::string pc_name_transfer_inform;
                /** @brief Reads from input the name of the planning context that is for the real world.*/
                std::string pc_name_transit_planning;
                /** @brief Reads from input the name of the planning context that is for the manipulator and the
                 * object while all the other objects are active.*/
                std::string pc_name_transfer_planning;

                /** @brief The state space over the manipulator that the manipulation task planner is using. */
                const util::space_t* manip_state_space;
                /** @brief The control space over the manipulator that the manipulation task planner is using. */
                const util::space_t* manip_control_space;
                /** @brief The state space over the objects that the task planner is using*/
                const util::space_t* object_state_space;
                /** @brief The state space over the manipulator and the object that the task planner is using*/
                const util::space_t* mo_space;
                /** @brief The state space for the real objects*/
                const util::space_t* real_object_space;
                /** @brief The state space for all the other cup except the one that we control*/
                const util::space_t* other_objects_space;
                /** @brief The space for sampling random poses.*/
                util::space_t* stable_pose_space;


                //Helping variables for state points.
                sim::state_t* manip_state;
                sim::state_t* object_state;
                /** @brief The safe state for the manipulator.*/
                sim::state_t* safe_state;
                /** @brief Real object initial state point.*/
                sim::state_t* real_initial_state;
                /** @brief The random sampled state for a pose.*/
                sim::state_t* stable_pose_state;
                /** @brief Helping variable for storing the initial pose of the object that we want to grasp */
                sim::state_t* initial_pose;
                /** @brief Helping variable for storing the target pose of the object that we want to transfer */
                sim::state_t* goal_pose;

                //Temporary storages for random samples.
                /** @brief Helping state point for grasped configurations.*/
                sim::state_t* grasped_point; //manip and object state space
                /** @brief Helping state point for released configurations.*/
                sim::state_t* released_point; //manip state space

                //Helping variables for control points
                sim::control_t* manip_ctrl;
                /** @brief The control that will bring the manipulator to the safe state. Because the manipulator is a rigid body the safe_control = safe_state.*/
                sim::control_t* safe_control;

                //Helping vector variables.
                std::vector<double> manip_control_vec;
                std::vector<double> mo_control_vec;
                std::vector<double*> stable_pose_memory;

                /** @brief Temporary retract path storage. */
                sim::trajectory_t retract_path;
                /** @brief Temporary retract plan storage. */
                sim::plan_t retract_plan;

                /** @brief Random manipulation sampling module. Samples both manipulator's state and object's state */
                manip_sampler_t* manip_sampler;

                /** @brief The specification for this manipulation problem */
                manipulation_specification_t* specs;

                /** @brief The query for this manipulation problem */
                manipulation_query_t* manip_query;

                std::string ungrasped_name;
                std::string grasped_name;

                std::string reach_phase;
                std::string transfer_phase;
                std::string retract_phase;

                plan::motion_planning_query_t* reach_query;
                plan::motion_planning_query_t* transfer_query;
                plan::motion_planning_query_t* retract_query;

                util::multiple_goal_states_t* reach_goal;
                util::multiple_goal_states_t* transfer_goal;

                /** @brief A flag indicating whether PRM should send its computed graph to the visualization node. */
                bool visualize_graph;

                /** @brief Helping variable for queries.*/
                plan::motion_planning_query_t* graph_query;

                /** @brief The specification for the motion planners*/
                plan::motion_planning_specification_t* graph_specification;

                //Files serialization/deserialization variables.
                std::string prx_dir;
                std::string ungrasped_graph_file_name;
                std::string grasped_graph_file_name;
                bool serialize_grasped_graph;
                bool serialize_ungrasped_graph;
                bool deserialize_grasped_graph;
                bool deserialize_ungrasped_graph;
                bool graph_builder;

                util::sys_clock_t _clock;
                double statistics_time;

                util::config_t retraction_config;
                util::config_t tmp_config;

                /**
                 * Detects the manipulator and one cup in the full state space.
                 *
                 * @brief Detects the manipulator and one cup in the full state space.
                 */
                virtual void detect_plants();

                virtual void compute_posible_grasps(pose_t& pose, int number_of_grasps, int max_tries);

                /**
                 * Returns a state point for the manipulator that grasps a movable object
                 * at the position pose_state.
                 *
                 * @param point The grasped state for the manipulator that will be returned by this function.
                 * @param pose_state The state of the movable object that we want to grasp.
                 *
                 * @return True if the function returned a good grasped point, otherwise False.
                 */
                virtual bool get_grasp(sim::state_t* point, const sim::state_t* pose_state, int max_tries);

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
                virtual bool valid_move(sim::plan_t& plan, sim::trajectory_t& path, const sim::state_t* manip_start, const sim::state_t* start, util::config_t& goal_config);

                /** Checks if the grasp is valid in the environment.
                 * It also checks if the retract point and retract path are valid.
                 * In the process of validation of the grasp it also builds the retract path and retract point.
                 *
                 * @brief Checks if the grasp is valid in the environment.
                 *
                 * @param point Is the grasped point that it checks for validity.
                 */
                virtual bool is_valid_grasp(sim::state_t* point);

                virtual bool reach(pose_t& pose, plan::query_t* query, int grasp_index = 0);
                virtual bool retract(pose_t& pose, plan::query_t* query, int grasp_index = 0);
                virtual bool tranfer(pose_t& start_pose, pose_t& target_pose, plan::query_t* query, int start_grasp_index = 0, int target_grasp_index = 0);
                virtual bool manipulate(pose_t& start_pose, pose_t& target_pose);

                virtual void update_query_information(plan::query_t* query, int index = -1);
                virtual void combine_queries(unsigned& min_path_size);
                virtual void add_null_path();

                virtual bool validate();
            };
        }
    }
}


#endif
