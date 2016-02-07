/**
 * @file rearrangement_manipulation_tp_t.hpp
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

#ifndef PRX_REARRANGEMENT_MANIPULATION_TP_HPP
#define PRX_REARRANGEMENT_MANIPULATION_TP_HPP

#include "planning/modules/pose.hpp"
#include "planning/graphs/pebble_graph.hpp"
#include "planning/graphs/super_graph.hpp"
#include "planning/modules/smoothing_info.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx/planning/task_planners/task_planner.hpp"

//From different packages
#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "../../../manipulation/simulation/plants/movable_body_plant.hpp"


#include <list>
#include <sstream>

namespace prx
{

    namespace util
    {
        class bounds_t;
        class goal_state_t;
        class statistics_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class manip_sampler_t;
        }

        namespace rearrangement_manipulation
        {
            using namespace baxter;
            using namespace manipulation;

            class manipulator_tp_t;
            class rearrangement_manipulation_specification_t;
            class manipulator_specification_t;
            class manipulation_mp_specification_t;
            class manipulator_query_t;
            class obstacle_aware_astar_t;
            class system_name_validity_checker_t;            

            struct grasping_info_t
            {

                pose_t* pose;
                sim::plan_t plan;
                std::set<unsigned> constraints;      
                std::set<unsigned> full_constraints;
                double distance;
                unsigned retracted_point;

                void clear()
                {
                    plan.clear();
                    constraints.clear();
                }

                std::string print() const
                {
                    std::stringstream output(std::stringstream::out);
                    
                    output << "|plan|=" << plan.size() << "   cost=" << distance << "   |c|=" << constraints.size() << "    :";

                    foreach(unsigned i, constraints)
                    {
                        output << i << " , ";
                    }
                    output<< "       full_constraints: ";
                    foreach(unsigned i, full_constraints)
                    {
                        output << i << " , ";
                    }
                    return output.str();
                }
            };

            /**
             * Manipulation task planner. Computes the path for moving an object from an 
             * initial to a target position.             
             * 
             * @autors Athanasios Krontiris
             */
            class rearrangement_manipulation_tp_t : public plan::task_planner_t
            {

              public:

                rearrangement_manipulation_tp_t();
                virtual ~rearrangement_manipulation_tp_t();

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
                 * @copydoc planner_t::update_vis_info() const
                 */
                virtual void update_vis_info() const;

                //=============================//
                // Functions For Pebble Graphs //
                //=============================//                
                virtual util::undirected_graph_t* create_MPG(std::vector<pebble_edge_t*>& constrained_edges, const std::vector< unsigned >& seed_positions = std::vector< unsigned >(), bool is_fake = false);
                virtual bool valid_pose_in_MPG(util::undirected_graph_t* graph, int index);
                virtual bool pebble_graph_exists(const std::set<unsigned> poses);
                virtual bool find_best_path(unsigned start_index, unsigned target_index, std::set<unsigned>& constraints, std::set<unsigned>& full_constraints, double& edge_cost, int& from, int& to, int& middle);

                //============================//
                // Functions For Super Graphs //
                //============================//                
                virtual bool create_super_graph();
                virtual bool expand(std::vector< unsigned >& seed_positions);
                virtual void generate_nodes(std::vector< const super_node_t* >& new_nodes, std::vector< std::vector< unsigned > >& sig_map, util::undirected_graph_t * p_graph);
                virtual void connect_supernode(const super_node_t * hnode);
                virtual void connect_siblings(const std::vector< const super_node_t* >& new_nodes, std::vector< pebble_edge_t* >& edges);
                virtual void query_super_graph();
                
                virtual void compose_final_plan();
                virtual void smoothing();
                /** This function will bring all the grasps of the same object closer to each other. (part1: bring together) */
                virtual bool smoothing_part1();
                /** This function will combine the consecutive grasps of the same object to a common path. (part2: combine)*/
                virtual bool smoothing_part2();
                /** This function will apply the basic smoothing on the path. (part3: basic smoothing)*/
                virtual void smoothing_part3();
                virtual bool validate_full_path();
                
                virtual void detect_object_positions(std::set<unsigned> &poses, unsigned id, const std::list<smoothing_info_t*>::iterator& it);
                

                virtual bool connection_arrangment(std::set< unsigned >& arrangement, const super_node_t* node, const super_node_t* other);
                virtual bool can_assign(const super_node_t* node, const std::set< unsigned >& positions) const;
                virtual void get_from_to_components(const super_node_t* node, pebble_edge_t* edge, unsigned& component_from, unsigned& component_to);
                virtual bool is_feasible_edge(const hnode_data_t* node_data, pebble_edge_t* edge, unsigned component_to);
                virtual const super_node_t * find_sibling_with_signature(const std::vector<const super_node_t*>& new_nodes, const std::vector<unsigned>& signature);
                virtual super_node_t * random_selection(const util::undirected_graph_t & graph);
                virtual util::undirected_vertex_index_t is_hnode_generated(const hnode_data_t* new_node) const;

                /** Basis case: arrangements are the same?  Return empty path. 
                 * Otherwise
                 * Run Auletta's algorithm over the problem (node, start, goal)
                 * For each step in the returned
                 */
                void solve_path(const super_node_t& node, std::vector< unsigned >& start_arrangement, const std::vector< unsigned > goal_arrangement);
                virtual void write_statistics();

                manipulator_plant_t* _manipulator;
                movable_body_plant_t* _object;
                movable_body_plant_t* collision_object;

                /** @brief Reads from input the name of the planning context that is for the manipulator only.*/
                std::string pc_name_manipulator_only;
                /** @brief Reads from input the name of the planning context that is for the real world objects.*/
                std::string pc_name_real_world;
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
                std::string pc_name_all_objects;
                /** @brief Reads from input the name of the planning context that is for the manipulator and the 
                 * object while all the other objects are active.*/
                std::string pc_name_grasp_planning;
                /** @brief Reads from input the name of the planning context that is for collision checks between poses in MPG*/
                std::string pc_name_collision_check;

                /** @brief The state space over the manipulator that the manipulation task planner is using. */
                const util::space_t* manip_state_space;
                /** @brief The control space over the manipulator that the manipulation task planner is using. */
                const util::space_t* manip_control_space;
                /** @brief The state space over the objects that the task planner is using*/
                const util::space_t* object_state_space;
                /** @brief The state space over the manipulator and the object that the task planner is using*/
                const util::space_t* mo_space;
                /** @brief The state space for the real objects*/
                const util::space_t* all_object_space;
                /** @brief The state space for all the other cup except the one that we control*/
                const util::space_t* other_objects_space;
                /** @brief The state space for the object that will be used to check collisions between poses in an MPG*/
                const util::space_t* collision_object_space;
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
                /** @brief Helping variable for checking for collisions between poses during the construction of an MPG*/
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

                /** @brief The specification for this manipulation problem */
                rearrangement_manipulation_specification_t* specs;

                /** @brief Random manipulation sampling module. Samples both manipulator's state and object's state */
                manip_sampler_t* manip_sampler;

                /** 
                 * @brief A validity checker that is aware of the objects and can detect important collisions. 
                 * Will be used by the Obstacle aware A*
                 */
                system_name_validity_checker_t* system_name_validity_checker;

                std::string manipulation_tp_name;

                manipulator_tp_t* manip_tp;
                manipulator_query_t* manip_query;
                manipulator_specification_t* manip_specs;

                //======================//
                // For De/Serialization //
                //======================//

                //Files serialization/deserialization variables.
                std::string prx_output_dir;
                std::string prx_input_dir;

                //===============//
                //   For Poses   //
                //===============//

                std::string poses_file;

                std::vector<pose_t> poses_set;
                unsigned poses_set_length;

                /** @brief All the poses that will inform the graph. Id for the pose and the state*/
                std::vector< std::pair<unsigned, sim::state_t*> > seed_poses;
                /** @brief The new poses that correspond to the initial and final positions.*/
                std::vector< std::pair<unsigned, sim::state_t*> > query_poses;

                util::hash_t<unsigned, std::vector< grasping_info_t* > > grasping_poses;

                //==================================//
                // For Manipulation Motion Planners //
                //==================================//

                /** @brief An A* module that computes minimum conflict paths using tree search A* */
                obstacle_aware_astar_t* transit_astar;
                obstacle_aware_astar_t* transfer_astar;

                std::set<unsigned>* transit_constraints;
                std::set<unsigned>* transfer_constraints;

                manipulation_mp_specification_t* transit_specification;
                manipulation_mp_specification_t* transfer_specification;

                //===================//
                // For Pebble Graphs //
                //===================//

                std::vector<unsigned> initial_poses_ids;
                std::vector<unsigned> target_poses_ids;
                std::vector<bool> pose_checked;
                unsigned mpg_size;
                std::vector<util::undirected_vertex_index_t> mpg_vertices;


                //==================//
                // For Super Graphs //
                //==================//
                util::undirected_graph_t super_graph;
                util::undirected_vertex_index_t v_initial;
                util::undirected_vertex_index_t v_target;

                std::vector<hnode_data_t*> pebble_graph_storage;
                std::vector< util::undirected_vertex_index_t > generated_indices;
                bool initial_biasing;
                bool found_final_arrangement;
                double time_limit;

                //===============//
                // For Smoothing //
                //===============//
                bool validate_solution;
                util::hash_t<unsigned, int> pose_has_object;
                std::list<smoothing_info_t *> plan_parts;        
                unsigned *curr_poses;

                //================//
                // For Statistics //
                //================//

                /** @brief Number of Connected Components in the MPG_graphs*/
                unsigned P_cc;
                /** @brief Number of edges in the MPG_graphs*/
                unsigned P_e;
                /** @brief Counts the number of the super edges in the Super-graph*/
                unsigned no_super_edges;
                /** @brief Counts the number of the edges between siblings in the Super-graph*/
                unsigned no_sibling_edges;

                /** @brief A clock to measure time for statistics.*/
                util::sys_clock_t statistics_clock;
                /** @brief The time for constructing a pebble graph.*/
                double mpg_graph_time;
                /** @brief Time that is needed in order to select the poses for the pebble graph.*/
                double select_poses_time;
                /** @brief Time that is needed to grasp the poses.*/
                double grasp_poses_time;
                /** @brief Time that is needed to connect the poses.*/
                double connect_poses_time;

                /** @brief If we want to gather statistics or not*/
                bool gather_statistics;

                /** @brief A flag indicating whether PRM should send its computed graph to the visualization node. */
                bool visualize_graph;

              private:
                util::config_t tmp_config;

                /**
                 * Detects the manipulator and one cup in the full state space.
                 * 
                 * @brief Detects the manipulator and one cup in the full state space.
                 */
                bool detect_plants();

                void compute_random_poses(unsigned size);

                void compute_posible_grasps(pose_t& pose, int number_of_grasps, int max_tries);

                /**
                 * Returns a state point for the manipulator that grasps a movable object
                 * at the position pose_state.
                 * 
                 * @param point The grasped state for the manipulator that will be returned by this function.
                 * @param pose_state The state of the movable object that we want to grasp. 
                 * 
                 * @return True if the function returned a good grasped point, otherwise False. 
                 */
                bool get_grasp(sim::state_t* point, const sim::state_t* pose_state, int max_tries);

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
                bool is_valid_grasp(sim::state_t* point);

                int similar_pose(sim::state_t* pose);


                std::string print(const std::vector<unsigned>& arrangement);
                std::string print(const std::set<unsigned>& constratins);
            };
        }
    }
}


#endif	
