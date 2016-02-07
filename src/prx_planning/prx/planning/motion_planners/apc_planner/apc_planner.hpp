/**
 * @file prm_star.hpp
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
#pragma once

#ifndef PRX_APC_PLANNER_HPP
#define	PRX_APC_PLANNER_HPP

#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
#include "prx/planning/modules/local_planners/apc_local_planner.hpp"
#include "prx/planning/modules/samplers/apc_sampler.hpp"
#include "prx/planning/modules/IK_data_base/IK_data_base.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/world_model.hpp"

#include "simulation/plants/manipulator.hpp"

#include "simulation/manipulator_simulator.hpp"

namespace prx
{
    namespace plan
    {

        //class pno_criterion_t;
        //class astar_module_t;
        
       /**
         * @anchor apc_planner_t
         *
         * The APC planner which extend the Probabilistic Roadmap Method (PRM*):
         *
         * @brief <b> APC Planner </b>
         * 
         * @author Athanasios Krontiris, Andrew Dobson, Kostas Bekris
         */
        class apc_planner_t : public prm_star_t
        {

          public:
            apc_planner_t();
            virtual ~apc_planner_t();

            /**
             * @copydoc motion_planner_t::init(const util::parameter_reader_t*,const util::parameter_reader_t*) 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc motion_planner_t::reset() 
             */
            virtual void reset();

            /**
             * @copydoc motion_planner_t::link_specification(specification_t*) 
             */
            void link_specification(specification_t* new_spec);
            
            /** 
             * @copydoc motion_planner_t::setup() 
             * 
             * Will occupy memory for the random_point and the new_control, after 
             * planning_query has been linked. 
             */
            virtual void setup();
            void setup_online_mode();
            void restart_astar();
            
            /**
             * @copydoc planner_t::execute()
             */
            virtual bool execute();

            /** 
             * @copydoc motion_planner_t::step() 
             * 
             * The algorithm will add one new node for every call of this function.
             */
            virtual void step();

            /** 
             * @copydoc motion_planner_t::resolve_query() 
             *
             * At the end of the resolve_query the algorithm will remove the vertices 
             * for the start and the goal for this specific query from the graph.
             */
            virtual void resolve_query();

            void resolve_query( util::space_t* object_space, world_model_t* model, bool grasp );
            bool IK_steer_resolution( world_model_t* model, util::config_t end_config, bool grasping, bool camera_link );

            /**
             * @copydoc motion_planner_t::succeeded() const 
             */
            virtual bool succeeded() const;

            /**
             * @copydoc motion_planner_t::serialize() const 
             */
            virtual bool serialize();

            /**
             * @copydoc motion_planner_t::deserialize() const 
             */
            virtual bool deserialize();
            
            void link_manipulator(prx::packages::baxter::manipulator_plant_t* manipulator, bool is_left_arm);

            void validate_precomputation( );

            void print_graph( std::ostream& stream );
            
            /**
             * Generate the IK database from the loaded roadmap.
             * 
             * @param The data base that the motion planner will fill up with the new 
             * points 
             */
            void generate_IK_data_base(IK_data_base_t& IK_base, IK_data_base_t& camIK_base);
            
            virtual int get_number_of_roadmap_vertices()
            {
                return boost::num_vertices(graph.graph);
            }
            
            virtual int get_number_of_roadmap_edges()
            {
                return boost::num_edges(graph.graph);
            }

            virtual void change_grasping_state();

            /**
             * @copydoc motion_planner_t::get_statistics() const 
             */
            virtual const util::statistics_t* get_statistics();

            bool validate_roadmap( world_model_t* model );

            friend class pno_criterion_t;
            
          protected:

            /**
             * @copydoc planner_t::update_vis_info() const
             */
            virtual void update_vis_info() const;

            /**
             * @brief Generates a collision-free random sample and stores it in random_point.
             */
            virtual void valid_random_sample();
            
            /**
             * Generates a collision-free random sample and stores it in random_point. The random generated point
             * has the end-effector within the min and max bounds (x,y,z)
             * 
             * @param min_bounds The minimum bounds (x,y,z) for the end-effector
             * @param max_bounds The maximum bounds (x,y,z) for the end-effector
             */
            void valid_random_inside_sample(const util::vector_t& min_bounds, const util::vector_t& max_bounds, bool fix_gripper = true);

            /**
             * Copying over from current state the appropriate variables
             *
             * @brief Copying over from current state the appropriate variables
             *
             * @param state The state that we want to copy over the default values for the appropriate hand.
             */
            void impose_hand_state( sim::state_t* target_state, bool fix_gripper = true );

            /**
             * Adds a new node in the graph and trying to connect it with the 
             * existing graph. 
             * 
             * @brief Add a new node to the graph.
             *
             * @param n_state The new state that I want to add in the graph.
             * 
             * @return The index for the node in the boost::graph.
             */
            virtual std::pair<bool,util::undirected_vertex_index_t> add_node(const util::space_point_t* n_state);

            /**
             * @brief Connects the node v on the existing graph.
             * 
             * @param v Its the index of an existing node on the graph.
             */
            virtual void connect_node(util::undirected_vertex_index_t v);

            /**
             * @brief Connects the node v in a neighbor of radian rad on the existing graph.
             * 
             * @param v Its the index of an existing node on the graph.
             * @param rad The diameter of the neighbor that we need to check in order to connect the new node on the graph.
             */
            virtual void connect_node(util::undirected_vertex_index_t v, double rad);

            /**
             * @brief Tries to link the node v with the neighbor nodes in the vector neighbors.
             * 
             * @param v Its the index of an existing node on the graph, that we want to connect on the graph.
             * @param neighbors The nodes that we will try to connect with the node v.
             */
            virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);

            /**
             * @brief Checks if the trajectory on the edge is valid or not. 
             * 
             * Checks if the trajectory on the edge is valid or not. 
             * 
             * @param path The trajectory that has to be checked.
             */
            bool is_valid_trajectory(const sim::trajectory_t& path);
            
             /**
             * @copydoc planner_t::set_param( const std::string&, const boost::any& )
             */
            virtual void set_param(const std::string& parameter_name, const boost::any& value);
            
          private:
            sim::state_t* default_state;
            bool is_left_arm;

            util::space_point_t* final_state;

            //Keeps the information that a node is outside the shelf.
            bool trusted_node;
            int global_run_id; 
            int inside_samples;
            
            unsigned ik_failures;
            
            //When we are online we don't have to check for safe edges and nodes. 
            //They are safe. 
            bool online_version;
            
            double collision_check_time;
            double safe_check_time;
            int check_count;            
            
            prx::packages::baxter::manipulator_plant_t* _manipulator;
            apc_sampler_t* apc_sampler;
            apc_local_planner_t* apc_local_planner;
            
            double shelf_x, shelf_y, shelf_z;
            std::vector<double> col_widths;
            std::vector<double> row_heights;
            double shelfs_lip_height,support_column_width;

            //Every how many steps we will select a point for the trajector to the IK_data_base.
            int trajectory_subsampling;

            world_model_t* the_model;
        };

    }
}

#endif
