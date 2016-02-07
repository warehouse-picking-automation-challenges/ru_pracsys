/**
 * @file naive_sparse.hpp
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

#ifndef PRX_SPARS2_HPP
#define	PRX_SPARS2_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx
{
    namespace packages
    {
        namespace spars
        {
        /**
         * @anchor naive_sparse_t
         *
         * Sparse Roadmap Spanner Method (SPARS2*):
         * A planning method which computes collision-free roadmaps, generally for any
         * system which has a BVP solver, using a sparse data structure.
         *
         * @brief <b> Sparse Roadmap Spanner Method (SPARS2*) </b>
         * 
         * @author Andrew Dobson
         */
        class naive_sparse_t : public plan::prm_star_t
        {

          public:
            naive_sparse_t();
            virtual ~naive_sparse_t();

            /**
             * @copydoc motion_planner_t::init(const util::parameter_reader_t*,const util::parameter_reader_t*) 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc motion_planner_t::reset() 
             */
            virtual void reset();

            /** 
             * @copydoc motion_planner_t::setup() 
             * 
             * Will occupy memory for the random_point and the new_control, after 
             * planning_query has been linked. 
             */
            virtual void setup();

            virtual bool execute();
            
          protected:

            //Members
            
            double mean_valence;
            double dev_valence;
            

            //Methods
            void connect_graph();

            util::undirected_vertex_index_t internal_add_node(const util::space_point_t* n_state);
            
            util::undirected_edge_index_t internal_add_edge(util::undirected_vertex_index_t u, util::undirected_vertex_index_t v);

            // ///PRM* members
            // //sim::trajectory_t path1;
            // //sim::trajectory_t path2;
            // //util::undirected_vertex_index_t v_new;
            // //util::undirected_graph_t graph;
            // //util::space_point_t* random_point;
            // //sim::plan_t new_plan;
            // //sim::plan_t new_plan2;
            // //unsigned int k;
            // //bool delta_prm;
            // //double r_n;
            // //prm_astar_t* astar;

            // //int num_edges;
            // //int num_vertices;
            // //int num_generated;
            // //double last_solution_length;
            // //bool pno_mode;

            // virtual void update_vis_info() const;
            // virtual void valid_random_sample();

            // virtual void connect_node(util::undirected_vertex_index_t v);
            // virtual void connect_node(util::undirected_vertex_index_t v, double rad);
            // virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);
            // virtual void update_k(unsigned nr_nodes);
            
            ///PRM* members
            //bool visualize_graph;
            //bool visualize_solutions;
            //std::string visualization_graph_name;
            //std::string visualization_solutions_name;
            //std::vector<std::string> visualization_bodies;
            //std::vector<std::string> solutions_colors;
            //std::string graph_color;
            
            
          private:
            ///PRM* members
            //double similarity_threshold;
            //bool remove_start;
            //std::vector<bool> remove_goals;

        };

        }
    }
}

#endif
