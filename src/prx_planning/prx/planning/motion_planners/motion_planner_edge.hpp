/**
 * @file motion_planner_edge.hpp
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
#ifndef PRX_MOTION_PLANNER_EDGE_HPP
#define	PRX_MOTION_PLANNER_EDGE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_edge.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * A base edge class for graph operations on a directed graph in a
         * motion planner.  Specifically includes a sim::plan_t and sim::trajectory_t on the edge.
         * 
         * @brief <b> Base edge class for graph operations on a directed graph in a motion planner. </b>
         * 
         * @author Andrew Kimmel
         * 
         */
        class motion_planner_edge_t : public util::undirected_edge_t
        {

          public:

            motion_planner_edge_t(){ }

            virtual ~motion_planner_edge_t(){ }

            /**
             * @brief Clear the edge, by clearing out its plan.
             *
             * @param space An unused space parameter. TODO : Remove this.
             */
            virtual void clear(const util::space_t* space)
            {
                plan.clear();
            }

            /**
             * @brief Initialize this edge with data.
             *
             * @param space The space in which this edge operates.
             * @param new_plan The plan which generated this edge.
             * @param new_path The trajectory that represent this edge.
             */
            virtual void init(const util::space_t* space, const sim::plan_t& new_plan, const sim::trajectory_t& new_path)
            {
                plan.link_control_space(space);
                plan = new_plan;                
                path = new_path;
            }

            /**
             * @brief Output edge information to a stream.
             *
             * @param output_stream The stream to which to serialize the node information.
             */
            virtual void serialize(std::ofstream& output_stream)
            {
                util::undirected_edge_t::serialize(output_stream);
                //        output_stream << " " << sampling_steps;

            }

            /**
             * @brief Read in edge information to a stream.
             *
             * @param input_stream The stream from which to read the node information.
             */
            virtual void deserialize(std::ifstream& input_stream)
            {
                util::undirected_edge_t::deserialize(input_stream);
                //        input_stream >> sampling_steps;
            }

            /** @brief The computed plan for this edge. */
            sim::plan_t plan;
            /** @brief The computed path for this edge. */
            sim::trajectory_t path;
            /** @brief A unique identifier for this edge. */
            int id;

        };

    }
}

#endif

