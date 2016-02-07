/**
 * @file ground_truth_query_application.hpp 
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

#include "prx/planning/applications/planning_application.hpp"
#include "prx/planning/queries/query.hpp"

#ifndef PRX_GROUND_TRUTH_QUERY_APPLICATION_HPP
#define	PRX_GROUND_TRUTH_QUERY_APPLICATION_HPP

namespace prx
{
    namespace util 
    {
        class linear_distance_metric_t;
    }
    
    namespace plan
    {

        

        /**
         * @anchor ground_truth_query_application_t
         *
         * Some times, a consumer controller will need to query for a motion plan 
         * multiple times, such as in a replanning scenario.  This application handles
         * asynchronous requests for motion plans and begins providing the requester
         * with motion plans.
         *
         * @brief <b> Planning application which processes queries as they come. </b>
         *
         * @author Andrew Kimmel
         */
        class ground_truth_query_application_t : public planning_application_t
        {

          public:

            ground_truth_query_application_t();
            virtual ~ground_truth_query_application_t();

            /**
             * @copydoc planning_application_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader);

            /**
             * @copydoc planning_applicaiton_t::execute()
             *
             * The ground truth query application will process queries as they come.
             */
            virtual void execute();

            /**
             * @brief Query request processing function.
             *
             * When the simulation node requests for resolution of a query, this callback
             * takes the necessary steps to do so.
             *
             * @param msg The message containing the query to process.
             */
            virtual void process_query_callback(const prx_simulation::query_msg& msg);
            /**
             * @brief Ground truth update callback.
             *
             * The ground truth simulation node and the planning node may come to have 
             * different state information for a given time.  This callback processes a
             * ground truth message which updates the planning-side state information.
             *
             * @param msg The message containing the ground-truth state information.
             */
            virtual void process_ground_truth_callback(const prx_simulation::state_msg& msg);

          private:
            /** @brief The current query being processed by this application. */
            query_t* the_query;
            /** @brief Distance metric used to determine distances.  Helpful for determining goal achievement. */
            util::linear_distance_metric_t* goal_metric;

            // TODO : Move all of these statistics things to an actual statistics class!    
            /** @brief  */
            double avg_construction_time;
            /** @brief  */
            double avg_planning_time;
            /** @brief  */
            double avg_resolve_time;
            /** @brief  */
            double avg_visualize_time;
            /** @brief */
            double avg_number_nodes;
            /** @brief  */
            double avg_time;
            /** @brief  */
            double avg_steps;

            /** @brief */
            double total_construction_time;
            /** @brief  */
            double total_planning_time;
            /** @brief  */
            double total_resolve_time;
            /** @brief  */
            double total_visualize_time;
            /** @brief */
            double total_nodes;
            /** @brief  */
            double total_time;
            /** @brief  */
            double total_steps;

            /** @brief */
            int num_queries;
        };

    }
}

#endif

