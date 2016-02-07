/**
 * @file goal.hpp
 *  * 
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

#ifndef PRX_GOAL_HPP
#define	PRX_GOAL_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /** * 
         * Hold all the information for any abstract version of goal. Concrete implementations of the 
         * class goal deal with different versions of goal. 
         * @brief <b> Hold all the information for any abstract version of goal. </b>
         * 
         * @author Athanasios Krontiris
         */
        class goal_t
        {

          public:
            goal_t();
            virtual ~goal_t();

            /** 
             * Initialize the goal from the given parameters.
             * @brief Initialize the goal from the given parameters.
             *
             * @param reader A \ref parameter_reader_t with a dictionary of parameters for this goal.
             * @param template_reader A \ref parameter_reader_t with a dictionary of parameters for the template of this goal.
             */
            virtual void init(const parameter_reader_t* reader, const parameter_reader_t* template_reader = NULL);
            
            /**
             * Clears the goal states. This way we can reuse the same goal_t for a different search. 
             * @brief Clears the goal states.
             */
            virtual void clear();

            /**
             * Initialize the goal from the given parameters.
             * @brief Links a space to the goal so the internal distance_metric will have a linked space.
             * 
             * @param inspace The space that the goal is working on.
             */
            virtual void link_space(const space_t* inspace);

            /**
             * Link in a new metric for the goal to use.
             * @brief Link a distance metric.
             * 
             * @param inmetric The metric object to link to this goal.
             */
            virtual void link_metric(distance_metric_t* inmetric);

            /**
             * Checks if the goal is satisfied. 
             * @brief Checks if the goal is satisfied. 
             * 
             * @param state The state to check that might satisfy the goal.
             * @return Whether the goal is satified or not.
             */
            virtual bool satisfied(const space_point_t* state) = 0;

            /**
             * Checks if the goal is satisfied. 
             * @brief Checks if the goal is satisfied. 
             * 
             * @param state The state to check that might satisfy the goal.
             * @param distance The distance between the goal criterion and the state that we 
             *        passed as argument.
             * @return Whether the goal is satified or not.
             */
            virtual bool satisfied(const space_point_t* state, double* distance) = 0;

            /**
             * Returns a vector of goal points that will satisfy the goal.
             * @brief Returns a vector of goal points that will satisfy the goal.
             * 
             * @return A vector with goal points. 
             */
            const std::vector<space_point_t*>& get_goal_points() const;

            /**
             * Get the pluginlib loader for this base class.
             * @brief Get the pluginlib loader for this base class.
             * @return The pluginlib loader.
             */
            static pluginlib::ClassLoader<goal_t>& get_loader();

          protected:
            /**
             * @brief The space for goal points.
             */
            const space_t* space;

            /**
             * @brief The list of goal points.
             */
            std::vector<space_point_t*> goal_points;

            /**
             * @brief Distance metric for finding the distances between states.
             */
            distance_metric_t* distance_metric;


          private:
            /**
             * @brief The pluginlib class loader.
             */
            static pluginlib::ClassLoader<goal_t> loader;
        };

    }
}

#endif	
