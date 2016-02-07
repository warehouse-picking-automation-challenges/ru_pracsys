/**
 * @file cost_function.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_COST_FUNCTION_HPP
#define	PRX_COST_FUNCTION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

#include <boost/function.hpp>
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace sim
    {

        typedef boost::function<double (const util::space_point_t*)> state_cost_t;
        typedef boost::function<double (const util::space_point_t*,const util::space_point_t*)> heuristic_cost_t;
        typedef boost::function<double (const trajectory_t&)> trajectory_cost_t;

        /**
         * A class containing a possible cost function for points and trajectories.
         * @brief <b> A class containing a possible cost function for points and trajectories. </b>
         * @author Zakary Littlefield 
         */
        class cost_function_t
        {

          public:
            cost_function_t();
            virtual ~cost_function_t();

            virtual double state_cost(const util::space_point_t* s) = 0;

            virtual double heuristic_cost(const util::space_point_t* s,const util::space_point_t* t) = 0;

            virtual double trajectory_cost(const trajectory_t& t) = 0;

            void link_distance_function(util::distance_t in_dist);

            /**
             * Used for pluginlib operations. Looks at the plugin xml file for defined classes of this type.
             * @brief Used for pluginlib operations.
             * @return  The class loader from pluginlib.
             */
            static pluginlib::ClassLoader<cost_function_t>& get_loader();

            /**
             * @brief The function exposed for costs of a state.
             */
            state_cost_t cost_of_state;

            /**
             * @brief The function exposed for costs of a trajectory.
             */
            trajectory_cost_t cost_of_trajectory;

            /**
             * @brief The function exposed for costs of a trajectory.
             */
            heuristic_cost_t heuristic;

          protected:

            /**
             * @brief The function used to determine costs in some implementations.
             */
            util::distance_t dist;

          private:

            /**
             * @brief The class loader from pluginlib.
             */
            static pluginlib::ClassLoader<cost_function_t> loader;

        };

    }
}

#endif 