/**
 * @file distance_function.hpp 
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

#ifndef PRX_DISTANCE_FUNCTION_HPP
#define	PRX_DISTANCE_FUNCTION_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <boost/function.hpp>
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {

        class space_t;
        class space_point_t;

        typedef boost::function<double (const space_point_t*, const space_point_t*)> distance_t;

        /**
         * A class containing a possible distance function for points.
         * @brief <b> A class containing a possible distance function for points. </b>
         * @author Zakary Littlefield 
         */
        class distance_function_t
        {

          public:
            distance_function_t();
            virtual ~distance_function_t();

            void link_space(const space_t* space);

            virtual double distance(const space_point_t* s1, const space_point_t* s2) = 0;

            /**
             * Used for pluginlib operations. Looks at the plugin xml file for defined classes of this type.
             * @brief Used for pluginlib operations.
             * @return  The class loader from pluginlib.
             */
            static pluginlib::ClassLoader<distance_function_t>& get_loader();


            /**
             * @brief The function to give to the classes using this.
             */
            distance_t dist;

          protected:
            const space_t* ref_space;

          private:

            /**
             * @brief The class loader from pluginlib.
             */
            static pluginlib::ClassLoader<distance_function_t> loader;

        };

    }
}

#endif 