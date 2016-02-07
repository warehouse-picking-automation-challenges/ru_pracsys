/**
 * @file tf_listener.hpp 
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

#ifndef PRX_TF_LISTENER_HPP
#define	PRX_TF_LISTENER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include <tf/transform_listener.h>

namespace prx
{
    namespace util
    {

        class config_t;

        /**
         * This class is used to access transforms stored in tf and create configurations
         * from these transforms for use in other classes.
         * 
         * @brief <b> The ros::tf listener which receives broadcasted transforms </b>
         *
         * @authors James Marble, Andrew Kimmel
         */

        class tf_listener_t
        {

          public:

            /**
             * Performs a global transformation lookup and update the given config
             * 
             * @brief Performs a global transformation lookup and update the given config
             * @param name The pathname to lookup 
             * @param config The configuration that will be updated
             */
            void lookup(const std::string& name, config_t& config) const;

          private:
            /** @brief The ros::tf listener */
            tf::TransformListener tf_listener;

        };

    }
}

#endif

