/**
 * @file tf_broadcaster.hpp 
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

#ifndef PRX_TF_BROADCASTER_HPP
#define	PRX_TF_BROADCASTER_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <tf/transform_broadcaster.h>
#include <vector>

namespace prx
{
    namespace util
    {

        class config_t;

        /**
         * This class is used to broadcast configurations through ros::tf.  These
         * transforms are used to move the geometries visualized in visualization.
         * 
         * @brief <b> Broadcasts tf::Transforms </b>
         *
         * @authors James Marble, Andrew Kimmel
         */
        class tf_broadcaster_t
        {

          public:
            /**
             * Queues a configuration to be broadcasted through tf
             * 
             * @brief Queues a configuration to be broadcasted through tf
             * @param config The configuration that will be queued for broadcasting
             * @param name The pathname used to index the configuration
             */
            void queue_config(const config_t& config, const std::string& name);

            /**
             * Broadcasts all queued configurations, and then clears the queue
             * 
             * @brief Broadcasts all queued configurations, and then clears the queue
             */
            void broadcast_configs();

          private:

            /** @brief the tf broadcaster */
            tf::TransformBroadcaster br;

            /** @brief The queued configurations as transforms */
            std::vector<tf::StampedTransform> transforms;
        };

    }
}

#endif

