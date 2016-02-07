/**
 * @file query.hpp
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

#ifndef PRX_QUERY_HPP
#define	PRX_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace plan
    {

        /**
         * @anchor query_t
         *
         * The abstract query class represents some query that needs to be answered.
         * Queries are used by motion planners to define start/goal pairs, and by task
         * planners for high-level objectives.
         *
         * @brief <b> Abstract query class. </b>
         */
        class query_t
        {

          public:

            query_t(){ }

            virtual ~query_t(){ }

            /**
             * @brief Initialize the query from input parameters.
             *
             * @param reader The reader for the parameters.
             * @param template_reader A template reader for reading template parameters 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @brief Retrieve the pluginlib class loader for queries.
             *
             * @return The pluginlib class loader for queries.
             */
            static pluginlib::ClassLoader<query_t>& get_loader();

            /** @brief The name of the planner this query is supplied to. */
            std::string planner_name;


          private:
            /** @brief The query pluginlib class loader object. */
            static pluginlib::ClassLoader<query_t> loader;

        };

    }
}

#endif
