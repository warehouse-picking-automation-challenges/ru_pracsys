/**
 * @file base_apc_task_query.hpp
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

#ifndef PRX_BASE_APC_TASK_QUERY_HPP
#define PRX_BASE_APC_TASK_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/utilities/goals/goal.hpp"

namespace prx
{
    namespace plan
    {
        class base_apc_task_query_t : public motion_planning_query_t
        {

          public:
            /**
             * @copydoc query_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            void clear_goal()
            {
                goal->clear();
            }

        };

    }
}

#endif 
