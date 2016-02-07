/**
 * @file base_apc_task_specification.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_BASE_APC_TASK_SPECIFICATION_HPP
#define	PRX_BASE_APC_TASK_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }
    
    namespace plan
    {

        class base_apc_task_specification_t : public specification_t
        {

          public:
            base_apc_task_specification_t();
            virtual ~base_apc_task_specification_t();

            /**
             * @brief Initialize the planing query from input parameters.
             *
             * Initialize the planing query from input parameters.
             * 
             * @param reader The reader for the parameters.
             * @param template_reader A template reader for reading template parameters 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

        };

    }
}

#endif

