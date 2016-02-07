/**
 * @file world_model_validity_checker.hpp 
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

#ifndef PRX_WORLD_MODEL_VALIDITY_CHECKER_HPP
#define PRX_WORLD_MODEL_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @anchor world_model_validity_checker_t
         *
         * The world model validity checker simply calls the world model's state
         * validator to validate or invalidate states.
         *
         * @brief <b> Simple validity checker. </b>
         *
         * @author Zakary Littlefield
         */
        class world_model_validity_checker_t : public validity_checker_t
        {

          public:

            world_model_validity_checker_t() : validity_checker_t(){ }

            virtual ~world_model_validity_checker_t(){ }

            /**
             * @copydoc validity_checker_t::is_valid()
             */
            virtual bool is_valid(const sim::state_t* point);
        };

    }
}


#endif