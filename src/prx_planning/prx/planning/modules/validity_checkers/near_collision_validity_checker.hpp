/**
 * @file near_collision_validity_checker.hpp 
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

#ifndef PRX_NEAR_COLLISION_VALIDITY_CHECKER_HPP
#define PRX_NEAR_COLLISION_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/validity_checkers/ignore_list_validity_checker.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * The world model validity checker simply calls the world model's state
         * validator to validate or invalidate states.
         *
         * @brief <b> Clearance-based validity checker. </b>
         *
         * @author Andrew Dobson
         */
        class near_collision_validity_checker_t : public ignore_list_validity_checker_t
        {

          public:

            near_collision_validity_checker_t() : ignore_list_validity_checker_t(){ ignore_list = NULL; }

            virtual ~near_collision_validity_checker_t(){ }

            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
            
            /**
             * @copydoc validity_checker_t::is_valid()
             */
            virtual bool is_valid(const sim::state_t* point);
          
          private:
            
            double clearance;
        };

    }
}


#endif