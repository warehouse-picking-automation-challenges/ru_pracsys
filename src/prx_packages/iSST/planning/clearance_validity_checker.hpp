/**
 * @file clearance_validity_checker.hpp 
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

#ifndef PRX_CLEARANCE_VALIDITY_CHECKER_HPP
#define PRX_CLEARANCE_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/simulation/cost_functions/cost_function.hpp"

namespace prx
{
    namespace packages
    {
        namespace isst
        {


            class clearance_t : public sim::cost_function_t
            {
            public:
                clearance_t(){}
                ~clearance_t(){}
                
                virtual double state_cost(const util::space_point_t* s);

                virtual double trajectory_cost(const sim::trajectory_t& t);
                virtual double heuristic_cost(const util::space_point_t* s,const util::space_point_t* t);

                void link_model(plan::world_model_t* wm);

            protected:
                plan::world_model_t* model;
                
            };

            /**
             * @anchor clearance_validity_checker_t
             *
             * Uses clearances as cost function
             *
             * @brief <b> Simple validity checker. </b>
             *
             * @author Zakary Littlefield
             */
            class clearance_validity_checker_t : public plan::validity_checker_t
            {

              public:

                clearance_validity_checker_t() : validity_checker_t(){ }

                virtual ~clearance_validity_checker_t(){ }

                /**
                 * @brief Link a distance function for this validity checker to use.
                 *
                 * @param model The distance function to link to this validity checker.
                 */
                virtual void link_distance_function(util::distance_t dist);

                /**
                 * @copydoc validity_checker_t::is_valid()
                 */
                virtual bool is_valid(const sim::state_t* point);
            };
        }
    }
}


#endif