/**
 * @file l_infinite_norm.hpp 
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

#ifndef PRX_L_INFINITE_NORM_HPP
#define	PRX_L_INFINITE_NORM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

namespace prx
{
    namespace util
    {

        /**
         * A class that computes the l_infinite norm of two points. 
         * 
         * @brief <b> A class that computes the l_infinite norm of two points.</b>
         * 
         * @author Athanasios Krontiris
         */
        class l_infinite_norm_t : public distance_function_t
        {

          public:

            l_infinite_norm_t(){ }

            ~l_infinite_norm_t(){ }

            virtual double distance(const space_point_t* s1, const space_point_t* s2);

        };

    }
}

#endif 