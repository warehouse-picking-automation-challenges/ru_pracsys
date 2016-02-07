/**
 * @file default_euclidean.hpp 
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

#ifndef PRX_DEFAULT_EUCLIDEAN_HPP
#define	PRX_DEFAULT_EUCLIDEAN_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

namespace prx 
 { 
 namespace util 
 {

/**
 * A class containing a wrapper around the space distance function.
 * @brief <b> A class containing a wrapper around the space distance function. </b>
 * @author Zakary Littlefield 
 */
class default_euclidean_t : public distance_function_t
{
    public:
        default_euclidean_t(){}
        ~default_euclidean_t(){}
        
        virtual double distance(const space_point_t* s1, const space_point_t* s2);
        
};

} 
 }

#endif 