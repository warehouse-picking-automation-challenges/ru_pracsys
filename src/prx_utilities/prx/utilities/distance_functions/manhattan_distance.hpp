/**
 * @file manhattan_distance.hpp 
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

#ifndef PRX_MANHATTAN_DISTANCE_HPP
#define	PRX_MANHATTAN_DISTANCE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

namespace prx 
{ 
    namespace util
    {
		/**
		 * A class that computes the manhattan distance between two points.
		 * @brief <b> A class that computes the manhattan distance between two points. </b>
		 * @author Zakary Littlefield 
		 */
		class manhattan_distance_t : public distance_function_t
		{
		    public:
		        manhattan_distance_t(){}
		        ~manhattan_distance_t(){}
		        
		        virtual double distance(const space_point_t* s1, const space_point_t* s2);
		        
		};

	} 
}

#endif 