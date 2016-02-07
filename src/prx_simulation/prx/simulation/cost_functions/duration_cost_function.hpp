/**
 * @file duration_cost_function.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_DURATION_COST_HPP
#define	PRX_DURATION_COST_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/cost_functions/default_uniform.hpp"

namespace prx 
{ 
	namespace sim 
	{
		/**
		 *  
		 */
		class duration_cost_function_t : public default_uniform_t
		{
		public:
		    duration_cost_function_t();
		    ~duration_cost_function_t();
		    
            virtual double state_cost(const util::space_point_t* s);

            virtual double trajectory_cost(const trajectory_t& t);
            
            virtual double heuristic_cost(const util::space_point_t* s,const util::space_point_t* t);
		    
		};
	} 
}

#endif 