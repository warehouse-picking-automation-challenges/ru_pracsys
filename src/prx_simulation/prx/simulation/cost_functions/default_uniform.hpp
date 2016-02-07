/**
 * @file default_uniform.hpp 
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

#ifndef PRX_DEFAULT_UNIFORM_HPP
#define	PRX_DEFAULT_UNIFORM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/cost_functions/cost_function.hpp"

namespace prx 
{ 
	namespace sim 
	{
		class default_uniform_t : public cost_function_t
		{
		public:
		    default_uniform_t(){}
		    ~default_uniform_t(){}
		    
            virtual double state_cost(const util::space_point_t* s);

            virtual double trajectory_cost(const trajectory_t& t);

            virtual double heuristic_cost(const util::space_point_t* s,const util::space_point_t* t);
		    
		};
	} 
}

#endif 