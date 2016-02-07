/**
 * @file trrt_cost_function.hpp 
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

#ifndef PRX_TRRT_COST_HPP
#define	PRX_TRRT_COST_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/cost_functions/default_uniform.hpp"

namespace prx 
{ 
	namespace sim 
	{
		/**
		 * Implements the cost transition from the T-RRT* paper.
		 * 
		 * Efficient Sampling-based Approaches to Optimal Path Planning in Complex Cost Spaces
		 * Didier Devaurs, Thierry Sim´eon, and Juan Cort´es, WAFR `14
		 * 
		 * @brief Implements the cost transition from the T-RRT* paper.
		 * 
		 */
		class trrt_cost_function_t : public default_uniform_t
		{
		public:
		    trrt_cost_function_t();
		    ~trrt_cost_function_t();
		    
            virtual double state_cost(const util::space_point_t* s);

            virtual double trajectory_cost(const trajectory_t& t);
        protected:

        	double* heightmap;
            int max_x;
            int max_y;
		    
		};
	} 
}

#endif 