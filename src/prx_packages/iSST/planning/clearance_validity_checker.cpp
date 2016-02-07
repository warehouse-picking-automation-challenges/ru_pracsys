/**
 * @file clearance_validity_checker.cpp 
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


#include "planning/clearance_validity_checker.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::isst::clearance_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    namespace packages
    {
    	namespace isst
    	{

	        double clearance_t::state_cost(const space_point_t* s)
	        {
	        	double val = model->state_clearance(s);
	        	if(val<=PRX_ZERO_CHECK)
	        		return PRX_INFINITY;
	        	return 1.0/val;
	        }
	        void clearance_t::link_model(world_model_t* wm)
	        {
	        	model = wm;
	        }

	        double clearance_t::trajectory_cost(const trajectory_t& t)
	        {
	            PRX_ASSERT(t.size()>0);
	            
	            double cost = 0;
	            trajectory_t::const_iterator i = t.begin();
	            trajectory_t::const_iterator j = t.begin();
	            j++;

	            for ( ; j != t.end(); ++i,++j)
	            {
	            	double temp = state_cost(*j);
	            	if(temp<PRX_INFINITY)
	                	cost+=dist(*i,*j)*temp;
	                else
	                {
	                	cost = PRX_INFINITY;
	                	break;
	                }
	            }
	            return cost;
	        }
	        double clearance_t::heuristic_cost(const util::space_point_t* s,const util::space_point_t* t)
	        {
	            return dist(s,t)*(.005);
	        }


	        bool clearance_validity_checker_t::is_valid(const state_t* point)
	        {
	            return world_model->valid_state(point);
	        }
	        void clearance_validity_checker_t::link_distance_function(util::distance_t dist)
	        {
	            cost_function = new clearance_t();
	            cost_function->link_distance_function(dist);
	            dynamic_cast<clearance_t*>(cost_function)->link_model(world_model);
	            state_cost = cost_function->cost_of_state;
	            trajectory_cost = cost_function->cost_of_trajectory;
	        }
	    }
    }
}