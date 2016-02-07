/**
* @file roadmap_validity_checker.cpp 
* 
* @copyright Software License Agreement (BSD License)
* Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
* All Rights Reserved.
* For a full description see the file named LICENSE.
* 
* Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
* 
* Email: pracsys@googlegroups.com
*/

#include "roadmap_validity_checker.hpp"
#include <pluginlib/class_list_macros.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::roadmap_validity_checker_t, prx::plan::validity_checker_t)



namespace prx
{
	using namespace util;
	using namespace sim;
	
	namespace packages
	{
		namespace dynamic_prm
		{
			roadmap_validity_checker_t::roadmap_validity_checker_t() : plan::validity_checker_t()
			{
				
			}
			
			
			
			roadmap_validity_checker_t::~roadmap_validity_checker_t()
			{
				
			}
			
			
			
			bool roadmap_validity_checker_t::is_valid(const state_t* point)
			{
				std::vector<double> vec;
				space_t *stateSpace = world_model->get_state_space();
				unsigned int dim = stateSpace->get_dimension();
				
				for(size_t i = 0; i < dim; ++i)
					vec.push_back(point->at(i));
				
				stateSpace->set_from_vector(vec);
				stateSpace->copy_to_point((state_t*)point);
				
				return world_model->valid_state(point);
			}//is_valid()
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
