/**
 * @file roadmap_validity_checker.hpp 
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
#pragma once

#ifndef PRX_ROADMAP_VALIDITY_CHECKER_HPP
#define PRX_ROADMAP_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			/**
			 * @anchor roadmap_validity_checker_t
			 *
			 * The roadmap validity checker adjusts the given state to take into account 
			 * the latest state from the world model, then calls the world model's 
			 * function to determine whether the fixed state is valid.
			 * 
			 * @brief <b> Validity checker that works on roadmap states. </b>
			 * 
			 * @author Justin Cardoza
			 */
			class roadmap_validity_checker_t : public plan::validity_checker_t
			{
				public:
					roadmap_validity_checker_t();
					virtual ~roadmap_validity_checker_t();
					
					/**
					 * @copydoc validity_checker_t::is_valid()
					 */
					virtual bool is_valid(const sim::state_t* point);
			};
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx



#endif
