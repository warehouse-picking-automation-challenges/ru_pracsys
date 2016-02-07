/**
 * @file feedback_controller.hpp
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

#ifndef PRX_FEEDBACK_CONTROLLER_HPP
#define PRX_FEEDBACK_CONTROLLER_HPP

#include "prx/simulation/systems/controllers/consumer/consumer_controller.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include <deque>



namespace prx
{
	using namespace sim;
	
	namespace packages
	{
		namespace dynamic_prm
		{
			class feedback_controller_t : public consumer_controller_t
			{
				public:
					feedback_controller_t();
					void copy_plan(const plan_t& inplan);
					
				private:
					util::sys_clock_t lastPlanClock;
					bool started;
					std::deque<double> history;
			};
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx



#endif
