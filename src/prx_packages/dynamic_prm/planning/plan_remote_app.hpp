/**
 * @file plan_remote_app.hpp 
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

#ifndef PRX_DPRM_PLAN_REMOTE_APP_HPP
#define PRX_DPRM_PLAN_REMOTE_APP_HPP

#include "prx/planning/applications/planning_application.hpp"
#include "dprm_task_planner.hpp"
#include "prx_simulation/plan_msg.h"
#include "prx_simulation/query_msg.h"
#include <ros/ros.h>



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			class plan_remote_app_t : public plan::planning_application_t
			{
				public:
					plan_remote_app_t();
					virtual ~plan_remote_app_t();
					
					/**
					 * @copydoc planning_application_t::init()
					 */
					virtual void init(const util::parameter_reader_t* reader);
					
					/**
					 * @copydoc planning_application_t::execute()
					 */
					virtual void execute();
					
					void queryCallback(const prx_simulation::query_msg& msg);
					
				protected:
					void makePlanMsg(prx_simulation::plan_msg& msg);
					void sendPlanMsg();
					
					dprm_task_planner_t *taskPlanner;
					ros::Subscriber querySubscriber;
					ros::Publisher solutionPublisher;
			};
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx



#endif
