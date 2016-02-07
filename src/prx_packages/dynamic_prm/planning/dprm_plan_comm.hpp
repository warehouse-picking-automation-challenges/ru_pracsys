/**
 * @file dprm_plan_comm.hpp 
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

#ifndef PRX_DPRM_PLAN_COMM_HPP
#define PRX_DPRM_PLAN_COMM_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "dprm_plan_application.hpp"



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			class dprm_plan_application_t;
			
			
			/**
			 * The planning-side application for dynamic PRM. Interacts with the simulation 
			 * counterpart application to get information about the environment so it can 
			 * react to changes and replan as needed.
			 * 
			 * @brief <b>The planning-side application for dynamic PRM.</b>
			 * 
			 * @authors Justin Cardoza
			 */
			class dprm_plan_comm_t
			{
				public:
					dprm_plan_comm_t(dprm_plan_application_t *app);
					virtual ~dprm_plan_comm_t();
					
					
					
				protected:
					void stateTopicCallback(const std_msgs::String::ConstPtr& msg);
					
					
					ros::NodeHandle node;
					ros::Subscriber subscriber;
					dprm_plan_application_t *application;
			};
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx



#endif
