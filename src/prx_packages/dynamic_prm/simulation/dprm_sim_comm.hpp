/**
 * @file dprm_sim_comm.hpp
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

#ifndef PRX_DPRM_SIM_COMM_HPP
#define PRX_DPRM_SIM_COMM_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>

#include "dprm_sim_application.hpp"



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			class dprm_sim_application_t;
			
			
			/**
			 * Simulation communication class for dynamic PRM. Used for sending 
			 * the current plant states to the planner so it can react to changes 
			 * in the environment.
			 * 
			 * @brief <b>Simulation-side communication class for Dynamic PRM experiments.</b>
			 * 
			 * @authors Justin Cardoza
			 */
			class dprm_sim_comm_t
			{
				public:
					dprm_sim_comm_t(dprm_sim_application_t *app);
					virtual ~dprm_sim_comm_t();
					
					void publishState();
					
					
				protected:
					//bool dprm_sim_comm_t::stateServiceCallback(std_srvs::Empty::Request& request, std_srvs::String::Response& response);
					
					
					ros::NodeHandle node;
					ros::Publisher stateTopic;
					//ros::ServiceServer stateService;
					dprm_sim_application_t *application;
			};
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx



#endif