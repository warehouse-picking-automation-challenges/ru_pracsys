/**
 * @file dprm_sim_comm.cpp
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

#include "dprm_sim_comm.hpp"
//#include <pluginlib/class_list_macros.h>


//PLUGINLIB_EXPORT_CLASS( prx::sim::empty_application_t, prx::sim::application_t)


namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			dprm_sim_comm_t::dprm_sim_comm_t(dprm_sim_application_t *app)
			{
				application = app;
				stateTopic = node.advertise<std_msgs::String>("simulation/dprm_state", 1);
				//stateService = node.advertiseService("simulation/dprm_state_service", &dprm_sim_comm_t::stateServiceCallback, this);
			}
			
			
			
			dprm_sim_comm_t::~dprm_sim_comm_t()
			{
				
			}
			
			
			
			void dprm_sim_comm_t::publishState()
			{
				std_msgs::String msg;
				application->serializeState(msg.data);
				stateTopic.publish(msg);
			}
			
			
			/*
			bool dprm_sim_comm_t::stateServiceCallback(std_srvs::Empty::Request& request, std_srvs::String::Response& response)
			{
				application->serializeState(response.data);
			}*/
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
