/**
 * @file dprm_plan_comm.cpp 
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

#include "dprm_plan_comm.hpp"



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			dprm_plan_comm_t::dprm_plan_comm_t(dprm_plan_application_t *app)
			{
				application = app;
				subscriber = node.subscribe("simulation/dprm_state", 1, &dprm_plan_comm_t::stateTopicCallback, this);
			}
			
			
			
			dprm_plan_comm_t::~dprm_plan_comm_t()
			{
				
			}
			
			
			
			void dprm_plan_comm_t::stateTopicCallback(const std_msgs::String::ConstPtr& msg)
			{
				//PRX_WARN_S("Received state: " << msg->data);
				//application->receiveStateUpdate(msg->data);
			}
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
