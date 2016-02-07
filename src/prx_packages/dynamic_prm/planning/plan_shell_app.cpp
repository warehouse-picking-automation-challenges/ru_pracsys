/**
 * @file plan_shell_app.hpp 
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

#include "plan_shell_app.hpp"
#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::plan_shell_app_t, prx::plan::planning_application_t);



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			plan_shell_app_t::plan_shell_app_t()
			{
				PRX_WARN_S("Created the planning shell application.");
			}
			
			
			
			plan_shell_app_t::~plan_shell_app_t()
			{
				
			}
			
			
			
			void plan_shell_app_t::init(const util::parameter_reader_t* reader)
			{
				//planning_application_t::init(reader);
				
				/*std::string plugin_type_name;
				plugin_type_name = reader->get_attribute("plan_comm", "planning_comm");
				plan::comm::plan_comm = plan::plan_base_communication_t::get_loader().createUnmanagedInstance("prx_planning/" + plugin_type_name);
				plan::comm::plan_comm->link_application(this);
				
				plugin_type_name = reader->get_attribute("plan_to_sim_comm", "simulation_comm");
				plan::comm::sim_comm = plan::plan_base_communication_t::get_loader().createUnmanagedInstance("prx_planning/" + plugin_type_name);
				plan::comm::sim_comm->link_application(this);
				
				plugin_type_name = reader->get_attribute("plan_to_vis_comm", "visualization_comm");
				plan::comm::vis_comm = plan::plan_base_communication_t::get_loader().createUnmanagedInstance("prx_planning/" + plugin_type_name);
				plan::comm::vis_comm->link_application(this);*/
				
				
				std::string node_name = ros::this_node::getName();
				ros::NodeHandle node;
				queryRepublisher = node.advertise<prx_simulation::query_msg>("republishedQueries", 10);
				planPublisher = node.advertise<prx_simulation::plan_msg>(node_name + "/plans", 10, true);
				querySubscriber = node.subscribe("/simulation/planning/planning_queries", 1, &plan_shell_app_t::process_query_callback, this);
				solutionSubscriber = node.subscribe("planningSolutions", 1, &plan_shell_app_t::solutionCallback, this);
			}
			
			
			
			void plan_shell_app_t::execute()
			{
				
			}
			
			
			
			void plan_shell_app_t::process_query_callback(const prx_simulation::query_msg& msg)
			{
				PRX_INFO_S("In process_query_callback()...");
				
				while(queryRepublisher.getNumSubscribers() == 0)
					sleep(0);
				
				PRX_INFO_S("Republished query.");
				
				queryRepublisher.publish(msg);
			}
			
			
			
			void plan_shell_app_t::solutionCallback(const prx_simulation::plan_msg& msg)
			{
				PRX_INFO_S("In solutionCallback()...");
				
				while(planPublisher.getNumSubscribers() == 0)
					sleep(0);
				
				PRX_INFO_S("Published solution.");
				
				planPublisher.publish(msg);
			}
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
