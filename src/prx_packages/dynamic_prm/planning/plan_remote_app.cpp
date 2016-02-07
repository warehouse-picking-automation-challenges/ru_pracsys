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

#include "plan_remote_app.hpp"

#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/utilities/spaces/embedded_space.hpp"
#include "prx/planning/task_planners/multi_planner/multi_planner_query.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::plan_remote_app_t, prx::plan::planning_application_t);



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			plan_remote_app_t::plan_remote_app_t()
			{
				
			}
			
			
			
			plan_remote_app_t::~plan_remote_app_t()
			{
				
			}
			
			
			
			void plan_remote_app_t::init(const util::parameter_reader_t* reader)
			{
				planning_application_t::init(reader);
				
				ros::NodeHandle node;
				solutionPublisher = node.advertise<prx_simulation::plan_msg>("planningSolutions", 10);
				querySubscriber = node.subscribe("republishedQueries", 1, &plan_remote_app_t::queryCallback, this);
			}
			
			
			
			void plan_remote_app_t::execute()
			{
				taskPlanner = dynamic_cast<dprm_task_planner_t*>(this->root_task);
				
				this->root_task->link_query(root_queries[0]);
				this->root_task->setup();
				this->root_task->execute();
				this->root_task->resolve_query();
				
				/*if(visualize)
				{
					this->root_task->update_visualization();
					((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->send_geometries();
				}*/
				
				if(simulate)
				{
					PRX_ERROR_S("Plan: " << dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan.print());
					//((plan::comm::planning_comm_t*)plan::comm::plan_comm)->publish_plan(consumer_path, dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan);
					sendPlanMsg();
				}
			}
			
			
			
			void plan_remote_app_t::queryCallback(const prx_simulation::query_msg& msg)
			{
				PRX_INFO_S("Processing a query...");
				taskPlanner->handleQueryMessage(msg);
				taskPlanner->resolve_query();
				
				if(simulate)
				{
					//PRX_ERROR_S("Plan: " << dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan.print());
					//((plan::comm::planning_comm_t*)plan::comm::plan_comm)->publish_plan(consumer_path, dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan);
					sendPlanMsg();
				}
			}
			
			
			
			void plan_remote_app_t::makePlanMsg(prx_simulation::plan_msg& msg)
			{
				sim::plan_t& p = dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan;
				
				msg.system_name = consumer_path;
				msg.plan.reserve(p.size());
				
				foreach(sim::plan_step_t step, p)
				{
					prx_simulation::control_msg control;
					sim::control_t* storage;
					
					control.duration = step.duration;
					
					if(dynamic_cast<util::embedded_point_t*>(step.control) != NULL)
						storage = dynamic_cast<util::embedded_point_t*>(step.control)->link;
					else
						storage = step.control;
					
					std::vector<double> converted_control = plan_to_sim_control(storage);
					
					for(unsigned int i = 0; i < converted_control.size(); ++i)
						control.control.push_back(converted_control[i]);
					
					msg.plan.push_back(control);
				}
				
				if(p.get_end_state() != NULL)
				{
					for(unsigned int i = 0; i < p.get_end_state()->memory.size(); ++i)
					{
						msg.end_state.push_back(p.get_end_state()->at(i));
					}
				}
			}
			
			
			
			void plan_remote_app_t::sendPlanMsg()
			{
				prx_simulation::plan_msg msg;
				
				PRX_INFO_S("Sending plan...");
				makePlanMsg(msg);
				
				while(solutionPublisher.getNumSubscribers() == 0)
				{
					sleep(0);
				}
				
				PRX_INFO_S("Plan sent.");
				solutionPublisher.publish(msg);
			}
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
