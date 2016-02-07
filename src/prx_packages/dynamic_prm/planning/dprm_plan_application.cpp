/**
 * @file dprm_plan_application.cpp 
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

#include "dprm_plan_application.hpp"
#include <pluginlib/class_list_macros.h>

#include "dprm_plan_comm.hpp"
#include "dprm_task_planner.hpp"

#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/planning/task_planners/multi_planner/multi_planner_query.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"



PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::dprm_plan_application_t, prx::plan::planning_application_t);



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			dprm_plan_application_t::dprm_plan_application_t()
			{
				PRX_WARN_S("Loaded DPRM application.");
			}
			
			
			
			dprm_plan_application_t::~dprm_plan_application_t()
			{
				
			}
			
			
			
			void dprm_plan_application_t::init(const util::parameter_reader_t* reader)
			{
				planning_application_t::init(reader);
			}
			
			
			
			void dprm_plan_application_t::execute()
			{
				taskPlanner = dynamic_cast<dprm_task_planner_t*>(this->root_task);
				
				this->root_task->link_query(root_queries[0]);
				this->root_task->setup();
				this->root_task->execute();
				this->root_task->resolve_query();
				
				//Statistics?
				
				//Path smoothing?
				
				if(visualize)
				{
					this->root_task->update_visualization();
					((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->send_geometries();
				}
				
				if(simulate)
				{
					PRX_ERROR_S("Plan: " << dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan.print());
					((plan::comm::planning_comm_t*)plan::comm::plan_comm)->publish_plan(consumer_path, dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan);
				}
				
				communicator = new dprm_plan_comm_t(this);
			}
			
			
			
			/*void dprm_plan_application_t::receiveStateUpdate(const std::string& data)
			{
				//PRX_ERROR_S("State update: " << data);
				//PRX_WARN_S("State update: " << data);
				
				if(taskPlanner->canHandleUpdates() && taskPlanner->handle_state_update(data))
				{
					PRX_ERROR_S("Replanning...");
					taskPlanner->resolve_query();
					
					if(visualize)
					{
						this->root_task->update_visualization();
						((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->send_geometries();
					}
					
					if(simulate)
					{
						PRX_ERROR_S("Plan: " << dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan.print());
						((plan::comm::planning_comm_t*)plan::comm::plan_comm)->publish_plan(consumer_path, dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan);
					}
				}
			}*/
			
			
			
			void dprm_plan_application_t::process_query_callback(const prx_simulation::query_msg& msg)
			{
				taskPlanner->handleQueryMessage(msg);
				taskPlanner->resolve_query();
				
				if(simulate)
				{
					//PRX_ERROR_S("Plan: " << dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan.print());
					((plan::comm::planning_comm_t*)plan::comm::plan_comm)->publish_plan(consumer_path, dynamic_cast<plan::motion_planning_query_t*>(root_queries[0])->plan);
				}
			}//process_query_callback()
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
