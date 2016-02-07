/**
 * @file dprm_plan_application.hpp 
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

#ifndef PRX_DPRM_PLAN_APPLICATION_HPP
#define PRX_DPRM_PLAN_APPLICATION_HPP

#include "prx/planning/applications/planning_application.hpp"
#include "dprm_plan_comm.hpp"
 


namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			class dprm_plan_comm_t;
			class dprm_task_planner_t;
			
			
			/**
			 * Planning application for dynamic PRM planners. Interacts with the counterpart 
			 * application in simulation to respond to changes in the environment.
			 * 
			 * @brief <b>Planning application for dynamic PRM planners.</b>
			 * 
			 * @author Justin Cardoza
			 */
			class dprm_plan_application_t : public plan::planning_application_t
			{
				public:
					dprm_plan_application_t();
					virtual ~dprm_plan_application_t();
					
					/**
					 * @copydoc planning_application_t::init()
					 */
					virtual void init(const util::parameter_reader_t* reader);
					
					/**
					 * @copydoc planning_application_t::execute()
					 */
					virtual void execute();
					
					//void receiveStateUpdate(const std::string& data);
					void process_query_callback(const prx_simulation::query_msg& msg);
					
				protected:
					dprm_plan_comm_t *communicator;
					dprm_task_planner_t *taskPlanner;
			};
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx



#endif