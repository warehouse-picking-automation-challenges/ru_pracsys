/**
 * @file feedback_controller.cpp
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

#include "feedback_controller.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::feedback_controller_t, prx::sim::system_t);



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			feedback_controller_t::feedback_controller_t() : consumer_controller_t()
			{
				PRX_WARN_S("Feedback controller created!");
				started = false;
			}
			
			
			
			void feedback_controller_t::copy_plan(const plan_t& inplan)
			{
				PRX_WARN_S("Feedback controller is working!");
				
				double elapsed = lastPlanClock.measure();
				double averageTime = elapsed;//0.0;
				
				
				//Plan adjustment method -- old code.
				/*if(started && inplan.length() > elapsed)
				{
					plan_t adjustedPlan(inplan);
					adjustedPlan.consume_control(elapsed);
					
					if(adjustedPlan.size() > 0)
						consumer_controller_t::copy_plan(adjustedPlan);
				}
				else if(!started && inplan.size() > 0)
				{
					started = true;
					consumer_controller_t::copy_plan(inplan);
					lastPlanClock.reset();
				}*/
				
				
				//Prediction method -- new code.
				if(!started)
				{
					started = true;
					lastPlanClock.reset();
					elapsed = 0.0;
				}
				
				if(inplan.size() > 0)
					consumer_controller_t::copy_plan(inplan);
				
				
				
				/*history.push_back(elapsed);
				if(history.size() > 5)
					history.pop_front();
				
				for(std::deque<double>::iterator it = history.begin(); it != history.end(); ++it)
				{
					averageTime += *it;
				}
				
				averageTime /= history.size();*/
				
				/*goal_index = 0;
				queried_planner = false;
				PRX_WARN_S("active_queries: " << active_queries << ", goal_index: " << goal_index << ", goal_states.size(): " << goal_states.size());
				query_planner();*/
				
				child_state_space->copy_to_point(get_state);
				std::vector<double> start_state, goal_state;
				for( unsigned i = 0; i < child_state_space->get_dimension(); i++ )
				{
					start_state.push_back(get_state->at(i));
					//goal_state.push_back(get_state->at(i));
				}
				
				PRX_ERROR_S("Planning node: " << planning_node);
				((comm::planning_comm_t*)comm::plan_comm)->publish_planning_query(
					start_state, goal_state, averageTime, pathname, planning_node, 
					true, smooth, true, false);
				
				lastPlanClock.reset();
				PRX_ERROR_S("End of copy_plan function");
			}
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx