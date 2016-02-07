/**
 * @file dprm_task_planner.cpp 
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

#include "serializing_task_planner.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::serializing_task_planner_t, prx::plan::planner_t)



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			serializing_task_planner_t::serializing_task_planner_t()
			{
				
			}
			
			
			
			serializing_task_planner_t::~serializing_task_planner_t()
			{
				
			}
			
			
			
			void serializing_task_planner_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
			{
				task_planner_t::init(reader, template_reader);
			}
			
			
			
			void serializing_task_planner_t::setup()
			{
				model->use_space(space_names[planner_names[0]]);
				planners[planner_names[0]]->setup();
				
				//PRX_WARN_S("WORLD MODEL: " << model);
				validity_checker->link_model(model);
				
				motionPlanner = dynamic_cast<plan::motion_planner_t*>(planners[planner_names[0]]);
			}
			
			
			
			const util::statistics_t* serializing_task_planner_t::get_statistics()
			{
				return planners[planner_names[0]]->get_statistics();
			}
			
			
			
			void serializing_task_planner_t::link_query(plan::query_t* input_query)
			{
				output_queries[planner_names[0]] = input_query;
				query = dynamic_cast<plan::motion_planning_query_t*>(output_queries[planner_names[0]]);
				query->link_spaces(model->get_state_space(), model->get_control_space());
				planners[planner_names[0]]->link_query(query);
				query->get_stopping_criterion()->link_motion_planner(dynamic_cast<plan::motion_planner_t*>(planners[planner_names[0]]));
				query->get_stopping_criterion()->link_goal(query->get_goal());
			}
			
			
			
			bool serializing_task_planner_t::execute()
			{
				query->get_stopping_criterion()->reset();
				
				if(motionPlanner->can_deserialize())
				{
					motionPlanner->deserialize();
				}
				else
				{
					try
					{
						motionPlanner->execute();
					}
					catch(plan::stopping_criteria_t::stopping_criteria_satisfied e)
					{
					}
				}
				
				if(motionPlanner->can_serialize())
				{
					motionPlanner->serialize();
				}
				
				resolve_query();
				
				return true;
			}
			
			
			
			void serializing_task_planner_t::resolve_query()
			{
				planners[planner_names[0]]->resolve_query();
			}
			
			
			
			bool serializing_task_planner_t::succeeded() const
			{
				return planners[planner_names[0]]->succeeded();
			}
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
