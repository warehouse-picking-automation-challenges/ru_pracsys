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

#include "dprm_task_planner.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::dprm_task_planner_t, prx::plan::planner_t)



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			dprm_task_planner_t::dprm_task_planner_t() : query(NULL)
			{
				
			}
			
			
			
			dprm_task_planner_t::~dprm_task_planner_t()
			{
				
			}
			
			
			
			void dprm_task_planner_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
			{
				task_planner_t::init(reader, template_reader);
			}
			
			
			
			void dprm_task_planner_t::setup()
			{
				model->use_space(space_names[planner_names[0]]);
				planners[planner_names[0]]->setup();
				
				//PRX_WARN_S("WORLD MODEL: " << model);
				validity_checker->link_model(model);
				//model->use_space("no_embedding");
				
				motionPlanner = dynamic_cast<dprm_motion_planner_t*>(planners[planner_names[0]]);
				motionPlanner->setWorldModel(model);
			}
			
			
			
			const util::statistics_t* dprm_task_planner_t::get_statistics()
			{
				return planners[planner_names[0]]->get_statistics();
			}
			
			
			
			void dprm_task_planner_t::link_query(plan::query_t* input_query)
			{
				output_queries[planner_names[0]] = input_query;
				query = dynamic_cast<plan::motion_planning_query_t*>(output_queries[planner_names[0]]);
				query->link_spaces(model->get_state_space(), model->get_control_space());
				planners[planner_names[0]]->link_query(query);
				query->get_stopping_criterion()->link_motion_planner(dynamic_cast<plan::motion_planner_t*>(planners[planner_names[0]]));
				query->get_stopping_criterion()->link_goal(query->get_goal());
			}
			
			
			
			bool dprm_task_planner_t::execute()
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
						if(motionPlanner->can_serialize())
						{
							motionPlanner->serialize();
						}
						
						return true;
					}
				}
				
				return false;
			}
			
			
			
			void dprm_task_planner_t::resolve_query()
			{
				PRX_INFO_S("Resolve query in task planner");
				motionPlanner->resolve_query();
			}
			
			
			
			bool dprm_task_planner_t::succeeded() const
			{
				return planners[planner_names[0]]->succeeded();
			}
			
			
			
			void dprm_task_planner_t::update_vis_info() const
			{
				planners[planner_names[0]]->update_visualization();
			}
			
			
			
			/*bool dprm_task_planner_t::handle_state_update(const std::string& states)
			{
				std::stringstream ss(states);
				util::hash_t<std::string, sim::plant_t*> plantHash;
				std::string plantName;
				sim::plant_t* plant, *manipulator;
				util::space_point_t *startState;
				const util::space_t *stateSpace;
				std::pair<unsigned, unsigned> indices;
				sim::system_graph_t& sysgraph = model->get_system_graph();
				std::vector<double> state;
				size_t n;
				
				sysgraph.get_name_plant_hash(plantHash);
				
				while(ss >> plantName)
				{
					if(plantName == "manipulator")
					{
						plantHash.erase(plantName);
						stateSpace = model->get_state_space();
						n = stateSpace->get_dimension();
						state.resize(n);
						
						for(size_t i = 0; i < n; i++)
							ss >> state[i];
						
						startState = stateSpace->alloc_point();
						stateSpace->set_from_vector(state, startState);
						query->set_start(startState);
					}
					else
					{
						plant = plantHash[plantName];
						
						stateSpace = plant->get_state_space();
						n = stateSpace->get_dimension();
						state.resize(n);
						
						for(size_t i = 0; i < n; i++)
							ss >> state[i];
						
						stateSpace->set_from_vector(state);
					}
					
					//PRX_WARN_S("Set plant " << plantName << "'s state to " << stateSpace->print_memory(3));
				}
				
				return motionPlanner->updateObstacles(plantHash);
			}*/
			
			
			
			void dprm_task_planner_t::handleQueryMessage(const prx_simulation::query_msg& msg)
			{
				util::hash_t<std::string, sim::plant_t*> plantHash;
				sim::system_graph_t& sysgraph = model->get_system_graph();
				sim::plant_t* plant;
				util::space_point_t *statePoint;
				const util::space_t *stateSpace;
				std::string plantName;
				size_t i, p, n;
				
				sysgraph.get_name_plant_hash(plantHash);
				stateSpace = model->get_state_space();
				statePoint = stateSpace->alloc_point();
				stateSpace->set_from_vector(msg.start, statePoint);
				query->set_start(statePoint);
				
				/*statePoint = stateSpace->alloc_point();
				stateSpace->set_from_vector(msg.goal, statePoint);
				query->set_goal(statePoint);*/
				
				for(p = 0; p < msg.plant_locations.plant_paths.size(); ++p)
				{
					plantName = util::reverse_split_path(msg.plant_locations.plant_paths[p]).second;
					plant = plantHash[plantName];
					
					if(plantName != "manipulator")
					{
						stateSpace = plant->get_state_space();
						stateSpace->set_from_vector(msg.plant_locations.plant_states[p].elements);
					}
				}
				
				motionPlanner->setPredictedTime(msg.goal_region_radius);
			}
			
			
			
			bool dprm_task_planner_t::canHandleUpdates() const
			{
				return motionPlanner->isGraphBuilt();
			}
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
