/**
 * @file dprm_sim_application.cpp
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

#include "dprm_sim_application.hpp"

#include <vector>
#include <sstream>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::dprm_sim_application_t, prx::sim::application_t)


namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			dprm_sim_application_t::dprm_sim_application_t() : communicator(NULL), interval(0.1), timer(0.0)
			{
				PRX_ERROR_S("Constructing a dprm_sim_application_t...");
			}
			
			
			
			dprm_sim_application_t::~dprm_sim_application_t()
			{
				if(communicator != NULL)
					delete communicator;
			}
			
			
			
			void dprm_sim_application_t::init(const util::parameter_reader_t * const reader)
			{
				sim::application_t::init(reader);
				
				PRX_ERROR_S("In dprm_sim_application_t::init()!");
				
				communicator = new dprm_sim_comm_t(this);
				interval = reader->get_attribute_as<double>("interval", 0.1);
				
				PRX_ERROR_S("Update interval: " << interval);
			}
			
			
			
			void dprm_sim_application_t::serializeState(std::string& str)
			{
				std::stringstream ss;
				std::vector<sim::plant_t*> plants;
				std::vector<std::string> plantNames;
				std::pair<unsigned, unsigned> indices;
				const util::space_t *plantSpace;
				size_t n, i, j;
				
				sys_graph.get_plants(plants);
				sys_graph.get_plant_names(plantNames);
				
				n = plants.size();
				
				for(i = 0; i < n; i++)
				{
					ss << plantNames[i] << ' ';
					plantSpace = plants[i]->get_state_space();
					indices = simulation_state_space->get_subspace(plantSpace);
					
					for(j = indices.first; j < indices.second; j++)
					{
						ss << simulation_state->at(j) << ' ';
					}
					
					ss << '\t';
				}
				
				str = ss.str();
			}//serializeSpace()
			
			
			
			void dprm_sim_application_t::frame(const ros::TimerEvent& event)
			{
				application_t::frame(event);
				
				timer += event.current_real.toSec() - event.last_real.toSec();
				if(timer >= interval && communicator != NULL)
				{
					timer = 0.0;
					communicator->publishState();
				}
			}
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx
