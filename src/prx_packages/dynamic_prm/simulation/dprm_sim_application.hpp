/**
 * @file dprm_sim_application.hpp
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

#pragma once

#ifndef PRX_DPRM_SIM_APPLICATION_HPP
#define PRX_DPRM_SIM_APPLICATION_HPP

#include <string>

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/applications/empty_application.hpp"
#include "dprm_sim_comm.hpp"


namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			class dprm_sim_comm_t;
			
			
			/**
			 * Application for Dynamic PRM experiments. Mainly needed 
			 * for interfacing with custom communication classes.
			 * 
			 * @brief <b>Application for Dynamic PRM experiments.</b>
			 * 
			 * @authors Justin Cardoza
			 */
			class dprm_sim_application_t : public sim::empty_application_t
			{
				public:
					dprm_sim_application_t();
					virtual ~dprm_sim_application_t();
					
					/** @copydoc sim::application_t::init(const util::parameter_reader_t * const) */
					virtual void init(const util::parameter_reader_t * const reader);
					
					/**
					 * Serializes the simulator state into a string containing the 
					 * state data and the names of the corresponding plants.
					 * 
					 * @brief <b>Serializes the simulator state.</b>
					 * 
					 * @param str The standard string variable to store the serialization into.
					 */
					void serializeState(std::string& str);
					
					/**
					 * Monitors simulation step events, and every \ref interval seconds 
					 * tells the communication class to publish the current simulation 
					 * state. See also \ref prx::sim::application_t::frame(const ros::TimerEvent&)
					 * 
					 * @brief <b>Monitors simulation step events</b>
					 */
					virtual void frame(const ros::TimerEvent& event);
					
					/**
					 * Sets the update interval for state messages to \ref sec seconds.
					 * 
					 * @brief <b>Sets the update message interval.</b>
					 * 
					 * @param sec The new update interval in seconds.
					 */
					void setInterval(double sec);
					
					
				protected:
					dprm_sim_comm_t *communicator;
					double interval, timer;
			};
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx


#endif