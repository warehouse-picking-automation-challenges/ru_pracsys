/**
* @file replanning.hpp
*
* @copyright Software License Agreement (BSD License)
* Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
* All Rights Reserved.
* For a full description see the file named LICENSE.
* 
* Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
* 
* Email: pracsys@googlegroups.com
*/
#pragma once

#ifndef PRX_REPLANNING_HPP
#define	PRX_REPLANNING_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/task_planners/task_planner.hpp"

namespace prx 
{ 
   namespace plan 
   {

		class stopping_criteria_t;
		class timed_criterion_t;
		class motion_planning_query_t;
		class motion_planning_specification_t;
		class specification_t;

		/**
		* @anchor replanning_planner_t
		*
		* For dynamic environments, simple open-loop planning is generally insufficient.
		* In such situations this task planner can employ a sampling-based planner to
		* recompute paths on the fly given new ground truth information.
		*
		* @brief <b> Task planner repsonsible for replanning tasks. </b>
		*
		* @author Andrew Kimmel
		*/
		class replanning_planner_t : public task_planner_t
		{    
		public:
		   
		   replanning_planner_t();
		   virtual ~replanning_planner_t();

		   /**
		    * @copydoc task_planner_t::init()
		    */
		   virtual void init(const util::parameter_reader_t* reader,const util::parameter_reader_t* template_reader);
		   
		   /**
		    * @copydoc task_planner_t::setup()
		    *
		    * The setup also adds an additional interruption criterion which is a timed
		    * criterion which interrupts planning at each planning cycle.
		    */
		   virtual void setup();
		   
		   /**
		    * @copydoc task_planner_t::execute()
		    */
		   virtual bool execute();
		       
		   /**
		    * @copydoc task_planner_t::get_statistics()
		    */
		   virtual const util::statistics_t* get_statistics();
		   
		   /**
		    * @copydoc task_planner_t::succeeded()
		    */
		   virtual bool succeeded() const;
		   
            /**
             * @copydoc task_planner_t::link_specification(specification_t*)
             */
            virtual void link_specification(specification_t* in_specification);

		   /**
		    * @copydoc task_planner_t::link_query()
		    */
		   virtual void link_query(query_t* in_query);
		   
		   /**
		    * @copydoc task_planner_t::resolve_query()
		    */
		   virtual void resolve_query();
		   
		protected:
		   /**
		    * @copydoc task_planner_t::update_vis_info()
		    */
		   virtual void update_vis_info() const;

		   /** @brief Pointer to the motion planning query's stopping criterion. */
		   stopping_criteria_t* s_criteria;
		   /** @brief Pointer to the motion planner's specifications.  */
		   motion_planning_specification_t* mp_specification;
		   /** @brief Pointer to the motion planner's query.  */
		   motion_planning_query_t* mp_query;
		   
		   /** @brief Timed criterion used for planning cycle timeout detection. */
		   timed_criterion_t* t_criterion;
		   /** @brief The duration of a single planning cycle. */
		   double planning_duration;
		   /** @brief Scaling parameter for how much time is given to planning per planning cycle. */
		   double duration_scaling;
		   
		   /** @brief Boolean indicating whether the replanning has ever been set up. */
		   bool test; //TODO : Make this a more meaningful variable?
		   
		   bool reset_underlying_planner;
		   bool add_time_interrupt;
		};

   }
}

#endif

