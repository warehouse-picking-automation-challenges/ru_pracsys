/**
 * @file dprm_task_planner.hpp 
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

#ifndef PRX_SERIALIZING_TASK_PLANNER_HPP
#define PRX_SERIALIZING_TASK_PLANNER_HPP



#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "dprm_motion_planner.hpp"

#include <string>



namespace prx
{
	namespace packages
	{
		namespace dynamic_prm
		{
			class serializing_task_planner_t : public plan::task_planner_t
			{
				public:
					serializing_task_planner_t();
					virtual ~serializing_task_planner_t();
					
					/** 
					 * @copydoc planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t* )
					 */
					virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
					
					/** 
					 * @copydoc planner_t::setup()
					 */
					virtual void setup();
					
					/** 
					 * @copydoc planner_t::get_statistics()
					 */
					virtual const util::statistics_t* get_statistics();
					
					/** 
					 * @copydoc planner_t::link_query(query_t*)
					 */
					virtual void link_query(plan::query_t* input_query);
					
					/** 
					 * @copydoc planner_t::execute()
					 */
					virtual bool execute();
					
					/** 
					 * @copydoc planner_t::resolve_query()
					 */
					virtual void resolve_query();
					
					/** 
					 * @copydoc planner_t::succeeded()
					 */
					virtual bool succeeded() const;
					
				protected:
					plan::motion_planner_t *motionPlanner;
					plan::motion_planning_query_t* query;
			};
		}//namespace dynamic_prm
	}//namespace packages
}//namespace prx




#endif