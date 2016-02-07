/**
* @file replanning.cpp
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

#include "prx/planning/task_planners/replanning/replanning_task_planner.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/modules/stopping_criteria/element/timed_criterion.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS( prx::plan::replanning_planner_t, prx::plan::planner_t)

namespace prx 
{ 
   using namespace util;
   using namespace sim;
   
   namespace plan 
   {

		replanning_planner_t::replanning_planner_t()
		{
		   s_criteria  =  NULL;
		   t_criterion = new timed_criterion_t();
		   test = false;
		}

		replanning_planner_t::~replanning_planner_t()
		{
		   delete t_criterion;
		}

		void replanning_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
		{
		   task_planner_t::init(reader, template_reader);
		   planning_duration = parameters::get_attribute_as<double>("planning_duration", reader, template_reader,5.0);
		   duration_scaling = parameters::get_attribute_as<double>("duration_scaling", reader, template_reader, 0.5);
		   reset_underlying_planner = parameters::get_attribute_as<bool>("reset", reader, template_reader, false);
		   add_time_interrupt = parameters::get_attribute_as<bool>("add_time_interrupt", reader, template_reader, true);
		}

		void replanning_planner_t::setup()
		{
		   mp_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[planner_names[0]]);

		   model->use_context(space_names[planner_names[0]]);
		   mp_specification->link_spaces(model->get_state_space(),model->get_control_space());
		   
           mp_specification->setup(model);
		   planners[planner_names[0]]->link_specification(mp_specification);
		   planners[planner_names[0]]->setup();
		   
		   s_criteria = mp_specification->get_stopping_criterion();
		   if (add_time_interrupt)
		   {
		       s_criteria->add_interruption_criterion(t_criterion);
		   }
		   
		   // This assumes the single shot always has a motion planner underneath it
		   s_criteria->link_motion_planner(dynamic_cast<motion_planner_t*>(planners[planner_names[0]]));

		   // Since we are replanning, we will add in a new criterion
		   
		   // In order to give enough time to send the plans, we scale
		   // how much time we allow the planner to plan for (i.e. half the amount of time)
		   t_criterion->set_time_limit(duration_scaling*planning_duration);
		   s_criteria->reset();
		   
		   if (reset_underlying_planner)
		   {
		       planners[planner_names[0]]->reset();
		   }
		   test = true;
		}

        void replanning_planner_t::link_specification(specification_t* in_specification)
        {
            output_specifications[planner_names[0]] = in_specification;
        }

		bool replanning_planner_t::execute()
		{        
            mp_specification->get_stopping_criterion()->reset();
			try
			{
			   planners[planner_names[0]]->execute();
			}
            catch( stopping_criteria_t::stopping_criteria_satisfied e )
            {
                throw e;
            }
		}

		const statistics_t* replanning_planner_t::get_statistics()
		{
		   return planners[planner_names[0]]->get_statistics();
		}

		bool replanning_planner_t::succeeded() const
		{
		   return planners[planner_names[0]]->succeeded();
		}

		void replanning_planner_t::link_query(query_t* in_query)
		{
			output_queries[planner_names[0]] = in_query;
			mp_query = dynamic_cast<motion_planning_query_t*>(output_queries[planner_names[0]]);
			mp_query->link_spaces(model->get_state_space(), model->get_control_space());
			mp_specification->get_stopping_criterion()->link_motion_planner(dynamic_cast<motion_planner_t*>(planners[planner_names[0]]));            
			mp_specification->get_stopping_criterion()->link_goal(mp_query->get_goal());
			planners[planner_names[0]]->link_query(mp_query);
		}

		void replanning_planner_t::resolve_query()
		{

		   if (add_time_interrupt)
		   {
		       s_criteria->add_interruption_criterion(t_criterion);
		   }
		   s_criteria->reset();

			planners[planner_names[0]]->resolve_query();
			PRX_INFO_S(mp_query->plan.length()<<" "<<mp_query->path.size());
			if(mp_query->plan.length() > this->planning_duration)
					mp_query->plan.augment_plan(this->planning_duration);
			int index = (int)((mp_query->plan.length() / simulation::simulation_step) + .1);
			PRX_INFO_S(mp_query->plan.length()<<" "<<index);
			if (mp_query->path.size() != 0)
			{
			   if (index < mp_query->path.size() )
			       mp_query->plan.copy_end_state(mp_query->path[index]);
			   else
			       mp_query->plan.copy_end_state(mp_query->path[mp_query->path.size()-1]);
			   PRX_INFO_S ("End state is : " << mp_query->state_space->print_point(mp_query->plan.get_end_state(),3));
			}
		}


		void replanning_planner_t::update_vis_info() const
		{		   
		   planners[planner_names[0]]->update_visualization();
		}

   }
}


