 /**
 * @file heuristic_task_planner.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/heuristic_task_planner.hpp"
#include "planning/heuristic_task_specification.hpp"
// #include "planning/heuristic_task_query.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/motion_planners/sst/isst.hpp"
#include "prx/planning/motion_planners/rrt_star/rrt_star.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::isst::heuristic_task_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    namespace packages
    {
    	namespace isst
    	{

	        void heuristic_task_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
	        {
	            task_planner_t::init(reader, template_reader);
	            if( reader->has_attribute("stats_criteria") )
	            {
	                parameter_reader_t::reader_map_t stats_criterion_map = reader->get_map("stats_criteria/elements");

	                foreach(const parameter_reader_t::reader_map_t::value_type key_value, stats_criterion_map)
	                {
	                    PRX_DEBUG_S("Creating criterion from namespace: " << key_value.first);
	                    const parameter_reader_t* child_template_reader = NULL;
	                    std::string type, template_name;

	                    if( key_value.second->has_attribute("template") )
	                    {
	                        template_name = key_value.second->get_attribute("template");
	                        child_template_reader = new parameter_reader_t(template_name);
	                    }

	                    type = parameters::get_attribute_as<std::string > ("type", reader, child_template_reader);
	                    stats_criteria.push_back(parameters::initialize_from_loader<criterion_t > ("prx_planning", key_value.second, "", child_template_reader, ""));
	                    stats_criteria.back()->set_type(stats_criteria.back()->get_type() + "_I" + int_to_str(stats_criteria.size()));
	                    if( child_template_reader != NULL )
	                        delete child_template_reader;
	                }
	            }
	            real_planner_name = parameters::get_attribute_as<std::string > ("real_planner", reader, template_reader);
	            heuristic_planner_name = parameters::get_attribute_as<std::string > ("heuristic_planner", reader, template_reader);

	            real_motion_planner = dynamic_cast<isst_t*>(planners[real_planner_name]);
	            heuristic_planner = dynamic_cast<motion_planner_t*>(planners[heuristic_planner_name]);

	            heuristic_query = new motion_planning_query_t();
	            heuristic_goal = new goal_state_t();

	            real_motion_planner->h = boost::bind(&heuristic_task_planner_t::calculate_heuristic, this, _1);
	        }

	        void heuristic_task_planner_t::setup()
	        {
	        	//setup the real motion planner
	        	PRX_INFO_S("Setting up real motion planner");
	            model->use_context(space_names[real_planner_name]);
	            space_t* real_space = model->get_state_space();
	            specification->real_specification->link_spaces(model->get_state_space(), model->get_control_space());
	            specification->real_specification->setup(model);
	            real_motion_planner->link_specification(specification->real_specification);
	            real_motion_planner->setup();

	            //setup the heuristic planner
	        	PRX_INFO_S("Setting up heuristic motion planner: "<<space_names[heuristic_planner_name]);
	            model->use_context(space_names[heuristic_planner_name]);
	            space_t* app_space = model->get_state_space();
	            heuristic_state = app_space->alloc_point();
	            specification->heuristic_specification->link_spaces(model->get_state_space(), model->get_control_space());
	            specification->heuristic_specification->setup(model);
	            heuristic_planner->link_specification(specification->heuristic_specification);
	            heuristic_planner->setup();

	            //setup the query for the heuristic
	            heuristic_goal->link_metric(specification->heuristic_specification->metric);
	            heuristic_query->link_spaces(model->get_state_space(), model->get_control_space());
	            heuristic_goal->link_space(model->get_state_space());
	            heuristic_query->set_goal(heuristic_goal);

	            //setup mapping function
	            specification->state_mapping->preimage_space = real_space;
            	specification->state_mapping->preimage_interval = std::pair<unsigned, unsigned>(0, real_space->get_dimension());
            	specification->state_mapping->image_space = app_space;
            	specification->state_mapping->image_interval = std::pair<unsigned, unsigned>(0, app_space->get_dimension());

	        }
            double heuristic_task_planner_t::calculate_heuristic(const util::space_point_t* state)
            {
            	model->use_context(space_names[heuristic_planner_name]);
            	specification->state_mapping->preimage_space->copy_from_point(state);
            	specification->state_mapping->embed();
            	specification->state_mapping->image_space->copy_to_point(heuristic_state);
            	dynamic_cast<goal_state_t*>(heuristic_query->get_goal())->set_goal_state(heuristic_state);
            	double cost = heuristic_planner->compute_cost();
	            model->use_context(space_names[real_planner_name]);
	            return cost;
            }

	        bool heuristic_task_planner_t::execute()
	        {
	        	PRX_INFO_S("SWITCHING TO "<<space_names[heuristic_planner_name]);
            	model->use_context(space_names[heuristic_planner_name]);
                try
                {
                    heuristic_planner->execute();
                }
                catch( stopping_criteria_t::stopping_criteria_satisfied e )
                {
                	PRX_INFO_S("Heuristic Data Structure Completed!");
                }
	            foreach(criterion_t* crit, stats_criteria)
	            {
	                crit->reset();
	            }
	            model->use_context(space_names[real_planner_name]);
	            specification->real_specification->get_stopping_criterion()->link_interruption_criteria(stats_criteria);
	            specification->real_specification->get_stopping_criterion()->reset();
	            bool done = false;
	            std::vector<const statistics_t*> stats;

	            char* w = std::getenv("PRACSYS_PATH");
	            std::string dir(w);
	            dir += ("/prx_output/heuristic_output/");
	            boost::filesystem::path output_dir(dir);
	            if( !boost::filesystem::exists(output_dir) )
	            {
	                boost::filesystem::create_directory(output_dir);
	            }
	            std::string output_directory = dir;

	            // PRX_INFO_S("Performing single shot execute");
	            string_hash s;
		        unsigned int random_seed = s(ros::this_node::getName());
		        init_random(random_seed);

		        PRX_INFO_S(ros::this_node::getName()<<" "<<random_seed<<" "<<time(NULL));
	            while( !done )
	            {
	                try
	                {
	                    real_motion_planner->execute();
	                }
	                catch( stopping_criteria_t::interruption_criteria_satisfied e )
	                {
	                    real_motion_planner->resolve_query();
	                    stats.push_back(real_motion_planner->get_statistics());
	                    //            PRX_INFO_S(planners[planner_names[0]]->get_statistics()->get_statistics());

	                    // If we want to keep interrupting, then reset the criteria

	                    foreach(criterion_t* crit, stats_criteria)
	                    {
	                        crit->reset();
	                    }

	                    // You must relink the interruption
	                    specification->real_specification->get_stopping_criterion()->link_interruption_criteria(stats_criteria);
	                }
	                catch( stopping_criteria_t::stopping_criteria_satisfied e )
	                {
	                    real_motion_planner->resolve_query();
	                    stats.push_back(real_motion_planner->get_statistics());
	                    done = true;

	                    std::string filename = output_directory + ros::this_node::getName() + "_";
	                    filename += real_planner_name + ".txt";
	                    std::ofstream fout;
	                    fout.open(filename.c_str());

	                    foreach(const statistics_t* stat, stats)
	                    {
	                        stat->serialize(fout);
	                    }
	                    fout.close();


	                    char* w = std::getenv("PRACSYS_PATH");
	                    std::string dir(w);
	                    std::stringstream s1;
	                    s1 << "/prx_output/published_plans/";
	                    dir += (s1.str());
	                    boost::filesystem::path output_dir(dir);
	                    if( !boost::filesystem::exists(output_dir) )
	                    {
	                        boost::filesystem::create_directories(output_dir);
	                    }
	                    std::string dir2 = dir;
	                    dir += "plan.txt";
	                    dir2 += "trajectory.txt";

	                    query->plan.save_to_file(dir);
	                    query->path.save_to_file(dir2);


	                    return true;
	                }
	            }

	            return false;
	        }

	        const statistics_t* heuristic_task_planner_t::get_statistics()
	        {
	            return real_motion_planner->get_statistics();
	        }

	        bool heuristic_task_planner_t::succeeded() const
	        {
	            return real_motion_planner->succeeded();
	        }

	        void heuristic_task_planner_t::link_specification(specification_t* in_specification)
	        {
	            specification = dynamic_cast<heuristic_task_specification_t*>(in_specification);
	        }

	        void heuristic_task_planner_t::link_query(query_t* in_query)
	        {
	            //this will overwrite any query read from input
	            output_queries[real_planner_name] = in_query;
	            output_queries[heuristic_planner_name] = in_query;

            	model->use_context(space_names[real_planner_name]);
	            query = dynamic_cast<motion_planning_query_t*>(in_query);
	            query->link_spaces(model->get_state_space(), model->get_control_space());
	            specification->real_specification->get_stopping_criterion()->link_motion_planner(dynamic_cast<motion_planner_t*>(real_motion_planner));            
	            specification->real_specification->get_stopping_criterion()->link_goal(query->get_goal());
	            real_motion_planner->link_query(query);



            	model->use_context(space_names[heuristic_planner_name]);
	            heuristic_planner->link_query(heuristic_query);
	            //TODO create a goal point using the mapping function, give it as goal
            	specification->state_mapping->preimage_space->copy_from_point(query->get_start_state());
	            specification->state_mapping->embed();
	            heuristic_goal->set_goal_state(specification->state_mapping->image_space->alloc_point());
	        }

	        void heuristic_task_planner_t::resolve_query()
	        {
	            PRX_FATAL_S("heuristic_task_planner_t does not support multiple queries");
	        }

	        void heuristic_task_planner_t::update_vis_info() const
	        {
	            model->use_context(space_names[heuristic_planner_name]);
	            heuristic_planner->update_visualization();
	            model->use_context(space_names[real_planner_name]);
	            real_motion_planner->update_visualization();
	        }
	    }
    }
}


