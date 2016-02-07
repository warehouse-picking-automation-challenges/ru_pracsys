/**
 * @file single_shot.cpp
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

#include "prx/planning/task_planners/single_shot/single_shot.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/sst/sst_statistics.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/planning/applications/single_query_application.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/adaptor/map.hpp>
#include <iostream>
#include <fstream>
#include <ros/ros.h>

////    These lines setup the configuration function for transforming state into
////    configurations for visualization purposes. WE WILL NEED THESE

////    comm::visualization_comm_t::state_to_config_t to_config =
////            boost::bind(&world_model_t::get_config, model, _1, _2, _3);


PLUGINLIB_EXPORT_CLASS(prx::plan::single_shot_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    namespace plan
    {

        void single_shot_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
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
        }

        void single_shot_planner_t::setup()
        {
            specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[planner_names[0]]);

            PRX_INFO_S("Setting the world model to use space " << space_names[planner_names[0]]);
            model->use_context(space_names[planner_names[0]]);
            specification->link_spaces(model->get_state_space(), model->get_control_space());
            specification->setup(model);
            planners[planner_names[0]]->link_specification(specification);
            planners[planner_names[0]]->setup();
        }

        bool single_shot_planner_t::execute()
        {

            foreach(criterion_t* crit, stats_criteria)
            {
                crit->reset();
            }
            specification->get_stopping_criterion()->link_interruption_criteria(stats_criteria);
            specification->get_stopping_criterion()->reset();
            bool done = false;
            std::vector<const statistics_t*> stats;

            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_output/single_shot_output/");
            boost::filesystem::path output_dir(dir);
            if( !boost::filesystem::exists(output_dir) )
            {
                boost::filesystem::create_directory(output_dir);
            }
            std::string output_directory = dir;

            PRX_INFO_S("Performing single shot execute");
            double best_solution = PRX_INFINITY;
            while( !done )
            {
                try
                {
                    planners[planner_names[0]]->execute();
                }
                catch( stopping_criteria_t::interruption_criteria_satisfied e )
                {
                    planners[planner_names[0]]->resolve_query();
                    stats.push_back(planners[planner_names[0]]->get_statistics());
                    //            PRX_INFO_S(planners[planner_names[0]]->get_statistics()->get_statistics());

                    // double val = dynamic_cast<const sst_statistics_t*>(stats.back())->solution_quality;
                    // if(val>0.01 && val < best_solution)
                    // {
                    //     best_solution = val;
                    //     PRX_INFO_S(stats.back()->get_statistics());
                    // }
                    // update_visualization();

                    // ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                    // If we want to keep interrupting, then reset the criteria

                    foreach(criterion_t* crit, stats_criteria)
                    {
                        crit->reset();
                    }

                    // You must relink the interruption
                    specification->get_stopping_criterion()->link_interruption_criteria(stats_criteria);
                }
                catch( stopping_criteria_t::stopping_criteria_satisfied e )
                {
                    PRX_INFO_S(e.what());
                    planners[planner_names[0]]->resolve_query();
                    stats.push_back(planners[planner_names[0]]->get_statistics());
                    done = true;

                    std::string filename = output_directory + ros::this_node::getName() + "_";
                    filename += planner_names[0] + ".txt";
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

        const statistics_t* single_shot_planner_t::get_statistics()
        {
            return planners[planner_names[0]]->get_statistics();
        }

        bool single_shot_planner_t::succeeded() const
        {
            return planners[planner_names[0]]->succeeded();
        }

        void single_shot_planner_t::link_specification(specification_t* in_specification)
        {
            output_specifications[planner_names[0]] = in_specification;
        }

        void single_shot_planner_t::link_query(query_t* in_query)
        {
            //this will overwrite any query read from input
            output_queries[planner_names[0]] = in_query;

            query = dynamic_cast<motion_planning_query_t*>(output_queries[planner_names[0]]);
            query->link_spaces(model->get_state_space(), model->get_control_space());
            specification->get_stopping_criterion()->link_motion_planner(dynamic_cast<motion_planner_t*>(planners[planner_names[0]]));            
            specification->get_stopping_criterion()->link_goal(query->get_goal());
            planners[planner_names[0]]->link_query(query);
        }

        void single_shot_planner_t::resolve_query()
        {
            PRX_FATAL_S("Single shot planner does not support multiple queries");
        }

        void single_shot_planner_t::update_vis_info() const
        {
            PRX_DEBUG_S("Single shot update_vis_info in "<<planner_names[0]);
            planners[planner_names[0]]->update_visualization();
        }

    }
}


