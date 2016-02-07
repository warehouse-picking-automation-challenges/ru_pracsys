///**
// * @file multi_planner.cpp
// *
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// * 
// * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
// * 
// * Email: pracsys@googlegroups.com
// */
//
//#include "prx/planning/task_planners/multi_planner/multi_planner.hpp"
//#include "prx/planning/task_planners/multi_planner/multi_planner_query.hpp"
//#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
//#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
//#include "prx/planning/motion_planners/motion_planner.hpp"
//#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
//#include "prx/utilities/definitions/string_manip.hpp"
//
//
//#include <pluginlib/class_list_macros.h>
//#include <boost/range/adaptor/map.hpp>
//
//PLUGINLIB_EXPORT_CLASS( prx::plan::multi_planner_t, prx::plan::planner_t)
//
//namespace prx 
//{ 
//    using namespace util;
//    namespace plan 
//    {
//multi_planner_t::multi_planner_t() : task_planner_t()
//{
//    stats = new multi_planner_statistics_t();
//}
//multi_planner_t::~multi_planner_t()
//{
//    delete stats;
//}
//
//void multi_planner_t::setup()
//{
//    foreach(std::string planner_name, planner_names )
//    {
//        PRX_WARN_S("Planner name: "<<planner_name<<" Output query: "<<output_queries[planner_name]);
//        motion_planning_specification_t* specification = dynamic_cast<motion_planning_specification_t*>(output_queries[planner_name]);
//        motion_planning_query_t* query = dynamic_cast<motion_planning_query_t*>(output_queries[planner_name]);
//
//        PRX_WARN_S("Setting the world model to use space "<<space_names[planner_name]);
//        model->use_context(space_names[planner_name]);
//        specification->link_spaces(model->get_state_space(),model->get_control_space());
//        query->link_spaces(model->get_state_space(),model->get_control_space());
//                
//        planners[planner_name]->link_specification(specification);
//        planners[planner_name]->link_query(query);
//
//        // This assumes the multi_planner always has motion planners underneath it
//        specification->get_stopping_criterion()->link_motion_planner(dynamic_cast<motion_planner_t*>(planners[planner_name]));
//        specification->get_stopping_criterion()->link_goal(query->get_goal());
//    }
//    
//}
//void multi_planner_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
//{
//    task_planner_t::init(reader,template_reader);
//    if (reader->has_attribute("stats_criteria"))
//    {
//        parameter_reader_t::reader_map_t stats_criterion_map = reader->get_map("stats_criteria/elements");
//
//        foreach(const parameter_reader_t::reader_map_t::value_type key_value, stats_criterion_map)
//        {
//            PRX_DEBUG_S("Criterion name: "<<key_value.first);
//            const parameter_reader_t* child_template_reader = NULL;  
//            std::string type, template_name;
//
//            if(key_value.second->has_attribute("template"))
//            {
//                template_name = key_value.second->get_attribute("template");
//                child_template_reader = new parameter_reader_t(template_name);  
//            }
//
//            type = parameters::get_attribute_as<std::string>("type", reader, child_template_reader);
//            stats_criteria.push_back(parameters::initialize_from_loader<criterion_t>("prx_planning", key_value.second,"",child_template_reader,""));
//            stats_criteria.back()->set_type(stats_criteria.back()->get_type() + "_I" + int_to_str(stats_criteria.size()));
//            if(child_template_reader!=NULL)
//                delete child_template_reader;
//        }
//    }    
//}
//
//bool multi_planner_t::execute()
//{       
//    //loops over all the planners, doing the following
//    // setup
//    // execute
//    //   catch interrupts for statistics gathering
//    // reset the motion_planner
//    // repeat until all planners are done
//    
//    foreach(std::string planner_name, planner_names )
//    {
//        model->use_context(space_names[planner_name]);
//        planners[planner_name]->setup();
//        
//        bool done = false;        
//        // First link the interrupt into the query
//                
//        // If we want to keep interrupting, then reset the criteria
////        foreach(criterion_t* crit, stats_criteria)
////        {
////            crit->reset();
////        }
//        dynamic_cast<motion_planning_query_t*>(output_queries[planner_name])->get_stopping_criterion()->link_interruption_criteria(stats_criteria);
//        dynamic_cast<motion_planning_query_t*>(output_queries[planner_name])->get_stopping_criterion()->reset();
//        
//        // Loop to ensure we completely go through the algorithm's execution
//        int count = 0 ;
//        while (!done)
//        {
//            try
//            {
//                PRX_INFO_S("Executing in the multiplanner");
//                done = planners[planner_name]->execute();
//            }
//            catch (stopping_criteria_t::interruption_criteria_satisfied e)
//            {
//                count++;
//                PRX_ERROR_S("Stats! "<<planner_name<<" "<<count);
//                planners[planner_name]->resolve_query();
//                stats->planner_statistics[planner_name].push_back(planners[planner_name]->get_statistics());
//                
//                // If we want to keep interrupting, then reset the criteria
//                foreach(criterion_t* crit, stats_criteria)
//                {
//                    crit->reset();
//                }
//                
//                // You must relink the interruption
//                dynamic_cast<motion_planning_query_t*>(output_queries[planner_name])->get_stopping_criterion()->link_interruption_criteria(stats_criteria);
//            }
//            catch(stopping_criteria_t::stopping_criteria_satisfied e)
//            {
//                PRX_ERROR_S(""<<planner_name<<" is done!");
//                planners[planner_name]->resolve_query();
//                stats->planner_statistics[planner_name].push_back(planners[planner_name]->get_statistics());
//                stats->output_stats();
//                done = true;
//            }
//        }
//        planners[planner_name]->update_visualization();
//        delete planners[planner_name];
////        planners[planner_name]->reset();
//    }
//    stats->output_stats();
//    return true;
//}
//
//const statistics_t* multi_planner_t::get_statistics()
//{
//    //this can return the statistics that this task planner will create
//    return stats;
//}
//
//bool multi_planner_t::succeeded() const
//{
//    //this can return if all the planners succeeded
//    return true;
//}
//
//void multi_planner_t::link_query(query_t* in_query)
//{
//    //this should read in a query that contains multiple motion_planning_queries
//    // and sets them into the output queries hash
//    multi_planner_query_t* q = dynamic_cast<multi_planner_query_t*>(in_query);
//    output_queries = q->planner_queries;
//}
//
//void multi_planner_t::resolve_query()
//{
//    PRX_LOG_ERROR("Multi planner does not support multiple queries");
//}
//
//
//void multi_planner_t::update_vis_info() const
//{
////    PRX_LOG_ERROR("Multi planner does not support updating visualization");
//}
//
//    }
//}
//
//
