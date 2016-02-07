/**
 * @file single_query.cpp 
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

#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/task_planners/multi_planner/multi_planner_query.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/applications/single_query_application.hpp"

#include "prx/utilities/statistics/statistics.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

namespace prx
{
    using namespace util;
    namespace plan
    {

        PLUGINLIB_EXPORT_CLASS( prx::plan::single_query_application_t, prx::plan::planning_application_t)

        using namespace comm;

        single_query_application_t::single_query_application_t()
        {
            interruptable = false;
        }

        single_query_application_t::~single_query_application_t() { }

        void single_query_application_t::init(const parameter_reader_t* reader)
        {
            planning_application_t::init(reader);

            //  Here's how you read in an interruption criteria...
            if( reader->has_attribute("interruption_criteria") )
            {
                parameter_reader_t::reader_map_t interruption_criterion_map = reader->get_map("interruption_criteria/elements");

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, interruption_criterion_map)
                {
                    const parameter_reader_t* child_template_reader = NULL;
                    std::string type, template_name;

                    if( key_value.second->has_attribute("template") )
                    {
                        template_name = key_value.second->get_attribute("template");
                        child_template_reader = new parameter_reader_t(template_name);
                    }
                    interruption_criteria.push_back(parameters::initialize_from_loader<criterion_t > ("prx_planning", key_value.second, "", child_template_reader,""));
                    interruption_criteria.back()->set_type(interruption_criteria.back()->get_type() + "_I" + int_to_str(interruption_criteria.size()));
                    if( child_template_reader != NULL )
                        delete child_template_reader;
                }

                interruptable = true;
            }

            //DEBUG: Testing out a path smoother
            if( reader->has_attribute("path_smoother") )
            {
                path_smoother = (path_smoother_t*)reader->initialize_from_loader<planner_t > ("path_smoother", "prx_planning");
            }
            else
            {
                path_smoother = NULL;
            }
            
            PRX_ASSERT_MSG(root_queries[0] != NULL, "Root queries must have 1 non-null query");
        }

        void single_query_application_t::execute()
        {
            this->root_task->link_specification(root_specifications[0]);
            this->root_task->setup();
            this->root_task->link_query(root_queries[0]);
            
            // If we've defined an interrupt criterion
            if( interruptable )
            {
                bool done = false;

                // First link the interrupt into the query
                dynamic_cast<motion_planning_specification_t*>(root_specifications[0])->get_stopping_criterion()->link_interruption_criteria(interruption_criteria);

                // Loop to ensure we completely go through the algorithm's execution
                while( !done )
                {
                    try
                    {
                        done = this->root_task->execute();
                    }
                    catch( stopping_criteria_t::interruption_criteria_satisfied e )
                    {
                        PRX_DEBUG_S("Interruption satisfied and caught! " << e.what());

                        // If we want to keeep interrupting, then reset the criteria

                        foreach(criterion_t* crit, interruption_criteria)
                        {
                            crit->reset();
                        }

                        // You must relink the interruption
                        dynamic_cast<motion_planning_specification_t*>(root_specifications[0])->get_stopping_criterion()->link_interruption_criteria(interruption_criteria);
                    }
                }
            }
            else
            {
                this->root_task->execute();
            }            

            const statistics_t* stats = this->root_task->get_statistics();

            if( stats != NULL )
            {
                PRX_INFO_S("Planner Statistics: " << stats->get_statistics());
            }

            //If we have an initialized path smoother, attempt to smooth out the solutions we have
            if( path_smoother != NULL )
            {
                PRX_DEBUG_COLOR("Beginning path smoothing...", PRX_TEXT_CYAN);
                path_smoother->link_world_model(model);
                path_smoother->link_query(root_queries[0]);
                for( unsigned i = 0; i < 3; ++i ) //TODO : Should really try to set up the criterion for the smoother.
                {
                    path_smoother->step();
                }
                PRX_DEBUG_COLOR("Query smoothing succesfully completed!", PRX_TEXT_GREEN);
            }

            if( visualize )
            {
                PRX_INFO_S("Updating visualization geoms.");

                this->root_task->update_visualization();

                ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
            }
            if( simulate )
            {
                if( dynamic_cast<multi_planner_query_t*>(root_queries[0]) != NULL )
                {

                    foreach(query_t* query, dynamic_cast<multi_planner_query_t*>(root_queries[0])->planner_queries | boost::adaptors::map_values)
                    {
                        ((comm::planning_comm_t*)comm::plan_comm)->publish_plan(consumer_path, ((motion_planning_query_t*)(query))->plan);
                    }
                }
                else
                    ((comm::planning_comm_t*)comm::plan_comm)->publish_plan(consumer_path, dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan);
            }
            PRX_INFO_S("Single query application has finished!");

        }

    }
}
