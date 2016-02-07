/**
 * @file manipulation_application_t.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/applications/single_query_application.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#ifndef PRX_MANIPULATION_APPLICATION_HPP
#define	PRX_MANIPULATION_APPLICATION_HPP

namespace prx
{
    namespace plan
    {
        class criterion_t;
    }
    
    namespace packages
    {
        namespace manipulation
        {

            /**
             * @anchor manipulation_application_t
             *
             * This application is specific for the manipulation problem. It is similar to single_query_application
             * but it also know how to deal with graphs that have been sent to it. 
             *
             * @brief <b> Planning application for manipulation tasks. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulation_application_t : public plan::single_query_application_t
            {

              public:
                manipulation_application_t();
                virtual ~manipulation_application_t();

                /**
                 * @copydoc planning_application_t::init()
                 */
                virtual void init(const util::parameter_reader_t* reader);

                /**
                 * @copydoc planning_application_t::execute()
                 *
                 * This execute function will actually terminate once the planning is
                 * completed.
                 */
                virtual void execute();

                /**
                 * 
                 * @copydoc planning_application_t::graph_callback(const prx_simulation::graph_msg&)
                 * 
                 * The function will store the graph into a file where the task planner will be able 
                 * to locate and read the graph. 
                 */
                virtual void process_graph_callback(const prx_simulation::graph_msg& msg);
                
                /**
                 * 
                 * @copydoc planning_application_t::process_ack_callback(const prx_simulation::manipulation_acknowledgement&)
                 * 
                 * The function will process the acknowledgement message from the other node.
                 */
                virtual void process_ack_callback(const prx_simulation::manipulation_acknowledgement& msg);
                
              private:
                int get_time_in_sec();
                void list_file();

              protected:                
                std::string grasped_graph_file_name;             
                std::string ungrasped_graph_file_name;
                std::string statistics_file_name;
                bool serialize_flag;
                bool deserialize_flag;
                bool ready_to_deserialize;
                bool graph_builder;
                bool graph_user;
                int sync_time;
                int sending_time;
                
                std::string roadmaps_dir;
                std::string store_dir;
                std::vector<std::string> file_names;
                int file_index;
                
                std::string stats_file;
                                
                util::sys_clock_t statistics_clock;
                
            };

        }
    }
}

#endif
