/**
 * @file manual_application.cpp
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

#include "prx/simulation/applications/manual_application.hpp"
#include "prx/simulation/systems/controllers/manual/manual_controller.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::manual_application_t, prx::sim::application_t)

namespace prx
{

    using namespace util;
    
    namespace sim
    {
        manual_application_t::manual_application_t() { }

        manual_application_t::~manual_application_t() { }

        void manual_application_t::init(const parameter_reader_t * const reader)
        {
            application_t::init(reader);


            PRX_DEBUG_COLOR("Manual application init", PRX_TEXT_GREEN);
            manual_controller_t* controller;
            system_ptr_t current_system;

            foreach(const parameter_reader_t * const body_reader, reader->get_child("manual_controllers")->get_list("system_paths"))
            {
                std::string body_name = body_reader->get_attribute("");
                manual_controller_paths.push_back(body_name);
                PRX_DEBUG_COLOR("Read in manual controller path: " << body_name, PRX_TEXT_CYAN);


                current_system = simulator->get_system(body_name);
                controller = dynamic_cast<manual_controller_t*>(current_system.get());
                if( !controller )
                    PRX_FATAL_S("Pathname given for manual controller %s does not exist " << body_name.c_str());
                manual_controllers.push_back(controller);

                delete body_reader;
            }
        }

        void manual_application_t::set_selected_path(const std::string& path)
        {
            //    empty_application_t::set_selected_path(path);

            std::string name, subpath, manual_path;
            system_ptr_t current_system; // = simulator->get_system(name); ;
            boost::tie(name, subpath) = reverse_split_path(path);
            try
            {
                current_system = simulator->get_system(name);
            }
            catch( system_t::invalid_path_exception path_exception )
            {
                PRX_WARN_S("Tried to select something that does not exist in the simulation system tree!");
                return;
            }
            manual_controller_t* controller;
            controller = dynamic_cast<manual_controller_t*>(current_system.get());

            while( controller == NULL && !subpath.empty() && std::strcmp(simulator->get_pathname().c_str(), name.c_str()) )
            {
                manual_path = name;
                current_system = simulator->get_system(name);
                controller = dynamic_cast<manual_controller_t*>(current_system.get());
                boost::tie(name, subpath) = reverse_split_path(name);
            }

            // Set all manual controllers to be inactive
            for( unsigned i = 0; i < manual_controllers.size(); i++ )
            {
                manual_controllers[i]->set_active(false, "");
            }
            if( controller != NULL )
            {
                PRX_DEBUG_COLOR("Found manual controller to set: " << manual_path, PRX_TEXT_BROWN);
                boost::tie(name, subpath) = reverse_split_path(manual_path);
                // Set the selected manual controller to be active
                simulator->set_active(true, subpath);
            }
            else
            {
                PRX_WARN_S("Selected a plant with no manual controller!");
            }
        }


    }
}

