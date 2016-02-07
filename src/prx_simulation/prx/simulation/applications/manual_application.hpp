/**
 * @file manual_application.hpp
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

#ifndef PRX_MANUAL_APPLICATION_HPP
#define	PRX_MANUAL_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/applications/empty_application.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

namespace prx
{
    namespace sim
    {

        class manual_controller_t;

        /**
         * An application which handles manually controlling multiple systems.
         * A test application for the manual controller. 
         * 
         * @brief <b>A test application for the manual controller.</b>
         * 
         * @authors Andrew Kimmel
         */
        class manual_application_t : public empty_application_t
        {

          public:
            manual_application_t();
            virtual ~manual_application_t();

            /** @copydoc empty_application_t::init(const util::parameter_reader_t * const) */
            virtual void init(const util::parameter_reader_t * const reader);

            /** 
             * @copydoc empty_application_t::set_selected_path(const std::string&) 
             * 
             * @note When a system is selected the application will deactivate the manual
             * controller from the previous system and will activate the manual controller
             * for the new selected system.
             */
            virtual void set_selected_path(const std::string& path);

          protected:

            /**
             * The list with the paths for all the manual controllers. This will be build
             * once at the beginning and it will be used when the user requests to activate/deactivate
             * manual controllers, by selecting different plants.
             * 
             * @brief The list with the paths for all the manual controllers. 
             */
            std::vector<std::string> manual_controller_paths;

            /**
             * A list with all the paths for all the manual controllers. This will be build
             * once at the beginning and it will be used when the user requests to activate/deactivate
             * manual controllers, by selecting different plants.
             * 
             * @brief A list with all the paths for all the manual controllers.
             */
            std::vector<manual_controller_t*> manual_controllers;
        };

    }
}

#endif	

