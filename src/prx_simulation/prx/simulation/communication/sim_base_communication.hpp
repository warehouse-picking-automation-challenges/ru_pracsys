/**
 * @file sim_base_communication.hpp 
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

#ifndef PRX_SIM_BASE_COMM_HPP
#define	PRX_SIM_BASE_COMM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/communication/communication.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace sim
    {

        class application_t;

        class sim_base_communication_t : public util::communication_t
        {

          public:
            /**
             * The function links the application to this communication class.
             * @param parent_app
             * @return 
             */
            virtual void link_application(application_t* parent_app);


            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<sim_base_communication_t>& get_loader();

          protected:
            /** @brief Pointer to the application */
            application_t* app;

          private:


            /** 
             * The pluginlib loader for the \ref sim_base_communication_t class.
             * 
             * @brief The pluginlib loader for the \ref sim_base_communication_t class.
             */
            static pluginlib::ClassLoader<sim_base_communication_t> loader;

        };

    }
}


#endif