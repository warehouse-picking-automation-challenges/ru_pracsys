/**
 * @file empty_application.hpp
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

#ifndef PRX_EMPTY_APPLICATION_HPP
#define	PRX_EMPTY_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/applications/application.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * An empty application. It is used mostly for the planning applications. It will
         * not do anything from the simulation side. 
         * 
         * @brief <b> An empty application </b>
         * 
         * @authors Zakary Littlefield
         */
        class empty_application_t : public application_t
        {

          public:
            empty_application_t();
            virtual ~empty_application_t();

            /** @copydoc application_t::set_selected_path(const std::string&)*/
            virtual void set_selected_path(const std::string& path);

        };

    }
}

#endif

