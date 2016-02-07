/**
 * @file empty_application.cpp
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


#include "prx/simulation/applications/empty_application.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::empty_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    
    namespace sim
    {        

        empty_application_t::empty_application_t() { }

        empty_application_t::~empty_application_t() { }

        void empty_application_t::set_selected_path(const std::string& path)
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = reverse_split_path(path);
            selected_path = name;
            PRX_DEBUG_COLOR("The selected path is now: " << selected_path.c_str(),PRX_TEXT_GREEN);
        }

    }
}

