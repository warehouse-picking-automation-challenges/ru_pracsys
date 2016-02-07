/**
 * @file visualization.cpp 
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

#include "prx/visualization/visualization.hpp"

namespace prx
{
    using namespace util;
    namespace vis
    {

        const char* const visualization_t::plugin_type_name = "visualization";
        
        void visualization_t::update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf)
        {
            PRX_WARN_S ("No default implementation of update info geoms!");
        }

        
        void visualization_t::take_screenshot(unsigned screen_num, int num_screenshots)
        {
            PRX_WARN_S ("No default implementation of take screenshot!");
        }

    }
}