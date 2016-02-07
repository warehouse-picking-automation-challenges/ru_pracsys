/**
 * @file succeeded_criterion.cpp
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

#include "prx/planning/modules/stopping_criteria/element/succeeded_criterion.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"

#include <pluginlib/class_list_macros.h>

namespace prx
{
    using namespace util;
    namespace plan
    {

        PLUGINLIB_EXPORT_CLASS( prx::plan::succeeded_criterion_t, prx::plan::criterion_t)

        succeeded_criterion_t::succeeded_criterion_t()
        {
            PRX_DEBUG_S("Created a \'succeeded\' criterion");
        }

        succeeded_criterion_t::~succeeded_criterion_t()
        {
        }

        void succeeded_criterion_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader )
        {
            criterion_type = parameters::get_attribute_as<std::string>("criterion_type", reader, template_reader, "succeeded");
        }

        bool succeeded_criterion_t::criterion_check()
        {
            //Note: Most motion planners are currently set up to throw upon a
            //call to succeeded().  This is definitely something that needs to
            //be addressed and talked about in a future meeting.
            return linked_motion_planner->succeeded();
        }

        void succeeded_criterion_t::reset()
        {
        }

        std::string succeeded_criterion_t::print_statistics()
        {
            return std::string("");
        }

    }
}
