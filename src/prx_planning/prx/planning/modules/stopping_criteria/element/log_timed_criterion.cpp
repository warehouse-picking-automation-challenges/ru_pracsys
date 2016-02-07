/**
 * @file stopping_criterion.cpp 
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

#include "prx/planning/modules/stopping_criteria/element/log_timed_criterion.hpp"
#include "prx/utilities/statistics/statistics.hpp"

#include <pluginlib/class_list_macros.h>

namespace prx 
{ 
    using namespace util;
    namespace plan 
    {

        PLUGINLIB_EXPORT_CLASS( prx::plan::log_timed_criterion_t, prx::plan::criterion_t)

        log_timed_criterion_t::log_timed_criterion_t()
        {
            PRX_DEBUG_S ("Created a log timed criterion");
            criterion_type = "log_timed";
            first_reset = true;
        }

        log_timed_criterion_t::~log_timed_criterion_t()
        {
            
        }
        void log_timed_criterion_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader )
        {
            time_limit = parameters::get_attribute_as<double>("condition", reader, template_reader, PRX_INFINITY);
            criterion_type = parameters::get_attribute_as<std::string>("criterion_type", reader, template_reader, "log_timed");
        }

        bool log_timed_criterion_t::criterion_check()
        {
        //    PRX_DEBUG_S ("Criterion check: " << timer.measure());
            if (timer.measure() >= time_limit)
                return true;
            else
                return false;
        }

        void log_timed_criterion_t::set_time_limit(double new_time_limit)
        {
            time_limit = new_time_limit;
            first_reset = true;
        }

        double log_timed_criterion_t::get_time()
        {
            return timer.measure();
        }

        double log_timed_criterion_t::get_remaining_time()
        {
            return time_limit - timer.measure();
        }

        void log_timed_criterion_t::reset()
        {
            timer.reset();
            if(first_reset)
            {
                first_reset = false;
            }
            else
            {
                time_limit*=2;
            }
        }

        std::string log_timed_criterion_t::print_statistics()
        {
            char s[32];
            std::sprintf(s, "%f", timer.measure());
            std::string ret(s);
            return std::string("Timer: ") + ret;
        }

    }
}