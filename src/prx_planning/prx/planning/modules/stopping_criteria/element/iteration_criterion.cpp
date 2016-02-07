/**
 * @file iteration_criterion.cpp 
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

#include "prx/planning/modules/stopping_criteria/element/iteration_criterion.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <pluginlib/class_list_macros.h>

namespace prx 
{ 
    using namespace util;
    namespace plan 
    {

        PLUGINLIB_EXPORT_CLASS( prx::plan::iteration_criterion_t, prx::plan::criterion_t)

        iteration_criterion_t::iteration_criterion_t()
        {
            PRX_DEBUG_S("Created an iteration criterion");
            iteration_counter = 0;
        }

        iteration_criterion_t::~iteration_criterion_t()
        {
            
        }
        void iteration_criterion_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader )
        {
            max_iterations = parameters::get_attribute_as<double>("condition", reader, template_reader);
            criterion_type = parameters::get_attribute_as<std::string>("criterion_type", reader, template_reader, "iteration");
        }

        bool iteration_criterion_t::criterion_check()
        {
        //    PRX_DEBUG_S ("Criterion Check: " << iteration_counter);
            iteration_counter++;
            
            if (iteration_counter >= max_iterations)
                return true;
            else
                return false;
        }

        void iteration_criterion_t::set_max_iterations(int new_max)
        {
            max_iterations = new_max;
        }

        void iteration_criterion_t::reset()
        {
            iteration_counter = 0;
        }

        std::string iteration_criterion_t::print_statistics()
        {
            return std::string("Iteration counter: " + int_to_str(iteration_counter));
        }
        
        int iteration_criterion_t::get_max_iterations()
        {
            return max_iterations;
        }


    }
}