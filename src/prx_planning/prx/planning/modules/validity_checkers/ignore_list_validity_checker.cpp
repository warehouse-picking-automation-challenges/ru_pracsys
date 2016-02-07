/**
 * @file ignore_list_validity_checker.cpp 
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

#include "prx/planning/modules/validity_checkers/ignore_list_validity_checker.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::ignore_list_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        void ignore_list_validity_checker_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            validity_checker_t::init(reader,template_reader);
            if( parameters::has_attribute("ignore_list", reader, template_reader ) )
            {
                ignore_list = parameters::initialize_from_loader< collision_list_t >("prx_simulation", reader, "ignore_list", template_reader, "ignore_list" );
            }
            else
            {
                PRX_WARN_S("No ignore list initialized in ignore list validity checker.  is_valid will error during execution.");
            }
        }

        bool ignore_list_validity_checker_t::is_valid(const state_t* point)
        {
            collision_list_t* cb = world_model->get_colliding_bodies(point);

            foreach(const collision_pair_t& cp, cb->get_body_pairs())
            {
                if( !(ignore_list->pair_in_list(cp.first, cp.second)) )
                    return false;
            }
            return true;
        }

    }
}