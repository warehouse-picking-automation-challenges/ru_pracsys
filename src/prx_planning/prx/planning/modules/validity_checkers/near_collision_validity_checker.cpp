/**
 * @file near_collision_validity_checker.cpp 
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


#include "prx/planning/modules/validity_checkers/near_collision_validity_checker.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::near_collision_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {
        void near_collision_validity_checker_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader) 
        {
            ignore_list_validity_checker_t::init( reader, template_reader );
            clearance = parameters::get_attribute_as< double >("clearance", reader, template_reader );
        }
        
        bool near_collision_validity_checker_t::is_valid(const state_t* point)
        {
            collision_list_t* cb = world_model->get_near_colliding_bodies(point, clearance);
            
            unsigned count =0;
            foreach(const collision_pair_t& cp, cb->get_body_pairs())
            {
                if( !(ignore_list->pair_in_list(cp.first, cp.second)) )
                    return false;
                ++count;
            }
            return true;
        }

    }
}