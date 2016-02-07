/**
 * @file uniform_sampler.cpp 
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


#include "prx/planning/modules/samplers/uniform_sampler.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::uniform_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    namespace plan
    {        

        void uniform_sampler_t::sample(const space_t* space, space_point_t* point)
        {
            space->uniform_sample(point);
        }

        void uniform_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& bounds, space_point_t* point)
        {
            space->uniform_sample_near(point, near_point, bounds);
        }

    }
}