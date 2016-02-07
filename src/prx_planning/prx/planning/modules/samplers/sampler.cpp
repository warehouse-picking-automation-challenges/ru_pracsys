/**
 * @file sampler.cpp 
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

#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx
{
    using namespace util;
    namespace plan
    {

        pluginlib::ClassLoader<sampler_t> sampler_t::loader("prx_planning", "prx::plan::sampler_t");

        void sampler_t::bounded_sample(const space_t* space, space_point_t* state, space_point_t* control) {
            //need to get control bounds based on state
            //bounds_t bounds;
            //bounds = world_model->get_control_bounds(state);
            //sample from those bounds
        }

        pluginlib::ClassLoader<sampler_t>& sampler_t::get_loader()
        {
            return loader;
        }

    }
}

