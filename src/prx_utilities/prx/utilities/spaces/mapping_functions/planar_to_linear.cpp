/**
 * @file planar_to_linear.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/mapping_functions/planar_to_linear.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::util::planar_to_linear_t, prx::util::mapping_function_t)

namespace prx
{
    namespace util
    {       

        void planar_to_linear_t::embed() const
        {
            unsigned start_index = preimage_interval.first;
            unsigned image_start_index = image_interval.first;
            get_image_index(image_start_index) = get_preimage_index(start_index);
            get_image_index(image_start_index + 1) = get_preimage_index(start_index + 2);
            get_image_index(image_start_index + 2) = get_preimage_index(start_index + 3);
            get_image_index(image_start_index + 3) = get_preimage_index(start_index + 4);
        }

        void planar_to_linear_t::invert() const
        {
            unsigned start_index = preimage_interval.first;
            unsigned image_start_index = image_interval.first;
            get_preimage_index(start_index) = get_image_index(image_start_index);
            get_preimage_index(start_index + 1) = 0;
            get_preimage_index(start_index + 2) = get_image_index(image_start_index + 1);
            get_preimage_index(start_index + 3) = get_image_index(image_start_index + 2);
            get_preimage_index(start_index + 4) = get_image_index(image_start_index + 3);
        }

        void planar_to_linear_t::init_spaces()
        {
            //subspace and preimage should be set
            if( subspace == NULL || preimage_space == NULL )
                PRX_FATAL_S("PLANAR TO LINEAR NOT HAPPY");
            //make the image space have the correct bounds information
            *subspace->get_bounds()[0] = *preimage_space->get_bounds()[0];
            *subspace->get_bounds()[1] = *preimage_space->get_bounds()[2];
            *subspace->get_bounds()[2] = *preimage_space->get_bounds()[3];
            *subspace->get_bounds()[3] = *preimage_space->get_bounds()[4];
        }

    }
}