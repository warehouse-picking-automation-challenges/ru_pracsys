/**
 * @file full_mapping.cpp 
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
#include "prx/utilities/spaces/mapping_functions/full_mapping.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::util::full_mapping_t, prx::util::mapping_function_t)

namespace prx
{
    namespace util
    {        

        void full_mapping_t::embed() const
        {
            unsigned pre_iter = preimage_interval.first, img_iter = image_interval.first;
            while (pre_iter < preimage_interval.second && img_iter < image_interval.second)
            {
                get_image_index(img_iter) = get_preimage_index(pre_iter);
                pre_iter++; img_iter++;
            }
//            for( unsigned pre_iter = preimage_interval.first, img_iter = image_interval.first;
//                 pre_iter < preimage_interval.second, img_iter < image_interval.second;
//                 pre_iter++, img_iter++ )
//            {
//                get_image_index(img_iter) = get_preimage_index(pre_iter);
//            }
        }

        void full_mapping_t::invert() const
        {
            
            unsigned pre_iter = preimage_interval.first, img_iter = image_interval.first;
            while (pre_iter < preimage_interval.second && img_iter < image_interval.second)
            {
                get_preimage_index(pre_iter) = get_image_index(img_iter);
                pre_iter++; img_iter++;
            }
            
//            for( unsigned pre_iter = preimage_interval.first, img_iter = image_interval.first;
//                 pre_iter < preimage_interval.second, img_iter < image_interval.second;
//                 pre_iter++, img_iter++ )
//            {
//                get_preimage_index(pre_iter) = get_image_index(img_iter);
//            }
        }


    }
}