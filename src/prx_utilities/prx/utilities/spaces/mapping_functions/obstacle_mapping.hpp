/**
 * @file obstacle_mapping.hpp 
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
#pragma once

#ifndef PRX_OBSTACLE_MAPPING_HPP
#define	PRX_OBSTACLE_MAPPING_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/hide_mapping.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        /**
         * @brief <b> Same as \ref hide_mapping_t, just renamed. </b>
         * @author Zakary Littlefield
         */
        class obstacle_mapping_t : public hide_mapping_t
        {
        public:
            obstacle_mapping_t();
            virtual ~obstacle_mapping_t();
            
        };
        
    } 
}


#endif