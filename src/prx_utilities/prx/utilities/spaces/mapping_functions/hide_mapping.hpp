/**
 * @file hide_mapping.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors PRACSYS
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_HIDE_MAPPING_HPP
#define	PRX_HIDE_MAPPING_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        /**
         * @brief <b> A mapping to hide systems in the world model. </b>
         * @author Zakary Littlefield
         */
        class hide_mapping_t : public mapping_function_t
        {
        public:
            hide_mapping_t() 
            {
                domain = 0;
                range = 0;
                image_space = NULL;
                subspace = NULL;
                preimage_space = NULL;
                image_interval = std::make_pair<unsigned,unsigned>(0,0);
                preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
                output_space_name = "";
                mapping_name = "hide_mapping";
            }
            virtual ~hide_mapping_t() {}
            
            /**
             * @copydoc mapping_function_t::embed() const
             */
            virtual void embed() const;
            
            /**
             * @copydoc mapping_function_t::invert() const
             */
            virtual void invert() const;
            
        };
        
    } 
}


#endif