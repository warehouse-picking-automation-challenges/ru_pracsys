/**
 * @file planar_to_linear.hpp 
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

#ifndef PRX_PLANAR_TO_LINEAR_HPP
#define	PRX_PLANAR_TO_LINEAR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        /**
         * @brief <b> Embeds an ODE box space into R3 </b>
         * @author Zakary Littlefield
         */
        class planar_to_linear_t : public mapping_function_t
        {
        public:
            planar_to_linear_t() 
            {
                domain = 5;
                range = 4;
                image_space = NULL;
                subspace = NULL;
                preimage_space = NULL;
                image_interval = std::make_pair<unsigned,unsigned>(0,0);
                preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
                output_space_name = "X|Rd|Rd|Rd";
                mapping_name = "planar_to_linear";
            }
            virtual ~planar_to_linear_t() {}    
            
            /**
             * @copydoc mapping_function_t::init_spaces()
             */
            void init_spaces();
            /**
             * @copydoc mapping_function_t::embed() const
             */
            virtual void embed() const;
            /**
             * @copydoc mapping_function_t::invert() const
             */
            virtual void invert() const;
            
        protected:
            
        };
        
    } 
}


#endif