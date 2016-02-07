/**
 * @file sampler.hpp 
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


#ifndef PRX_SAMPLER_HPP
#define PRX_SAMPLER_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    
    namespace util
    {
        class space_t;
        class space_point_t;
        class parameter_reader_t;
        class bounds_t;
    }
   
    
    namespace plan
    {
        /**
         * Encapsulates functionality for performing sampling of space_points. Can be 
         * overwritten for specific sampling functionality.
         * @brief <b> Encapsulates functionality for performing sampling of space_points </b>
         * @author Zakary Littlefield
         */
        class sampler_t
        {

          public:

            sampler_t(){ }

            virtual ~sampler_t(){ }

            /**
             * Initializes any internal variables of the sampler.
             * @param reader The primary reader. Any parameters found here will initialize variables.
             * @param template_reader The secondary reader. Any parameters found here will only initialize variables if not found in \c reader
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL){ }

            /**
             * Uses a space to sample a space point.
             * @brief Uses a space to sample a space point.
             * @param space The space to sample in.
             * @param point The point that will be assigned the random point.
             */
            virtual void sample(const util::space_t* space, util::space_point_t* point) = 0;

            /**
             * Using a space and a set of bounds, sample within those bounds around another point.
             * @brief Using a space and a set of bounds, sample within those bounds around another point.
             * @param space The space to sample in.
             * @param near_point The point to sample around.
             * @param bounds The bounds around that point to sample
             * @param point The point that will be assigned the random point.
             */
            virtual void sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point) = 0;

            /**
             * Sample a control that is dependent on state values.
             * @brief Sample a control that is dependent on state values.
             * @param space The space for the control.
             * @param state The state used to determine controls.
             * @param control The control that will be assigned the random point.
             */
            virtual void bounded_sample(const util::space_t* space, util::space_point_t* state, util::space_point_t* control);

            /**
             * @brief Pluginlib class loader
             * @return The loader.
             */
            static pluginlib::ClassLoader<sampler_t>& get_loader();

          private:
            /**
             * @brief Pluginlib's class loader.
             */
            static pluginlib::ClassLoader<sampler_t> loader;

        };

    }
}

#endif