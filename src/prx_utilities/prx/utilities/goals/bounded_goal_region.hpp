/**
 * @file bounded_goal_region.hpp
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

#ifndef PRX_BOUNDED_GOAL_REGION_HPP
#define	PRX_BOUNDED_GOAL_REGION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"

namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /**
         * A goal that is represented by a bounded region.
         * @brief <b> A goal that is represented by a bounded region. </b>
         * @author Zakary Littlefield
         */
        class bounded_goal_region_t : public goal_t
        {

          public:
            bounded_goal_region_t();
            virtual ~bounded_goal_region_t();

            /** @copydoc goal_t::init(const parameter_reader_t*, const parameter_reader_t*) */
            virtual void init(const parameter_reader_t* reader, const parameter_reader_t* template_reader);

            /** @copydoc goal_t::link_space( const space_t* ) */
            virtual void link_space(const space_t* inspace);

            /** @copydoc goal_t::satisfied(const space_point_t* ) */
            virtual bool satisfied(const space_point_t* state);

            /** @copydoc goal_t::satisfied(const space_point_t* , double* ) */
            virtual bool satisfied(const space_point_t* state, double* distance);

            /**
             * @brief The bounds for the successful region.
             */
            std::vector<bounds_t*> bounds;

          protected:

            /**
             * @brief The space point representing the center of the region.
             */
            space_point_t* point;
        };

    }
}

#endif	// PRX_RADIAL_GOAL_REGION_HPP
