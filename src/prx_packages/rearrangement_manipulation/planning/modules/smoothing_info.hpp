/**
 * @file smoothing_info.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_SMOOTHING_INFO_HPP
#define	PRX_SMOOTHING_INFO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/plan.hpp"

namespace prx
{
    namespace sim
    {
        class plan_t;
        class trajectory_t;
    }

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * A class that holds the info for the final path. How is constructed 
             */
            class smoothing_info_t
            {

              public:
                smoothing_info_t();
                smoothing_info_t(const util::space_t* control_space);
                virtual ~smoothing_info_t();
                
                bool is_constrained_by(unsigned pose);
                
                std::string print() const;

                unsigned object_id;
                unsigned from_pose;
                unsigned to_pose;
                std::set<unsigned> constraints;
                sim::plan_t plan;
                unsigned reaching_point;
                unsigned retracting_point;
                
            };
        }
    }
}

#endif
