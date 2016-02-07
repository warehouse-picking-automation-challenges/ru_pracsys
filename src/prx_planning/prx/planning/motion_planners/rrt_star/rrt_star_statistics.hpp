/**
 * @file rrt_star_statistics.hpp
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

#ifndef PRX_RRT_STAR_STATISTICS_HPP
#define	PRX_RRT_STAR_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/rrt/rrt_statistics.hpp"

namespace prx 
{ 
    namespace plan 
    {


/**
 * @brief <b> A statistics class the provides information about the RRT* motion planner.</b>
 * 
 * A statistics class the provides information about the RRT* motion planner.
 * 
 * @author Zakary Littlefield
 */
class rrt_star_statistics_t : public rrt_statistics_t
{

  public:
    rrt_star_statistics_t();
    virtual ~rrt_star_statistics_t();
    
    /**
     * @copydoc util::statistics_t::get_statistics() const 
     */
    virtual std::string get_statistics() const;
    
    /**
     * @brief The average cost over all nodes.
     */
    double average_cost;
};

    }
}

#endif
