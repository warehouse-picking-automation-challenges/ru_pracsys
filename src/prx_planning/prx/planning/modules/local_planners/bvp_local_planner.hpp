/**
 * @file bvp_local_planner.hpp
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

#ifndef PRX_BVP_LOCAL_PLANNER_HPP
#define	PRX_BVP_LOCAL_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

namespace prx 
{ 
    namespace plan 
    {

/**
 * @anchor bvp_local_planner_t
 *
 * This class exactly connects samples within the state space.  It does so by
 * employing solutions to the boundary value problem, which must be provided
 * by the underlying world model.
 *
 * @brief <b> Local planner which solves the boundary value problem. </b>
 *
 * @author Zakary Littlefield, Andrew Kimmel
 */
class bvp_local_planner_t : public local_planner_t
{
  public:
    bvp_local_planner_t();
    virtual ~bvp_local_planner_t();

    /**
     * @copydoc local_planner_t::init()
     */
    virtual void init(const util::parameter_reader_t * const reader,const util::parameter_reader_t* const template_reader=NULL);
    /**
     * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::trajectory_t&, bool connect )
     */
    virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan,sim::trajectory_t& traj, bool connect = true);  
    /**
     * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::state_t*, bool connect )
     */
    virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan,sim::state_t* result, bool connect = true);
  protected:

    double max_prop_length;
    
};

    }
}

#endif	

