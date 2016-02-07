/**
 * @file gradient_descent_local_planner.hpp
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

#ifndef PRX_GRADIENT_DESCENT_LOCAL_PLANNER_HPP
#define	PRX_GRADIENT_DESCENT_LOCAL_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

namespace prx 
{ 
    namespace plan 
    {

/**
 * @anchor gradient_descent_local_planner_t
 *
 * For some systems, we can define a function we wish to optimize.  The gradient
 * descent local planner seeks to follow a given gradient to compute local paths.
 *
 * @brief <b> Performs local planning by using a gradient. </b>
 *
 * @author Zakary Littlefield
 */
class gradient_descent_local_planner_t : public local_planner_t
{
  public:
    gradient_descent_local_planner_t(): local_planner_t(){}
    
    /**
     * @copydoc local_planner_t::init()
     */
    virtual void init(const util::parameter_reader_t * const reader,const util::parameter_reader_t* const template_reader=NULL);
    
    virtual ~gradient_descent_local_planner_t(){}
    /**
     * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::trajectory_t&, bool connect )
     */
    virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan,sim::trajectory_t& traj, bool connect = true);  
    /**
     * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::state_t* , bool connect)
     */
    virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan,sim::state_t* result, bool connect = true);
    
  protected:
    
    /**
     * @brief Gradient-aware steering function.
     *
     * @param start State from which to begin propagating.
     * @param goal Desired goal state to propagate to.
     * @param plan Resulting plan returned by the steering function.
     */
    virtual void gradient_steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan);
    
    /** @brief Storage for the end state. */
    sim::state_t* end_state;
    
    /** @brief Attempts taken at reaching the goal state. */
    unsigned attempts;
    /** @brief Max number of simulation steps allowed for any one local plan. */
    unsigned max_multiple;
    /** @brief Threshold for trajectory acceptance. */
    double accepted_radius;
    /** @brief Rate at which to follow the gradient. */
    double learning_rate;
    /** @brief Produced trajectory from the local planning. */
    sim::trajectory_t traj;    
};

    }
}

#endif
