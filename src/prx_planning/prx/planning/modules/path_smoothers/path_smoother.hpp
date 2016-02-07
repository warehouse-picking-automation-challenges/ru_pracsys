/**
 * @file path_smoother.hpp 
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

#ifndef PRX_SMOOTHER_HPP
#define PRX_SMOOTHER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"

#include "prx/planning/queries/motion_planning_query.hpp"

#include <pluginlib/class_loader.h>

namespace prx 
{ 
    namespace plan 
    {

/**
 * @anchor path_smoother_t
 *
 * This class is responsible for refining the answers produced by other motion
 * planners.  It takes an input query which has been already resolved by a
 * motion planner and attempts to refine the answer to optimize some criterion.
 *
 * @brief <b> Abstract path smoother class for path simplification. </b>
 * 
 * @author Andrew Dobson
 */
class path_smoother_t : public motion_planner_t
{
  public:

    path_smoother_t();

    virtual ~path_smoother_t();

    /**
     * @copydoc motion_planner_t::init()
     *
     * Path smoothers will in general use some subset of the modules needed for
     * regular motion planners, so this method overwrites the strict requirement
     * for all of those modules.
     */
    virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

    /**
     * @copydoc motion_planner_t::link_world_model()
     */
    virtual void link_world_model(world_model_t * const model);
    
    /**
     * @copydoc motion_planner_t::reset()
     */
    virtual void reset();
    
    /**
     * @copydoc motion_planner_t::link_query()
     */
    virtual void link_query(query_t* new_query);
    
    /**
     * @copydoc motion_planner_t::setup()
     */
    virtual void setup();
    
    /**
     * @brief Non-functional resolve query function.
     *
     * @note Path smoothers cannot resolve queries by themselves.
     */
    virtual void resolve_query();
    
    /**
     * @brief Returns whether the smoother has succeeded at some criterion.
     *
     * Some path smoothers seek to optimize certain criteria.  This function is
     * intended to inform task planners when the smoother has achieved such a
     * criterion.
     *
     * @note Default behavior for this function returns false.
     */
    virtual bool succeeded() const;
    
    /**
     * @brief Performs a single step of path smoothing.
     */
    virtual void step() = 0;
       
  protected:
    /** @brief The linked world model that this path smoother operates in. */
    world_model_t* world_model;    
    /** @brief The initial trajectory provided by the query, stored for resetting. */
    sim::trajectory_t* stored_trajectory;
    /** @brief The initial plan provided by the query, stored for resetting. */
    sim::plan_t* stored_plan;

};

    }
}

#endif
