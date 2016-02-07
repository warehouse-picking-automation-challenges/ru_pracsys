/**
 * @file shortcut_smoother.hpp 
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

#ifndef PRX_SHORTCUT_SMOOTHER_HPP
#define PRX_SHORTCUT_SMOOTHER_HPP

#include "prx/planning/modules/path_smoothers/path_smoother.hpp"
#include "prx/utilities/definitions/random.hpp"

namespace prx 
{ 
    namespace plan 
    {

/**
 * @anchor shortcut_smoother_t
 *
 * One of the simplest forms of smoothing for rigid bodies is to simply attempt
 * new connections between random points along a trajectory, which this smoother
 * performs.
 *
 * @brief <b> Path smoother using simple shortcuts for rigid bodies. </b>
 *
 * @author Andrew Dobson
 */
class shortcut_smoother_t : public path_smoother_t
{
public:
    shortcut_smoother_t() : path_smoother_t() {}
    /**
     * @brief Parameterized constructor for the shortcut smoother class.
     *
     * @param inlp Input local planner for the smoother to use.
     * @param invc Input validity checker for the smoother to use.
     */
    shortcut_smoother_t( local_planner_t* inlp, validity_checker_t* invc, double inres );
    
    virtual ~shortcut_smoother_t(){}

    /**
     * @copydoc path_smoother_t::init()
     */
    virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
    /**
     * @copydoc path_smoother_t::step()
     */
    void step();
    
protected:
    /** @brief Local planner resolution to use with this path smoother. */
    double resolution;
};

    }
}

#endif


