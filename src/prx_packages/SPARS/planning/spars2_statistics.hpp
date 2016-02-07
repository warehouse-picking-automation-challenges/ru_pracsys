/**
 * @file spars2_statistics.hpp
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

#ifndef PRX_SPARS2_STATISTICS_HPP
#define	PRX_SPARS2_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"

namespace prx 
{ 
    namespace packages
    {
        namespace spars
        {

/**
 * @anchor spars2_statistics_t
 *
 * This class gathers statistics for the \ref spars2_t sampling-based motion planner.
 * It gathers information on the size of computed roadmaps.
 *
 * @brief <b> SPARS2 statistics class. </b>
 *
 * @author Andrew Dobson
 */
class spars2_statistics_t : public plan::prm_star_statistics_t
{

  public:
    spars2_statistics_t();
    virtual ~spars2_statistics_t();

    /**
     * @copydoc util::statistics_t::get_statistics() 
     */
    virtual std::string get_statistics() const;
    
    /** @copydoc util::statistics_t::serialize(std::ofstream ) const */
    virtual bool serialize(std::ofstream& stream) const;

    /** @brief The recorded number of coverage vertices added. */
    double num_coverage;
    /** @brief The recorded number of coverage vertices added. */
    double num_connectivity;
    /** @brief The recorded number of coverage vertices added. */
    double num_interface;
    /** @brief The recorded number of coverage vertices added. */
    double num_quality;
};
        }
    }
}

#endif
