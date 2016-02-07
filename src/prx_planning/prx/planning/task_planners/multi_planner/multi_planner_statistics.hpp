/**
 * @file multi_planner_statistics.hpp
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

#ifndef PRX_MULTI_PLANNER_STATISTICS_HPP
#define	PRX_MULTI_PLANNER_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx 
{ 
    namespace plan 
    {

/**
 * @anchor multi_planner_statistics_t
 *
 * This class gathers statistics for the \ref multi_planner_t task planner.
 *
 * @brief <b> Multi-planner task planner statistics class. </b>
 *
 * @author Zakary Littlefield
 */
class multi_planner_statistics_t : public util::statistics_t
{

  public:
    multi_planner_statistics_t();
    virtual ~multi_planner_statistics_t();

    /**
     * @copydoc util::statistics_t::get_statistics()
     */
    virtual std::string get_statistics() const;
    
    /**
     * @brief Outputs statistics to the output_directory.
     *
     * Iterates over the known planner statistics, and for each one that passes
     * the output check, creates an output file at the output_directory and 
     * dumps the planner's statistics.
     */
    void output_stats();

    /** @brief A string indicating the directory to which statistics should be output. */
    std::string output_directory;
    /** @brief A map of individual planner statistics classes. */
    util::hash_t<std::string,std::vector<const util::statistics_t*> > planner_statistics;
    /** @brief A map indicating which of the planner's statistics should be output. */
    util::hash_t<std::string,bool> output_check;
};

    }
}

#endif
