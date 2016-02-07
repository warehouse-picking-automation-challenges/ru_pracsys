/**
 * @file replanning_query.hpp
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

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"


namespace prx 
{ 
    namespace plan 
    {

/**
 * @anchor replanning_query_t
 *
 * Task planner query which is used by \ref replanning_planner_t class.  This
 * query is a tag class which simply inherits from the \ref motion_planning_query_t.
 *
 * @brief <b> Task planner query used for replanning. </b>
 *
 * @author Andrew Kimmel
 */
class replanning_query_t : public motion_planning_query_t
{
};
        
    }
}