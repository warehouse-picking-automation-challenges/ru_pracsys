///**
// * @file multi_planner.hpp 
// * 
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// * 
// * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
// * 
// * Email: pracsys@googlegroups.com
// */
//#pragma once
//
//#ifndef PRX_MULTI_PLANNER_HPP
//#define	PRX_MULTI_PLANNER_HPP
//
//#include "prx/utilities/definitions/defs.hpp"
//#include "prx/utilities/parameters/parameter_reader.hpp"
//#include "prx/planning/task_planners/task_planner.hpp"
//#include "prx/planning/task_planners/multi_planner/multi_planner_statistics.hpp"
//
//namespace prx 
//{ 
//    namespace plan 
//    {
//
//class criterion_t;
//
///**
// * @anchor multi_planner_t
// * 
// * A task planner which runs multiple planners in succession.  The primary purpose
// * of this task planner is to benchmark different algorithms.
// * 
// * @brief <b> Runs multiple planners in succession. </b>
// *
// * @author Zakary Littlefield
// */
//class multi_planner_t : public task_planner_t
//{    
//public:
//    multi_planner_t();
//    virtual ~multi_planner_t();
//    
//    /**
//     * @copydoc task_planner_t::init()
//     */
//    virtual void init(const util::parameter_reader_t* reader,const util::parameter_reader_t* template_reader);
//    
//    /**
//     * @copydoc task_planner_t::setup()
//     */
//    virtual void setup();
//    
//    /**
//     * @brief Top-level function for executing the multi-planner.
//     *
//     * This function sets the planners running, attempting to solve given
//     * queries.  This task planner runs planners sequentially.
//     *
//     * @return A flag indicating success of achieving the planner's goal.
//     */
//    virtual bool execute();
//        
//    /**
//     * @copydoc task_planner_t::get_statistics()
//     */
//    virtual const util::statistics_t* get_statistics();
//    
//    /**
//     * @copydoc task_planner_t::succeeded()
//     */
//    virtual bool succeeded() const;
//    
//    /**
//     * @copydoc task_planner_t::link_query()
//     */
//    virtual void link_query(query_t* in_query);
//    
//    /**
//     * @copydoc task_planner_t::resolve_query()
//     */
//    virtual void resolve_query();
//    
//protected:
//    /**
//     * @copydoc task_planner_t::update_vis_info()
//     */
//    virtual void update_vis_info() const;
//    
//    /** @brief The statistics class for the multi-planner task planner. */
//    multi_planner_statistics_t* stats;
//        
//    /** @brief Individual criteria used for statistics gathering. */
//    std::vector<criterion_t*> stats_criteria;
//};
//
//    }
//}
//
//#endif
//
