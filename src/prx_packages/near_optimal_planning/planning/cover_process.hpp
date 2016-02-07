/**
 * @file cover_process.hpp
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

#ifndef PRX_COVER_PROC_HPP
#define	PRX_COVER_PROC_HPP

#include "prx/planning/motion_planners/prm_star/prm_star.hpp"

namespace prx
{
    namespace packages
    {
        namespace near_optimal_planning
        {
            class coverage_process_t : public plan::prm_star_t
            {
            public:
                coverage_process_t();
                virtual ~coverage_process_t();
                
                void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                virtual void setup();
                virtual void resolve_query();
                virtual bool succeeded();
                
                friend class coverage_criterion_t;
                
            protected:
                virtual void link_node_to_neighbors(util::directed_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);            
                
                std::vector< sim::state_t* > centers;
                std::vector< sim::state_t* > firsts;
                double ballrad;
                
            };
        }
    }
}

#endif




