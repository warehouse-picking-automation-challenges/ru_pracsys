/**
 * @file query.cpp
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
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/queries/query.hpp"

namespace prx
{
    using namespace util;
    namespace plan
    {

        void query_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader) { }

        pluginlib::ClassLoader<query_t> query_t::loader("prx_planning", "prx::plan::query_t");

        pluginlib::ClassLoader<query_t>& query_t::get_loader()
        {
            return loader;
        }


    }
}