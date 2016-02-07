/**
 * @file system_ptr.hpp 
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

#ifndef SYSTEM_PTR_HPP
#define	SYSTEM_PTR_HPP

#include "prx/utilities/definitions/defs.hpp"

namespace prx
{
    namespace sim
    {

        class system_t;

        /** A smart pointer for the systems*/
        typedef boost::shared_ptr<system_t> system_ptr_t;


    }
}

#endif	/* SYSTEM_PTR_HPP */

