/**
 * @file converter.hpp
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

#ifndef PRX_CONVERTER_HPP
#define	PRX_CONVERTER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"
#include "prx/simulation/systems/controllers/controller.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Converter controller.  By default provides no new state variables, but transforms the control space
         * into another control space using \ref util::mapping_function_t.
         * 
         * @brief <b> Converts a control space to another control space. </b>
         * 
         * @author Zakary Littlefield
         */
        class converter_t : public controller_t
        {

          public:
            converter_t();
            virtual ~converter_t();
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);
            virtual void construct_spaces();
            virtual void compute_control();

          protected:
            util::mapping_function_t* mapping;

        };

    }
}

#endif
