/**
 * @file test_controller.hpp 
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
#ifndef PRX_TEST_CONTROLLER
#define PRX_TEST_CONTROLLER

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"

namespace prx
{
    namespace packages
    {
        namespace tutorial
        {
    
            class test_controller_t : public sim::simple_controller_t
            {

              public:
                test_controller_t();
                virtual ~test_controller_t();
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                virtual void propagate(const double simulation_step = 0);

                virtual void verify() const;

                virtual void compute_control();

              protected:

                double _last_direction;
                double _control_param;

            };

        }
    }
}




#endif