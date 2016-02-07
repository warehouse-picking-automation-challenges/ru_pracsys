/**
 * @file test_controller.cpp 
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

#include "test_controller.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS( prx::packages::tutorial::test_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        namespace tutorial
        {
    
            test_controller_t::test_controller_t()
            {
                _last_direction = 0;
                state_memory = boost::assign::list_of( &_last_direction );    
                controller_state_space = new space_t("R", state_memory );

                _control_param = 0;
                control_memory = boost::assign::list_of( &_control_param );   
                input_control_space = new space_t("R",control_memory);

            }

            test_controller_t::~test_controller_t()
            {
            }

             void test_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                controller_t::init(reader, template_reader);    
            }

            void test_controller_t::propagate(const double simulation_step)
            {    
                _last_direction += _control_param;
                controller_t::propagate(simulation_step);
            }


            void test_controller_t::verify() const
            {
                PRX_ASSERT( controller_state_space->get_dimension() == 1 );
                PRX_ASSERT( input_control_space->get_dimension() == 1 );
                controller_t::verify();
            }

            void test_controller_t::compute_control()
            {   
                //move in the correct direction
                computed_control->at(0) = 10;
                computed_control->at(1) = _last_direction + _control_param;
                output_control_space->copy_from_point(computed_control);
                subsystems.begin()->second->compute_control();
            }
        }
    }
}
