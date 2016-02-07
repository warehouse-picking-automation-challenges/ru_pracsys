/**
 * @file integration_plant.cpp 
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

#include "prx/simulation/systems/plants/integration_plant.hpp"
#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>

namespace prx
{
    using namespace util;
    namespace sim
    {

        integration_plant_t::integration_plant_t() : derivative_function(boost::bind(&integration_plant_t::update_derivative, this, _1))
        {
            integrator = NULL;
        }

        integration_plant_t::~integration_plant_t()
        {
            delete integrator;
        }

        void integration_plant_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            plant_t::init(reader, template_reader);
            //    integrator = parameters::initialize_from_loader< integrator_t > ("integrator", "prx_simulation", "integrator_t",reader,template_reader);
            if( template_reader != NULL )
            {
                PRX_DEBUG_COLOR("Template integrator", PRX_TEXT_CYAN);
                integrator = template_reader->initialize_from_loader< integrator_t > ("integrator", "prx_simulation");
            }
            else
            {
                PRX_DEBUG_COLOR("Normal integrator", PRX_TEXT_CYAN);
                integrator = reader->initialize_from_loader< integrator_t > ("integrator", "prx_simulation");
            }
            integrator->init_space(state_space);
        }

        void integration_plant_t::propagate(const double simulation_step)
        {
            integrator->integrate(derivative_function, simulation_step);
        }

        void integration_plant_t::verify() const
        {
            if( integrator == NULL )
                throw std::runtime_error("An integration plant is without an integrator!");

            try
            {
                //        PRX_DEBUG_S ("Going to try integrator verify with ptr: " << integrator);
                integrator->verify();
            }
            catch( integrator_t::invalid_integrator_exception e )
            {
                throw invalid_system_exception("The system " + pathname + " does not have correct initialized the integrator.\n" + e.what());
            }


            plant_t::verify();
        }

    }
}

