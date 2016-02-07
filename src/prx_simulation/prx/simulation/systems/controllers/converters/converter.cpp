/**
 * @file converter.cpp
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

#include "prx/simulation/systems/controllers/converters/converter.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::converter_t, prx::sim::system_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        converter_t::converter_t() { }

        converter_t::~converter_t() { }

        void converter_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            mapping = parameters::create_from_loader<mapping_function_t>("prx_utilities", reader, "mapping", template_reader, "mapping");
            controller_t::init(reader, template_reader);

        }

        void converter_t::construct_spaces()
        {
            controller_t::construct_spaces();
            //need to give the preimage_space, the image space, the image interval and the preimage interval
            mapping->preimage_space = output_control_space;
            mapping->preimage_interval = std::pair<unsigned, unsigned>(0, output_control_space->get_dimension());
            mapping->init_spaces();
            mapping->image_space = mapping->get_embedded_subspace();
            input_control_space = mapping->get_embedded_subspace();
            mapping->image_interval = std::pair<unsigned, unsigned>(0, input_control_space->get_dimension());
        }

        void converter_t::compute_control()
        {
            if( active )
            {
                mapping->invert();
            }
            controller_t::compute_control();
        }


    }
}
