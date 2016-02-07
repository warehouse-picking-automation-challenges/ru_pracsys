/**
 * @file mapping_function.cpp
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"

#include <pluginlib/class_loader.h>

namespace prx 
{ 
    namespace util 
    {
        
        pluginlib::ClassLoader<mapping_function_t> mapping_function_t::loader("prx_utilities", "prx::util::mapping_function_t");
        
        pluginlib::ClassLoader<mapping_function_t>& mapping_function_t::get_loader()
        {
            return loader;
        }
        
        void mapping_function_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            mapping_name = parameters::get_attribute_as<std::string>("type",reader,template_reader);
            
            for(unsigned i=0;i<range;i++)
            {
                memory.push_back(new double);
            }
            if(range!=0)
            {
                subspace = new space_t(output_space_name,memory);
                if(template_reader!=NULL)
                    subspace->init(template_reader);
                subspace->init(reader);
            }
            
        }
        
        space_t* mapping_function_t::get_embedded_subspace()
        {
            return subspace;
        }
        
        void mapping_function_t::verify() const
        {
            if(domain!=0 && domain != preimage_interval.second-preimage_interval.first-1)
            {
                PRX_FATAL_S("The domain of a mapping doesn't match the interval");
            }
            if(range!=0 && range != image_interval.second-image_interval.first-1)
            {
                PRX_FATAL_S("The range of a mapping doesn't match the interval");
            }
        }
        void mapping_function_t::init_spaces()
        {
            if(subspace==NULL)
            {
                for(unsigned i=0;i<range;i++)
                {
                    memory.push_back(new double);
                }
                if(range!=0)
                {
                    subspace = new space_t(output_space_name,memory);
                    subspace->set_default_scale();
                    subspace->set_default_bounds();
                }
            }
        }
        
        
        double& mapping_function_t::get_image_index(unsigned index) const
        {
            return *(image_space->addresses[index]);
        }
        
        double& mapping_function_t::get_preimage_index(unsigned index) const
        {
            return *((preimage_space->addresses)[index]);
        }
    } 
}
