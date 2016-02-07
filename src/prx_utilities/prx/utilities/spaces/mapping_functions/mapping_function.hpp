/**
 * @file mapping_function.hpp
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

#ifndef PRX_MAPPING_FUNCTION_HPP
#define	PRX_MAPPING_FUNCTION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>

namespace prx 
{ 
    namespace util 
    {
        
        class parameter_reader_t;
        
        /**
         *
         *  @brief <b> This is a class that encapsulates the mapping from one space to another. </b>
         * 
         *  This is a class that encapsulates the mapping from one space to another
         *  
         *  Anybody that implements a mapping function needs to supply the following information
         *  when constructing a mapping function.\n
         *      1) Output space name (the mapped space)\n
         *      2) Domain of the mapping (number of elements to map from)\n
         *      3) Range of the mapping (number of elements to map to)\n
         *      4) Mapping Name\n
         * 
         *  When given to an embedded space, its constructor will populate the following
         *      5) preimage_space (the entire space that is being used to map from) \n
         *      6) image_space (the embedded space that was created with this and other mappings) \n
         *      7) image_interval (the interval in the image space that corresponds to this mapping) \n
         * 
         *  When created from the world model, the world model will give this information
         *      8) preimage_interval (the interval in the space we are mapping from) \n
         * 
         *  If the embedded space or world model are not used, the user must give these objects
         *  to the mapping function explicitly.
         * 
         *  @author Zakary Littlefield
         * 
         */
        class mapping_function_t
        {
        public:
            mapping_function_t()
            {
                domain = 0;
                range = 0;
                image_space = NULL;
                subspace = NULL;
                preimage_space = NULL;
                image_interval = std::make_pair<unsigned,unsigned>(0,0);
                preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
                output_space_name = "";
            }
            virtual ~mapping_function_t()
            {
                delete subspace;
            }
            
            /**
             * Converts the data from the full space to the embedded space.
             * @brief Converts the data from the full space to the embedded space.
             */
            virtual void embed() const = 0;
            /**
             * Converts the data from the embedded space to the full space.
             * @brief Converts the data from the full space to the embedded space.
             */
            virtual void invert() const = 0;
            
            /**
             * Read in parameters to initialize variables.
             * @param reader The primary reader. Any parameters found here will be read.
             * @param template_reader The secondary reader. Any parameters not in the primary reader will be found here.
             */
            virtual void init(const parameter_reader_t* reader, const parameter_reader_t* template_reader=NULL);
            
            /**
             * Get the embedded space that is used to map to.
             * @brief Get the embedded space that is used to map to.
             * @return The embedded space.
             */
            virtual space_t* get_embedded_subspace();
            
            /**
             * @brief Initialize the internal space of this mapping.
             * Initialize the internal space of this mapping.
             */
            virtual void init_spaces();
            
            /**
             * Checks the validity of the parameters that have been set.
             * @brief Checks the validity of the parameters that have been set.
             */
            virtual void verify() const;
            
            /**
             * Get the class loader from pluginlib.
             * @return The pluginlib class loader.
             */
            static pluginlib::ClassLoader<mapping_function_t>& get_loader();
            
            /**
             * @brief The name of the embedded space created.
             */
            std::string output_space_name;
            
            /**
             * @brief Storage for creating the embedded space.
             */
            std::vector<double*> memory;
            
            /**
             * @brief Interval in the embedded space that this mapping function is responsible for.
             */
            std::pair<unsigned,unsigned> image_interval;
            
            /**
             * @brief Interval in the preimage space that this mapping function reads from.
             */
            std::pair<unsigned,unsigned> preimage_interval;
            
            /**
             * @brief Number of elements in the preimage that are used.
             */
            unsigned domain;
            
            /**
             * @brief Number of elements in the image that are modified.
             */
            unsigned range;
            
            /**
             * @brief A pointer to the full space that is being embedded.
             */
            space_t* preimage_space;
            
            /**
             * @brief The space being mapped to.
             */
            space_t* image_space;
            
            /**
             * The name of this mapping.
             */
            std::string mapping_name;
            
        protected:
            
            /**
             * Gets the memory location in the image space.
             * @param i The index in the image space.
             * @return The memory location.
             */
            double& get_image_index(unsigned i) const;
            /**
             * Gets the memory location in the preimage space.
             * @param i The index in the preimage space.
             * @return The memory location.
             */
            double& get_preimage_index(unsigned i) const;
            
            /**
             * @brief The only space memory this mapping function is responsible for.
             */
            space_t* subspace;
            
        private:
            /**
             * @brief The pluginlib loader.
             */
            static pluginlib::ClassLoader<mapping_function_t> loader;
            
            
        };
        
    } 
}


#endif
