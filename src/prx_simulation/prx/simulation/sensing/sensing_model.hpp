/**
 * @file sensing_model.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_SENSING_MODEL_HPP
#define	PRX_SENSING_MODEL_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/spaces/space.hpp"



#include "prx/simulation/state.hpp"    
#include "prx/simulation/sensing/sensing_info.hpp"
#include "prx/simulation/systems/system.hpp"

namespace prx
{
    namespace sim
    {

        class plant_t;
        
        /**
         * @author Andrew Kimmel
         */
        class sensing_model_t
        {
        protected:
            /** @brief Contains the list of all sensing info to be altered by this sensing_t*/
            std::vector<sensing_info_t*> all_sensing_info;
            /** @brief Contains the list of all sensors owned by this sensing_t*/
            std::vector<sensor_t*> all_sensors;
            /** @brief Maps the source name to a specific sensor */
            util::hash_t<std::string, sensor_t*> sources_to_sensors;

            // Sensed state (updated by application)
            /** @brief */
            sim::state_t* sensed_state;

            // State space (linked by application)
            /** @brief */
            const util::space_t* state_space;

        public:

            sensing_model_t();
            virtual ~sensing_model_t();

            /**
             * Initializes from the given parameters.
             * 
             * @brief Initializes from the given parameters.
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
             * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
             * is not specified in the \c reader then it will be read from the \c template_reader 
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            virtual void initialize_sensors(sim::simulator_t* sim);
            /**
             *
             */
            virtual void update_sensed_state( const sim::state_t* new_state);
            /**
             *
             */
            virtual void sense(double time_step);
            /**
             *
             */
            virtual void set_sensing_info( const std::vector<sensing_info_t*>& sensing_info);
            
            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<sensing_model_t>& get_loader();
            
          private:
            /** 
             * The pluginlib loader for the \ref sensing_t class.
             * 
             * @brief The pluginlib loader for the \ref sensing_t class.
             */
            static pluginlib::ClassLoader<sensing_model_t> loader;

        };
    }
}

#endif

