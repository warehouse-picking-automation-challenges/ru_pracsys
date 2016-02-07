/**
 * @file config_sensor.hpp 
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

#ifndef PRX_CONFIG_SENSOR_HPP
#define	PRX_CONFIG_SENSOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/simulation/sensing/sensor.hpp"
#include "prx/simulation/systems/system.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }
    namespace sim
    {
        
        class plant_t;
        class obstacle_t;

        /**
         * @author Andrew Kimmel
         */
        class config_sensor_t : public sensor_t
        {
        protected:
            util::config_list_t plant_configs;
            util::config_list_t obstacle_configs;
            
            std::vector<plant_t*> sensed_plants;
            std::vector<obstacle_t*> sensed_obstacles;
            
            /** @brief Used to determine if obstacle configs need to be updated */
            bool update_obstacle_configs;

        public:

            config_sensor_t();
            virtual ~config_sensor_t();
            
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
            /**
             * @brief Sensors extract necessary information from the simulator, and self-initialize
             * @param sim A pointer to the simulator
             */
            virtual void initialize_sensor(simulator_t* sim);
            
            virtual void update_data(); // updates the internal sensing data (communication)
            
            virtual void set_update_obstacles(bool update_flag);

            /**
             * @brief Concatenates plant and obstacles configs into one list
             * @return 
             */
            virtual util::config_list_t get_configs();
            
            /**
             * @brief Returns the config list of plants specifically
             * @return 
             */
            virtual util::config_list_t get_plant_configs();
            
            /**
             * @brief Returns the config list of obstacles specifically
             * @return 
             */
            virtual util::config_list_t get_obstacle_configs();
            


            
        };

    }
}

#endif

