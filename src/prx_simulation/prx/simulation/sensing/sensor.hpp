/**
 * @file sensor.hpp 
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

#ifndef PRX_SENSOR_HPP
#define	PRX_SENSOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }
    namespace sim
    {
        class simulator_t;

        /**
         * @author Andrew Kimmel
         */
        class sensor_t
        {
        protected:
            std::string source; // the name of the sensor used to update this sensing data
            double sensor_delay; // the amount of time between the sensor updating its data. -1 means sensor never fires
            double trigger_time; // counts down how long before the sensor can fire. Reset sets it to start at sensor delay

        public:

            sensor_t();
            virtual ~sensor_t();
            
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
            virtual void initialize_sensor(simulator_t* sim) = 0;
            
            virtual void update_data() = 0; // updates the internal sensing data (communication)
            
            // Checks if the sensor is ready to update data. Otherwise, decrements trigger time by the time step
            virtual bool fires(double time_step);
            
            // Resets trigger time to be sensor_delay
            virtual void reset_delay();
            
            virtual void set_periodic_sensing(bool flag);
            
            std::string get_source();
            double get_sensor_delay();
            
            
            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<sensor_t>& get_loader();
            
          private:
            /** 
             * The pluginlib loader for the \ref sensor_t class.
             * 
             * @brief The pluginlib loader for the \ref sensor_t class.
             */
            static pluginlib::ClassLoader<sensor_t> loader;
            
            /**
             * A flag indicating whether this sensor should be sensing at regular intervals.
             * 
             * @brief Periodic sensing flag.
             */
            bool periodic_sensing;
        };

    }
}

#endif

