/**
 * @file sensing_info.hpp 
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

#ifndef PRX_SENSING_INFO_HPP
#define	PRX_SENSING_INFO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/sensing/sensor.hpp"

namespace prx
{
    
    namespace util
    {
        class parameter_reader_t;
    }
    namespace sim
    {

        /**
         * @author Andrew Kimmel
         */
        class sensing_info_t
        {
        protected:
            std::vector<sensor_t*> linked_sensors; // linked from sensing
            std::vector<std::string> sensor_sources; // the source sensors, set from input
            double update_delay; // Determines how often update info is called
            double trigger_time; // Determines how much time is left before update info can be called
            
            bool* linked_active_flag; // The controller that owns this sensing info, links its active flag

        public:

            sensing_info_t();
            virtual ~sensing_info_t();

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
            
            virtual void link_active(bool* active_flag);
            virtual bool is_active();
            
            // Typically set by sensing_t. This links in the actual sensors used by this sensing_info_t
            virtual void set_sensors(const std::vector<sensor_t*>& sensors);
            
            // Returns a vector of the sensor sources, which were set at input time
            virtual std::vector<std::string> get_sensor_sources();
            
            // The conversion function to turn sensing data into the controller's info
            virtual void update_info();
            // Used to access the info?
            virtual void get_info();
            
            virtual bool updates(double time_step);
            virtual void reset_delay();
            
            //Functions to do non-timed sensing
            virtual void set_periodic_sensing(std::string sensor_name, bool flag);
            virtual void forced_sense(std::string sensor_name);
            
            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<sensing_info_t>& get_loader();
            
          private:
            /** 
             * The pluginlib loader for the \ref sensing_info_t class.
             * 
             * @brief The pluginlib loader for the \ref sensing_info_t class.
             */
            static pluginlib::ClassLoader<sensing_info_t> loader;

        };
    }
}

#endif

