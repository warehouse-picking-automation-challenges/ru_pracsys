/**
 * @file statistics.hpp
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

#ifndef PRX_STATISTICS_HPP
#define	PRX_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <iostream>
#include <fstream>
namespace prx 
{ 
    namespace util 
    {
        
        /**
         * @brief <b> An abstract class to maintain statistics for experiments. </b>
         * 
         * An abstract class to maintain statistics for experiments. 
         * 
         * This version of statistics contains only a time and steps variables. So
         * it can count and store information about the time and how many steps have been
         * executed. If you need to keep extra information for an experiment you have to inherit 
         * from this class and build your own class with the statistics that you want to store.
         * 
         * @author Athanasios Krontiris
         */
        class statistics_t
        {
            
        public:
            statistics_t();
            
            virtual ~statistics_t();
            
            template<class T>
            T* as()
            {        
                return static_cast<T*>(this);
            }
            
            /**
             * Put all the statistics in a string and return that string.
             * 
             * @return A string with the statistics that have been collected during an experiment.
             */
            virtual std::string get_statistics() const;
            
            /**
             * Write the statistics in an ofstream that has been given by the user.
             * In the abstract statistics_t class this function is using the \c get_statistics() 
             * function to print the statistics to the stream.
             * 
             * @param stream The stream that the user wants to write the statistics to.
             * @return True if the stream was valid so the statistics are printed in this stream. Else false.
             */
            virtual bool serialize(std::ofstream& stream) const;
            
            /**
             * @brief Suggested statistic: Time of executions.
             */
            double time;
            /**
             * @brief Suggested statistic: Number of iterations.
             */
            int steps;
            
        };
        
    } 
}

#endif
