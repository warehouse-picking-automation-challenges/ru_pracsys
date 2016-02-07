/**
 * @file angle.hpp 
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

#ifndef PRACSYS_ANGLE_HPP
#define PRACSYS_ANGLE_HPP

#include "prx/utilities/definitions/defs.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        /**
         * Normalizes the given angle in radians to the interval [0, 2*PI]
         *
         * @brief Normalizes the angle to the interval [0,2*PI].
         *
         * @param angle The angle to normalize
         * @return The normalized angle from [0, 2PI]
         */
        static inline
        double norm_angle_zero( double angle )
        {
            while( angle > PRX_2PI )
                angle -= PRX_2PI;
            while( angle < 0.0 )
                angle += PRX_2PI;
            return angle;
        }
        
        /**
         * Normalizes the given angle in radians to the interval [-PI, PI]
         *
         * @brief Normalizes the angle to the interval [-PI,PI].
         *
         * @param angle The angle to normalize
         * @return The normalized angle from [-PI,PI]
         */
        static inline
        double norm_angle_pi( double angle )
        {
            while( angle > PRX_PI )
                angle -= PRX_2PI;
            while( angle < -PRX_PI )
                angle += PRX_2PI;
            return angle;
        }
        
        /**
         * Return the minimum angle between the two angles with the orientation of the angle
         *
         * @brief Returns the minimum angle and the orientation of the angle beteween two angles
         *
         * @param angle1 The first angle
         * @param angle2 The second angle
         * @return The minimum angle
         */
        static inline
        double minimum_angle_with_orientation(double angle1, double angle2)
        {
            angle1 = norm_angle_zero(angle1);
            angle2 = norm_angle_zero(angle2);
            
            double diff = angle2-angle1;
            if(diff < -PRX_PI)
                return PRX_2PI + diff;
            if(diff > PRX_PI)
                return -PRX_2PI + diff;
            return diff;
        }
        
        /**
         * Checks if two angles are approximately equal within some epsilon threshold
         * 
         * @brief Checks if two angles are approximately equal within some epsilon threshold
         * @param angle1 The first angle
         * @param angle2 The second angle
         * @param error_check Error epsilon thresold
         * @return True if the angles are equal, false otherwise
         */
        static inline
        bool are_angles_approximate_equal(double angle1, double angle2, double error_check=0)
        {
            double val;
            if(error_check == 0)
                error_check = 0.0002;
            
            angle1 = norm_angle_zero(angle1);
            angle2 = norm_angle_zero(angle2);
            
            val = fabs(angle1 - angle2);
            return (val >= PRX_2PI-error_check || val <= error_check);
            /*
             if(angle1 > error_check && angle2 > error_check)
             {
             val = fabs(angle1 - angle2);
             return (val >= PRX_2PI-error_check || val <= error_check);
             }
             else
             {
             if(angle1 < error_check)
             {
             if( angle2 > PRX_2PI - error_check )
             return fabs( angle1 + PRX_2PI - angle2) <= error_check;
             }
             else if( angle2 < error_check )
             {
             if( angle1 > PRX_2PI - error_check )
             return fabs( angle2 + PRX_2PI - angle1) <= error_check;
             }
             val = fabs(angle1 - angle2);
             return (val >= PRX_2PI-error_check || val <= error_check);
             }
             */
        }
        
        /**
         * Return the angle between two points (A.B)/ (|A|*|B|)
         *
         * @brief Returns the angle between two points
         *
         * @param dot_product The dot product between the two points
         * @param magnitude_A The magnitude of point A
         * @param magnitude_B The magnitude of point B
         * 
         * @return The angle between two points
         */
        static inline
        double angle_between_points(double dot_product, double magnitude_A, double magnitude_B)
        {
            return acos(dot_product/(magnitude_A*magnitude_B));
        }
        
    } 
}

#endif
