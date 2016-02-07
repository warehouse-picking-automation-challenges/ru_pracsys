/**
 * @file image.hpp
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
#ifndef PRX_IMAGE_HPP
#define PRX_IMAGE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "lodepng.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        /**
         * @brief <b> An image class for visualizing data.</b>
         * 
         * An image class for visualizing data. Creates a canvas that has its origin in the middle of the image.
         * 
         *  
         * @author Kostas Bekris
         */
        class image_t
        {
        public:
            /**
             * @brief Constructor
             * @param width Width of the image.
             * @param height Height of the image.
             */
            image_t( unsigned int width, unsigned int height );
            ~image_t();
            
            /**
             * @brief Makes the entire image blue.
             * Makes the entire image blue.
             */
            void clear();
            
            /**
             * @brief Sets an image coordinate's pixel value.
             * @param coord_x X-coordinate
             * @param coord_y Y-coordinate
             * @param red The red value.
             * @param green The green value.
             * @param blue The blue value.
             */
            void color_pixel( int coord_x, int coord_y, unsigned char red, unsigned char green, unsigned char blue );
            
            /**
             * @brief Outputs the image to a file.
             * @param filename The name of the file to output to.
             */
            void encode( const char* filename );
            
            /**
             * @brief Reports average red value and number of pixels colored.
             * @param counter The number of pixels colored.
             * @return The average red value.
             */
            double statistics( int* counter );
            
        protected:
            
            /**
             * @brief The storage for the image.
             */
            std::vector<unsigned char> image;
            
            /**
             * @brief The width of the image.
             */
            unsigned width;
            
            /**
             * @brief The height of the image.
             */
            unsigned height;
            
            /**
             * @brief The coordinates for the middle of the image.
             */
            int zero_x, zero_y;
            
            /**
             * @brief Min and max x values for the image.
             */
            int min_x, max_x;
            
            /**
             * @brief Min and max y values for the image.
             */
            int min_y, max_y;
        };
        
    } 
}

#endif 
