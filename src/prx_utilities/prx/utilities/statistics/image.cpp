/**
 * @file image.cpp
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

#include "prx/utilities/statistics/image.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        image_t::image_t( unsigned w, unsigned h )
        {
            width = w;
            height = h;
            
            image.resize(width * height * 4);
            
            clear();
            
            zero_x = width/2;
            zero_y = height/2;
            
            max_x = width/2 - 1;
            max_y = height/2 - 1;
            
            min_x = -max_x;
            min_y = -max_y;
        }
        
        image_t::~image_t()
        {
        }
        
        void image_t::clear()
        {
            for(unsigned y = 0; y < height; y++)
                for(unsigned x = 0; x < width; x++)
                {
                    image[4 * width * y + 4 * x + 0] = 0;
                    image[4 * width * y + 4 * x + 1] = 0;
                    image[4 * width * y + 4 * x + 2] = 255;
                    image[4 * width * y + 4 * x + 3] = 255;
                }
        }
        
        void image_t::color_pixel( int coord_x, int coord_y, unsigned char red, unsigned char green, unsigned char blue )
        {
            if( min_x <= coord_x && max_x >= coord_x )
                if( min_y <= coord_y && max_y >= coord_y )	
                {
                    int x_value = coord_x + zero_x;
                    int y_value = coord_y + zero_y;
                    
                    unsigned char old_red = image[4 * width * y_value + 4 * x_value + 0 ];
                    if( old_red == 0 || old_red > red )
                    {
                        image[4 * width * y_value + 4 * x_value + 0 ] = red;
                        image[4 * width * y_value + 4 * x_value + 1 ] = green;
                        image[4 * width * y_value + 4 * x_value + 2 ] = blue;	    
                    }
                }
        }
        
        void image_t::encode( const char* filename )
        {
            const int dimx = width, dimy = height;
            int i, j;
            FILE *fp = fopen(filename, "wb"); /* b - binary mode */
            (void) fprintf(fp, "P6\n%d %d\n255\n", dimx, dimy);
            for (j = 0; j < dimy; ++j)
            {
                for (i = 0; i < dimx; ++i)
                {
                    static unsigned char color[3];
                    color[0] = image[4 * width * j + 4 * i + 0 ];  /* red */
                    color[1] = image[4 * width * j + 4 * i + 1 ];  /* green */
                    color[2] = image[4 * width * j + 4 * i + 2 ];  /* blue */
                    (void) fwrite(color, 1, 3, fp);
                }
            }
            (void) fclose(fp);
            return;
            
            
            // lodepng::encode(filename, image, width, height);
        }
        
        double image_t::statistics( int* counter )
        {
            double color = 0;
            *counter = 0;
            for(unsigned y = 0; y < height; y++)
                for(unsigned x = 0; x < width; x++)
                {
                    unsigned char red = image[4 * width * y + 4 * x + 0 ];
                    if( red != 0 )
                    {
                        color += red;
                        *counter = *counter +  1;
                    }
                }
            return color/ (double)(*counter);
        }
        
    } 
}