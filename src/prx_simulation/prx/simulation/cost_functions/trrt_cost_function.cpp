/**
 * @file trrt_cost_function.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/cost_functions/trrt_cost_function.hpp"
#include "prx/utilities/statistics/svg_image.hpp"
#include "prx/utilities/definitions/random.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::trrt_cost_function_t, prx::sim::cost_function_t)

namespace prx
{
	using namespace util;
    using namespace svg;
    namespace sim
    {       
        #define MAX_X 100
        #define MAX_Y 100
        #define VARIANCE 30
        #define START 10

        void midpoint_displace(double* heightmap,int x_min,int x_max,int y_min, int y_max)
        {
            if(x_max-x_min == 1 || y_max-y_min == 1)
                return;

            double bottom_left = heightmap[ x_min + y_min * MAX_X];
            double bottom_right = heightmap[ x_max + y_min * MAX_X];
            double top_left = heightmap[ x_min + y_max * MAX_X];
            double top_right = heightmap[ x_max + y_max * MAX_X];

            int mid_x = (x_min+x_max)/2;
            int mid_y = (y_min+y_max)/2;

            heightmap[mid_x + mid_y * MAX_X] = (bottom_right + bottom_left + top_right + top_left)/4.0 + uniform_random(-VARIANCE,VARIANCE);
            heightmap[x_min + mid_y * MAX_X] = (bottom_left  + top_left    )/2.0 + uniform_random(-VARIANCE,VARIANCE);
            heightmap[x_max + mid_y * MAX_X] = (bottom_right + top_right   )/2.0 + uniform_random(-VARIANCE,VARIANCE);
            heightmap[mid_x + y_min * MAX_X] = (bottom_right + bottom_left )/2.0 + uniform_random(-VARIANCE,VARIANCE);
            heightmap[mid_x + y_max * MAX_X] = (top_right    + top_left    )/2.0 + uniform_random(-VARIANCE,VARIANCE);

            midpoint_displace(heightmap,x_min,mid_x,y_min,mid_y);
            midpoint_displace(heightmap,x_min,mid_x,mid_y,y_max);
            midpoint_displace(heightmap,mid_x,x_max,mid_y,y_max);
            midpoint_displace(heightmap,mid_x,x_max,y_min,mid_y);
        }
        trrt_cost_function_t::trrt_cost_function_t()
        {
            max_x = MAX_X;
            max_y = MAX_Y;

            heightmap = new double[max_x*max_y];
            heightmap[0] = START;
            heightmap[max_x-1] = START;
            heightmap[(max_y-1) * MAX_X] = START;
            heightmap[(max_x-1) + (max_y-1) * MAX_X] = START;

            midpoint_displace(heightmap,0,max_x-1,0,max_y-1);

            std::string name = "/Users/zlittlefield/prx/pracsys/prx_output/costmap.svg";
            Dimensions dimensions(800, 800);
            Document doc(name, Layout(dimensions, Layout::BottomLeft));
            int inflate = 800/MAX_X;
            for( int x = 0 ; x < max_x ; x++ )
            {
                for( int y = 0 ; y < max_y ; y++)
                {
                    // heightmap[x+y*max_x] = uniform_int_random(1,100);
                    if(heightmap[x+y*max_x]<0)
                        heightmap[x+y*max_x] = 1;
                    else if(heightmap[x+y*max_x]>=100)
                        heightmap[x+y*max_x] = 100;

                    doc<<Rectangle(Point(x*inflate,y*inflate),inflate,inflate,svg::Fill( svg::Color(255-heightmap[x+y*max_x],255-heightmap[x+y*max_x],255-heightmap[x+y*max_x])));
                }
            }
            doc<<Rectangle(Point(20*4,20*4),inflate,inflate,svg::Fill( svg::Color(255,0,0)));
            doc<<Rectangle(Point(180*4,180*4),inflate,inflate,svg::Fill( svg::Color(0,0,255)));
            doc.save();

        }
        trrt_cost_function_t::~trrt_cost_function_t()
        {
            delete heightmap;
        }

        double trrt_cost_function_t::state_cost(const space_point_t* s)
        {
            //return 100.0/(std::sqrt(s->at(2)*s->at(2)+s->at(3)*s->at(3)));
            return 1;
            // return heightmap[(int)(std::floor((s->at(0)+100)*(MAX_X/200))+std::floor((s->at(1)+100)*(MAX_X/200))*max_x)];

            // if(s->at(0) < 0)
            // {
            //     if(s->at(1) < 0)
            //     {
            //         return 50;
            //     }
            //     else
            //     {
            //         return 1;
            //     }
            // }
            // else
            // {
            //     if(s->at(1) < 0)
            //     {
            //         return 20;
            //     }
            //     else
            //     {
            //         return 1;
            //     }
            // }
        }

        double trrt_cost_function_t::trajectory_cost(const trajectory_t& t)
        {
            // return t.size();
            return default_uniform_t::trajectory_cost(t);
            // double cost = 0;
            // trajectory_t::const_iterator i = t.begin();
            // trajectory_t::const_iterator j = t.begin();
            // j++;

            // double start_cost;
            // double end_cost;
            // for ( ; j != t.end(); ++i,++j)
            // {
            //     cost+=(.0001);//dist(*i,*j)*
            //     start_cost = state_cost(*i);
            //     end_cost = state_cost(*j);
            //     if(end_cost > start_cost)
            //     {
            //         cost+=(end_cost-start_cost);//dist(*i,*j)*
            //     }
            // }
            // return cost;
        }


    }
}
