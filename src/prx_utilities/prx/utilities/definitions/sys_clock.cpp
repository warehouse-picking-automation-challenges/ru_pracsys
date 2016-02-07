/**
 * @file sys_clock.hpp 
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

#include <math.h>
#include "prx/utilities/definitions/sys_clock.hpp"

namespace prx 
 { 
 namespace util 
 {

sys_clock_t::sys_clock_t()
{
    reset();
    elapsed = 0;
}

sys_clock_t::~sys_clock_t()
{}

double sys_clock_t::get_time_in_secs()
{  
    gettimeofday( &finish, NULL ); 
    double s = finish.tv_sec;
    s += ( 0.000001 * finish.tv_usec );
    return s;
}

void sys_clock_t::reset()
{
    gettimeofday( &start, NULL );
}


double sys_clock_t::measure()
{
    gettimeofday( &finish, NULL ); 
    elapsed = (finish.tv_sec - start.tv_sec) + 0.000001 * (finish.tv_usec - start.tv_usec);
    return elapsed;
}


double sys_clock_t::measure_reset()
{
    measure();
    reset();
    return elapsed;
}

void sys_clock_t::add_delay_user_clock( double delay )
{
    int dl = (int)delay;
    int rest = (int)round( (double)(delay - dl) * 1000000.0);
    
    start.tv_sec  -= dl;
    start.tv_usec -= rest;
    if( start.tv_usec < 0 )
    {
	start.tv_sec -= 1;
	start.tv_usec = 1000000 + start.tv_usec;
    }
}

} 
 }