/**
 * @file linear_distance_metric.cpp 
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

#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::util::linear_distance_metric_t, prx::util::distance_metric_t)

namespace prx
{
    namespace util
    {       

        linear_distance_metric_t::linear_distance_metric_t() { }

        linear_distance_metric_t::~linear_distance_metric_t()
        {
            clear();
        }

        unsigned linear_distance_metric_t::add_point(const abstract_node_t* embed)
        {
            unsigned k;
            if( nr_points > 14 )
                k = 4.25 * log(nr_points);
            else
                k = nr_points;

            points.push_back(embed);
            ++nr_points;
            return nr_points;
        }

        unsigned linear_distance_metric_t::add_points(const std::vector< const abstract_node_t* >& embeds)
        {
            for( unsigned i = 0; i < embeds.size(); ++i )
                add_point(embeds[i]);
            return nr_points;
        }

        void linear_distance_metric_t::remove_point(const abstract_node_t* embed)
        {
            unsigned counter = 0;
            bool found = false;

            while( !found && counter < nr_points )
            {
                if( embed == points[counter] )
                    found = true;
                else
                    ++counter;
            }
            if( found )
            {
                --nr_points;
                points[counter] = points[nr_points];
                points.pop_back();
            }
        }

        const std::vector< const abstract_node_t* > linear_distance_metric_t::multi_query(const space_point_t* query_point, unsigned ink) const
        {
            //Make sure we are performing a query for less or as many points as we know
            if( ink > nr_points )
                ink = nr_points;
            //Set up the vector for the return
            std::vector< const abstract_node_t* > ret;
            ret.resize(ink);
            double dists[ink];
            double tmp_dist;
            double worst_dist = 0.0;
            unsigned worst_index = 0;
            //First make an initial population to return
            for( unsigned i = 0; i < ink; i++ )
            {
                ret[i] = points[i];
                dists[i] = distance_function(query_point, points[i]->point);
                if( dists[i] > worst_dist )
                {
                    worst_index = i;
                    worst_dist = dists[i];
                }
            }

            //Then, search through the remaining population to get the closest neighbors
            for( unsigned i = ink; i < nr_points; i++ )
            {
                //Get the linear distance
                tmp_dist = distance_function(query_point, points[i]->point);
                //If this distance is better than our worst distance
                if( tmp_dist < worst_dist )
                {
                    //Replace the dude who is worst
                    ret[worst_index] = points[i];
                    dists[worst_index] = tmp_dist;
                    //Now find the new worst guy ever
                    worst_dist = 0;
                    for( unsigned k = 0; k < ink; k++ )
                    {
                        if( dists[k] > worst_dist )
                        {
                            worst_dist = dists[k];
                            worst_index = k;
                        }
                    }
                }
            }

            std::sort(ret.begin(), ret.end(), compare_node_t(distance_function, query_point));
            return ret;
        }

        const std::vector< const abstract_node_t* > linear_distance_metric_t::radius_query(const space_point_t* query_point, double rad) const
        {
            //Set up the vector for the return
            std::vector< const abstract_node_t* > ret;

            //Then, search through the remaining population to get the closest neighbors
            for( unsigned i = 0; i < nr_points; i++ )
            {
                //Get the linear distance
                double tmp_dist = distance_function(query_point, points[i]->point);
                if( tmp_dist < rad )
                    ret.push_back(points[i]);
            }

            std::sort(ret.begin(), ret.end(), compare_node_t(distance_function, query_point));

            return ret;
        }

        unsigned linear_distance_metric_t::radius_query(const space_point_t* query_point, double rad, std::vector< const abstract_node_t* >& ret) const
        {
            unsigned counter = 0;
            std::vector<const abstract_node_t*>::iterator iter;
            std::vector<const abstract_node_t*>::const_iterator node_iter;
            iter = ret.begin();
            for( node_iter = points.begin(); node_iter != points.end(); node_iter++ )
            {
                const abstract_node_t* node = *node_iter;
                double distance = distance_function(node->point, query_point);
                if( distance < rad )
                {
                    *iter = node;
                    iter++;
                    counter++;
                }
            }
            return counter;
        }

        const std::vector< const abstract_node_t* > linear_distance_metric_t::radius_and_closest_query(const space_point_t* query_point, double rad, const abstract_node_t*& closest)const
        {
            std::vector< const abstract_node_t* > ret;
            for( unsigned i = 0; i < nr_points; i++ )
            {
                double distance = distance_function(points[i]->point, query_point);
                if( distance < rad )
                {
                    ret.push_back(points[i]);
                }
            }
            if( ret.size() == 0 )
            {
                double min_distance = PRX_INFINITY;
                int min_index = -1;
                for( unsigned i = 0; i < nr_points; i++ )
                {
                    double distance = distance_function(points[i]->point, query_point);
                    if( distance < min_distance )
                    {
                        min_index = i;
                        min_distance = distance;
                    }
                }
                ret.push_back(points[ min_index ]);
            }
            std::sort(ret.begin(), ret.end(), compare_node_t(distance_function, query_point));
            return ret;
        }

        unsigned linear_distance_metric_t::radius_and_closest_query(const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest) const
        {
            unsigned counter = 0;
            std::vector<const abstract_node_t*>::iterator iter;
            std::vector<const abstract_node_t*>::const_iterator node_iter;
            iter = closest.begin();
            for( node_iter = points.begin(); node_iter != points.end(); node_iter++ )
            {
                const abstract_node_t* node = *node_iter;
                double distance = distance_function(node->point, query_point);
                if( distance < rad )
                {
                    *iter = node;
                    iter++;
                    counter++;
                }
            }
            if( counter == 0 )
            {
                double min_distance = PRX_INFINITY;
                for( node_iter = points.begin(); node_iter != points.end(); node_iter++ )
                {
                    const abstract_node_t* node = *node_iter;
                    double distance = distance_function(node->point, query_point);
                    if( distance < min_distance )
                    {
                        *iter = node;
                        min_distance = distance;
                    }
                }
                iter++;
                counter++;
            }

            std::sort(closest.begin(), iter, compare_node_t(distance_function, query_point));
            return counter;
        }

        const abstract_node_t* linear_distance_metric_t::single_query(const space_point_t* query_point) const
        {
            double min_distance = PRX_INFINITY;
            int min_index = -1;
            for( unsigned i = 0; i < nr_points; i++ )
            {
                double distance = distance_function(points[i]->point, query_point);
                if( distance < min_distance )
                {
                    min_index = i;
                    min_distance = distance;
                }
            }
            if( min_index != -1 )
                return points[ min_index ];
            else
                return NULL;
        }

        void linear_distance_metric_t::clear()
        {
            points.resize(0);
            nr_points = 0;
        }

        void linear_distance_metric_t::rebuild_data_structure() {
            // The Array will take care of itself as it is being updated
            // This function thusly does nothing.
        }

    }
}



