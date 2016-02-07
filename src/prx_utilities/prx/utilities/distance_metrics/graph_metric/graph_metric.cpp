/**
 * @file graph_metric.cpp 
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


#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/distance_metrics/graph_metric/graph_metric.hpp"
#include "prx/utilities/graph/abstract_node.hpp"

#include <pluginlib/class_list_macros.h>
#include <algorithm> 

PLUGINLIB_EXPORT_CLASS( prx::util::graph_distance_metric_t, prx::util::distance_metric_t)

namespace prx
{
    namespace util
    {        

        graph_distance_metric_t::graph_distance_metric_t()
        {
            prox = new graph_proximity_t(NULL);
            query_node = new abstract_node_t();
            close_nodes = (proximity_node_t**)malloc(MAX_KK * sizeof (proximity_node_t*));
            distances = (double*)malloc(MAX_KK * sizeof (double));
        }

        graph_distance_metric_t::~graph_distance_metric_t()
        {
            delete prox;
            delete query_node;
            free(distances);
            free(close_nodes);
        }

        unsigned graph_distance_metric_t::add_point(const abstract_node_t* embed)
        {
            proximity_node_t* node = new proximity_node_t(embed);
            ((abstract_node_t*)embed)->prox_node = node;
            node->d = distance_function;
            prox->add_node(node);
            nr_points++;
            return nr_points;
        }

        unsigned graph_distance_metric_t::add_points(const std::vector< const abstract_node_t* >& embeds)
        {

            foreach(const abstract_node_t* node, embeds)
            {
                add_point(node);
            }
            return nr_points;
        }

        void graph_distance_metric_t::remove_point(const abstract_node_t* embed)
        {
            prox->remove_node(embed->prox_node);
            delete embed->prox_node;
            nr_points--;
        }

        const std::vector< const abstract_node_t* > graph_distance_metric_t::multi_query(const space_point_t* query_point, unsigned ink) const
        {
            query_node->point = (space_point_t*)query_point;
            int i = 0;
            if( ink != 0 )
                i = prox->find_k_close(query_node, close_nodes, distances, ink);
            std::vector<const abstract_node_t*> ret;
            ret.resize(i);
            for( int j = 0; j < i; j++ )
            {
                ret[j] = close_nodes[j]->get_state();
            }
            return ret;
        }

        const std::vector< const abstract_node_t* > graph_distance_metric_t::radius_query(const space_point_t* query_point, double rad) const
        {
            query_node->point = (space_point_t*)query_point;
            int i = prox->find_delta_close(query_node, close_nodes, distances, rad);
            std::vector<const abstract_node_t*> ret;
            ret.resize(i);
            for( int j = 0; j < i; j++ )
            {
                ret[j] = close_nodes[j]->get_state();
            }
            return ret;
        }

        const std::vector< const abstract_node_t* > graph_distance_metric_t::radius_and_closest_query(const space_point_t* query_point, double rad, const abstract_node_t*& closest)const
        {
            query_node->point = (space_point_t*)query_point;
            int i = prox->find_delta_close(query_node, close_nodes, distances, rad);

            std::vector<const abstract_node_t*> ret;
            ret.resize(i);
            for( int j = 0; j < i; j++ )
            {
                ret[j] = close_nodes[j]->get_state();
            }
            if( i == 0 )
                closest = single_query(query_point);
            return ret;
        }

        unsigned graph_distance_metric_t::radius_and_closest_query(const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest) const
        {
            query_node->point = (space_point_t*)query_point;
            int i = prox->find_delta_close_and_closest(query_node, close_nodes, distances, rad);

            for( int j = 0; j < i; j++ )
            {
                closest[j] = close_nodes[j]->get_state();
            }

            return i;
        }

        unsigned graph_distance_metric_t::radius_query(const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest) const
        {
            query_node->point = (space_point_t*)query_point;
            int i = prox->find_delta_close(query_node, close_nodes, distances, rad);
            if( i > 0 )
                sort(close_nodes, distances, 0, i - 1);


            for( int j = 0; j < i; j++ )
            {
                closest[j] = close_nodes[j]->get_state();
            }
            return i;
        }

        const abstract_node_t* graph_distance_metric_t::single_query(const space_point_t* query_point) const
        {
            double dist;
            query_node->point = (space_point_t*)query_point;
            proximity_node_t* node = prox->find_closest(query_node, &dist);
            return node->get_state();
        }

        void graph_distance_metric_t::clear() 
        {
            nr_points = 0;
            delete prox;
            delete query_node;
            free(distances);
            free(close_nodes);
            prox = new graph_proximity_t(NULL);
            query_node = new abstract_node_t();
            close_nodes = (proximity_node_t**)malloc(MAX_KK * sizeof (proximity_node_t*));
            distances = (double*)malloc(MAX_KK * sizeof (double));
        }

        void graph_distance_metric_t::rebuild_data_structure() {
            //no need to do this in graph_distance_metric
        }

    }
}



