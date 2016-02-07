/**
 * @file ANN_distance_metric.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com3
 */

#include "prx/utilities/distance_metrics/ann_metric/ann_distance_metric.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/graph/directed_node.hpp"
#include "prx/utilities/graph/undirected_node.hpp"

#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <list>

PLUGINLIB_EXPORT_CLASS( prx::util::ann_distance_metric_t, prx::util::distance_metric_t)

namespace prx
{
    namespace util
    {     

        ann_distance_metric_t::ann_distance_metric_t()
        {
            trees.resize(PRX_NO_KD_TREES);
            data_start.resize(PRX_NO_KD_TREES);
            for( unsigned i = 0; i < PRX_NO_KD_TREES; ++i )
            {
                trees[i] = NULL;
                data_start[i] = -1;
            }
            tree_points = 0;
            data = NULL;
            nr_points = 0;
            nr_trees = 0;
            data_max = PRX_KD_INITIAL_DATA_MAX;
        }

        ann_distance_metric_t::~ann_distance_metric_t()
        {
            points.clear();
            for( unsigned i = 0; i < PRX_NO_KD_TREES; ++i )
            {
                if( trees[i] )
                    delete trees[i];
                trees[i] = NULL;
                data_start[i] = -1;
            }
            for( unsigned int i = 0; i < nr_points; ++i )
                free(data[i]);
            free(data);

            data = NULL;
            tree_points = 0;
        }

        unsigned ann_distance_metric_t::add_point(const abstract_node_t* embed)
        {
            points.push_back(embed);

            if( data )
            {
                if( points.size() > data_max )
                {
                    data_max *= 2;
                    data = (double**)realloc(data, (data_max)*(sizeof (double*)));
                    for( unsigned k = 0; k < nr_trees; ++k )
                    {
                        trees[k]->set_data_start_point(data + data_start[k]);
                    }
                }
            }
            else
            {
                data = (double**)malloc((data_max)*(sizeof (double*)));
            }

            data[nr_points] = (double*)malloc(space->get_dimension()*(sizeof (double)));
            //Copy information from the point into the data memory
            for( unsigned i = 0; i < space->get_dimension(); ++i )
                data[nr_points][i] = embed->point->at(i);
            //embed->vector.get(data[nr_points-1]);
            ++nr_points;

            //Check for rebuilding the trees
            rebuild_data_structure();

            return nr_points;
        }

        bool ann_distance_metric_t::has_point(const abstract_node_t* embed)
        {
            //First, assume we are not deleting this point
            bool del = false;
            for( unsigned i = 0; i < nr_points; i++ )
            {
                if( std::find(ghost.begin(), ghost.end(), i) == ghost.end() && space->equal_points(points[i]->point, embed->point) )
                {
                    del = true;
                }
            }

            return del;
        }

        unsigned ann_distance_metric_t::add_points(const std::vector< const abstract_node_t* >& inembeds)
        {
            while( data_max < inembeds.size() )
                data_max *= 2;

            if( data )
            {
                data = (double**)realloc(data, (data_max)*(sizeof (double*)));
                for( unsigned k = 0; k < nr_trees; ++k )
                {
                    trees[k]->set_data_start_point(data + data_start[k]);
                }
            }
            else
            {
                data = (double**)malloc((data_max)*(sizeof (double*)));
            }

            //For all the points we wish to add
            for( unsigned i = 0; i < inembeds.size(); ++i )
            {
                //Push the point onto our list of points
                points.push_back(inembeds[i]);

                //And Add its data
                data[nr_points] = (double*)malloc(space->get_dimension()*(sizeof (double)));
                for( unsigned j = 0; j < space->get_dimension(); ++j )
                    data[nr_points][j] = inembeds[i]->point->at(j);
                //And increment the number of points we have
                ++nr_points;
            }
            //Then completely rebuild the data structure
            rebuild_data_structure();
            return nr_points;
        }

        void ann_distance_metric_t::remove_point(const abstract_node_t* embed)
        {
            bool del = false;

            //Then the Spectre needs to be removed
            unsigned index = points.size() + 1;
            for( unsigned j = 0; j < points.size(); j++ )
            {
                if( std::find(ghost.begin(), ghost.end(), j) == ghost.end() && points[j] == embed )
                {
                    index = j;
                    break;
                }
            }
            if( index != points.size() + 1 )
            {
                //PRX_WARN_S("internal: "<<space->print_point(points[index]->point) <<" parameter: "<< space->print_point(embed->point) );
                ghost.push_back(index);
            }
            //If our ghost has gotten too large...
            if( ghost.size() > PRX_PROXIMITY_GHOST_SIZE )
            {

                //We are going to rebuild things
                std::vector< const abstract_node_t* > hold;


                for( unsigned i = 0; i < points.size(); ++i )
                {
                    //First, assume we are not deleting this point
                    del = false;
                    //Search through the ghost, searching for victims
                    for( unsigned k = 0; k < ghost.size(); ++k )
                    {
                        //If the point is in the ghost
                        //if(  ghost[k] == points[i]  )
                        if( ghost[k] == i )
                        {
                            //Then we are getting rid of the point
                            del = true;
                        }
                    }
                    //If we are not deleting this object
                    if( !del )
                    {
                        //Then add it to the temporary list
                        hold.push_back(points[i]);
                    }
                    else
                    {
                        //                space->free_point(points[i]->point);
                        //                delete points[i];
                    }

                }
                //Now copy over the points we'll actually use
                points.clear();
                points.resize(0);
                //Reset the parameters
                nr_points = 0;
                tree_points = 0;
                ghost.clear();
                //Then rebuild

                while( data_max < hold.size() )
                    data_max *= 2;

                if( data )
                {
                    data = (double**)realloc(data, (data_max)*(sizeof (double*)));
                    for( unsigned k = 0; k < nr_trees; ++k )
                    {
                        trees[k]->set_data_start_point(data + data_start[k]);
                    }
                }
                else
                {
                    data = (double**)malloc((data_max)*(sizeof (double*)));
                }

                //For all the points we wish to add
                for( unsigned i = 0; i < hold.size(); ++i )
                {
                    //Push the point onto our list of points
                    points.push_back(hold[i]);
                    //And Add its data
                    data[nr_points] = (double*)malloc(space->get_dimension()*(sizeof (double)));
                    for( unsigned j = 0; j < space->get_dimension(); ++j )
                        data[nr_points][j] = hold[i]->point->at(j);
                    //And increment the number of points we have
                    ++nr_points;
                }
                //Then completely rebuild the data structure
                rebuild_data_structure();
            }
        }

        const std::vector< const abstract_node_t* > ann_distance_metric_t::multi_query(const space_point_t* query_point, unsigned ink) const
        {
            //Our data structures for handling getting the actual k-closest neighbors
            std::vector< const abstract_node_t* > ret;
            ret.resize(0);
            std::list< const abstract_node_t* > states;
            states.resize(0);
            std::list<double> dists;
            dists.resize(0);

            //Data for the ANN calls to return data with
            ANNidxArray nn_idx = new ANNidx[ink + ghost.size()];
            ANNdistArray nn_dist = new ANNdist[ink + ghost.size()];
            ANNpoint qp = new double[ space->get_dimension() ];
            for( unsigned i = 0; i < space->get_dimension(); ++i )
                qp[i] = query_point->at(i);

            std::list< const abstract_node_t* >::iterator state;
            std::list< double >::iterator distance_iter;

            vector_t vec;

            double tmpdist = 0.0;
            bool abort = false;
            bool dead = false;

            //First, search all of the trees for their best results
            for( unsigned j = 0; j < nr_trees; ++j )
            {
                //Search the tree for the nearest k+(size of ghost) neighbors
                unsigned int actual_k = trees[j]->annKSearch(qp, (ink + ghost.size()), nn_idx, nn_dist, 0);
                //For each of the points returned by the query
                for( unsigned i = 0; i < actual_k; ++i )
                {
                    //First assume that the point isn't dead
                    dead = false;
                    //Then, search through the ghost
                    for( unsigned g = 0; g < ghost.size() && !dead; ++g )
                    {
                        //If the point is a ghost
                        //if( points[nn_idx[i]+data_start[j]] == ghost[g] )
                        if( nn_idx[i] + data_start[j] >= nr_points || nn_idx[i] + data_start[j] == ghost[g] )
                        {
                            dead = true;
                        }
                    }
                    //If the point of interest is not dead
                    if( !dead )
                    {
                        //If we have no members, simply push to the back
                        if( states.size() == 0 )
                        {
                            states.push_back(points[nn_idx[i] + data_start[j]]);
                            dists.push_back(distance_function(points[nn_idx[i] + data_start[j]]->point, query_point));
                        }
                            //Otherwise
                        else
                        {
                            //Get its distance
                            tmpdist = distance_function(points[nn_idx[i] + data_start[j]]->point, query_point);
                            //And use that distance to find the point's place in the list
                            for( state = states.begin(), distance_iter = dists.begin(), abort = false; !abort; ++state, ++distance_iter )
                            {
                                //If we are smaller than the next guy in the list
                                if( tmpdist < *distance_iter )
                                {
                                    //Insert ourselves right before him
                                    states.insert(state, points[nn_idx[i] + data_start[j]]);
                                    dists.insert(distance_iter, tmpdist);
                                    //And report that we're done looking
                                    abort = true;
                                }
                                //If we reached the end of the list
                                if( state == states.end() )
                                {
                                    //Put the information at the back of the list
                                    states.push_back(points[nn_idx[i] + data_start[j]]);
                                    dists.push_back(tmpdist);
                                    //And report that we are done looking
                                    abort = true;
                                }
                            }
                        }
                    }
                }
            }

            //Then perform the linear search
            std::vector< const abstract_node_t* > lins;
            //lins.resize(ink+ghost.size());
            std::vector<double> lindists;
            double tmp_dist;
            double worst_dist = 0.0;
            unsigned worst_index = 0;
            unsigned num_entered = 0;

            unsigned lin_size = ink + ghost.size();
            if( lin_size > nr_points - tree_points )
                lin_size = nr_points - tree_points;
            //First make an initial population to return
            //Then, search through the remaining population to get the closest neighbors
            for( unsigned i = tree_points; i < nr_points; ++i )
            {
                //First assume that the point isn't dead
                dead = false;
                //Then, search through the ghost
                for( unsigned g = 0; g < ghost.size() && !dead; ++g )
                {
                    //If the point is a ghost
                    if( i == ghost[g] )
                    {
                        dead = true;
                        lin_size--;
                        if( lin_size < num_entered )
                        {
                            worst_dist = lindists[lin_size - 1];
                            worst_index = lin_size - 1;
                        }

                    }
                }
                //Get the linear distance
                if( !dead )
                {
                    tmp_dist = distance_function(query_point, points[i]->point);
                    //PRX_WARN_S("QUERY: "<<tmp_dist<<" "<<space->print_point(points[i]->point)<<" worst dist: "<<worst_dist<<" "<<lin_size);
                    if( num_entered < lin_size )
                    {
                        bool added = false;
                        for( unsigned int q = 0; q < num_entered && !added; q++ )
                        {
                            if( lindists[q] > tmp_dist )
                            {
                                lins.insert(lins.begin() + q, points[i]);
                                lindists.insert(lindists.begin() + q, tmp_dist);
                                added = true;
                            }
                        }
                        if( !added )
                        {
                            lins.push_back(points[i]);
                            lindists.push_back(tmp_dist);
                        }

                        //lins[num_entered] = points[i];



                        //lindists[num_entered] = tmp_dist;
                        //if(tmp_dist > worst_dist)
                        //{
                        //    worst_dist = tmp_dist;
                        //    worst_index = num_entered;
                        //}

                        num_entered++;
                        worst_index = num_entered - 1;
                        worst_dist = lindists[worst_index];
                    }
                        //If this distance is better than our worst distance
                    else if( tmp_dist < worst_dist )
                    {
                        bool added = false;
                        for( unsigned int q = 0; q < num_entered && !added; q++ )
                        {
                            if( lindists[q] > tmp_dist )
                            {
                                lins.insert(lins.begin() + q, points[i]);
                                lindists.insert(lindists.begin() + q, tmp_dist);
                                added = true;
                            }
                        }


                        //Replace the dude who is worst
                        //lins[worst_index] = points[i];
                        //lindists[worst_index] = tmp_dist;
                        //Now find the new worst guy ever
                        worst_dist = lindists[lin_size - 1];
                    }
                }
            }





            //Throw all of the points into a sorting vector
            std::set< const abstract_node_t*, compare_node_t > sort(compare_node_t(distance_function, query_point));
            //    sort.resize(0);
            for( state = states.begin(); state != states.end(); ++state )
            {
                sort.insert(*state);
            }



            for( unsigned j = 0; j < lin_size; ++j )
            {
                sort.insert(lins[j]);
            }
            ret.assign(sort.begin(), sort.end());
            delete []nn_idx;
            delete []nn_dist;
            delete []qp;

            //    sort( vec.begin(), vec.end() );
            //    ret.erase( std::unique( ret.begin(), ret.end() ), ret.end() );


            return ret;
        }

        const std::vector< const abstract_node_t* > ann_distance_metric_t::radius_query(const space_point_t* query_point, double rad) const
        {
            std::vector< const abstract_node_t* > ret;

            double dist = rad;
            if( rad < 0 )
            {
                dist = (log(points.size())) * gamma_val;
                return multi_query(query_point, (int)dist);
            }
            else
            {
                size_t sz = points.size();
                double nn_measure = space->n_ball_measure(dist);
                unsigned expected_nodes = (nn_measure / space_measure)*((double)sz) + 2;

                if( expected_nodes > sz )
                    expected_nodes = sz;

                std::vector< const abstract_node_t* > nearest_k;
                do
                {
                    nearest_k = multi_query(query_point, expected_nodes);

                    expected_nodes *= 2;
                    if( nearest_k.size() == 0 )
                    {
                        break;
                    }
                }
                while( distance_function(nearest_k.back()->point, query_point) <= dist && (expected_nodes < sz) );

                while( (nearest_k.size() > 0) && distance_function(nearest_k.back()->point, query_point) > dist )
                    nearest_k.pop_back();

                for( unsigned i = 0; i < nearest_k.size(); ++i )
                    ret.push_back(nearest_k[i]);
            }

            return ret;
        }

        const std::vector< const abstract_node_t* > ann_distance_metric_t::radius_and_closest_query(const space_point_t* query_point, double rad, const abstract_node_t*& closest)const
        {
            std::vector< const abstract_node_t* > ret;

            double dist = rad;
            if( rad < 0 )
            {
                dist = (log(points.size())) * gamma_val;
                return multi_query(query_point, (int)dist);
            }
            else
            {
                size_t sz = points.size();
                double nn_measure = space->n_ball_measure(dist);
                unsigned expected_nodes = (nn_measure / space_measure)*((double)sz) + 2;

                if( expected_nodes > sz )
                    expected_nodes = sz;

                std::vector< const abstract_node_t* > nearest_k;
                do
                {
                    nearest_k = multi_query(query_point, expected_nodes);

                    expected_nodes *= 2;
                    if( nearest_k.size() == 0 )
                    {
                        break;
                    }
                }
                while( distance_function(nearest_k.back()->point, query_point) <= dist && (expected_nodes < sz) );

                while( (nearest_k.size() > 0) && distance_function(nearest_k.back()->point, query_point) > dist )
                    nearest_k.pop_back();

                for( unsigned i = 0; i < nearest_k.size(); ++i )
                    ret.push_back(nearest_k[i]);
            }
            if( ret.size() != 0 )
                closest = ret[0];
            else
                closest = single_query(query_point);

            return ret;
        }

        const abstract_node_t* ann_distance_metric_t::single_query(const space_point_t* query_point) const
        {
            if( nr_points == 0 )
                return NULL;

            const abstract_node_t* ret;
            ret = (multi_query(query_point, 1))[0];
            return ret;
        }

        void ann_distance_metric_t::clear()
        {
            points.clear();
            for( unsigned i = 0; i < PRX_NO_KD_TREES; ++i )
            {
                if( trees[i] )
                    delete trees[i];
                trees[i] = NULL;
                data_start[i] = -1;
                if( data && data[i] )
                    free(data[i]);
            }
            if( data )
                free(data);
            data = NULL;
            tree_points = 0;
        }

        void ann_distance_metric_t::rebuild_data_structure()
        {
            // If there are trees to be built
            while( nr_points - tree_points >= pow(2.0, double(nr_trees)) * PRX_KD_SIZE )
            {
                trees[nr_trees] = new tkd_tree_t(data + tree_points, PRX_KD_SIZE * pow(2.0, double(nr_trees)), space->get_dimension(), PRX_KD_BUCKET, space);
                data_start[nr_trees] = tree_points;
                tree_points += PRX_KD_SIZE * pow(2.0, double(nr_trees));
                ++nr_trees;
            }
        }

        std::ostream& operator<<(std::ostream& out, const ann_distance_metric_t& t)
        {
            out << "Points: " << t.nr_points - t.ghost.size() << " = " << t.nr_points << " - " << t.ghost.size() << std::endl;
            out << "Points: Tree " << t.tree_points << " :: List " << t.nr_points - t.tree_points << std::endl;
            for( unsigned i = 0; i < PRX_NO_KD_TREES; ++i )
            {
                if( t.trees[i] )
                {
                    out << "Tree " << i << " : Start " << t.data_start[i] << std::endl;
                    out << t.trees[i];
                }
            }
            out << std::endl;
            
            return out;
        }

        std::string ann_distance_metric_t::print_points()
        {
            std::stringstream out(std::stringstream::out);
            for( unsigned i = 0; i < nr_points; ++i )
            {
                out << i << " : ";
                for( unsigned j = 0; j < space->get_dimension(); ++j )
                {
                    out << data[i][j] << " ";
                }
                out << std::endl;
            }
            
            return out.str();
        }

    }
}

