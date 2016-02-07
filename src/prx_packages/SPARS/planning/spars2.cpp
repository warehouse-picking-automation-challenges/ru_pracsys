/**
 * @file spars2.cpp
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

#include "planning/spars2.hpp"
//#include "planning/spars2_graph.hpp"
#include "planning/spars2_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/connected_components.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::spars::spars2_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace spars
        {

        spars2_t::spars2_t()
        {
            //plan::prm_star_t::prm_star_t();
            delete statistics;
            statistics = new spars2_statistics_t();
            deltas_set = false;
            near_point = NULL;
        }

        spars2_t::~spars2_t()
        {
            for(unsigned i=0; i<dense_sample_bounds.size(); ++i)
            {
                if(dense_sample_bounds[i] != NULL)
                {
                    delete dense_sample_bounds[i];
                }
            }
            if(near_point != NULL)
            {
                state_space->free_point(near_point);
                near_point = NULL;
            }
        }

        void spars2_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            plan::prm_star_t::init(reader, template_reader);

            sparse_delta = parameters::get_attribute_as<double>("sparse_delta", reader, template_reader, 0.1);
            dense_delta = parameters::get_attribute_as<double>("dense_delta", reader, template_reader, 0.005);

            stretch = parameters::get_attribute_as<double>("stretch_factor", reader, template_reader, 1.5);
            extent_limit = parameters::get_attribute_as<unsigned>("extent_limit", reader, template_reader, PRX_INFINITY);
        }

        void spars2_t::reset()
        {
            plan::prm_star_t::reset();
        }

        void spars2_t::setup()
        {
            plan::prm_star_t::setup();

            if(!near_point)
            {
                near_point = state_space->alloc_point();
            }

            //Compute the space's extent
            // double extent = 0;
            // double diff = 0;

            // std::vector<bounds_t*> bounds = state_space->get_bounds();
            // unsigned lim = PRX_MINIMUM(bounds.size(), extent_limit);

            // for(unsigned i=0; i<lim; ++i)
            // {
            //     bounds_t* b = bounds[i];
            //     diff = b->get_upper_bound() - b->get_lower_bound();
            //     extent += diff * diff;
            // }
            // extent = sqrt(extent);

            //Actally adjust the deltas
            // if(!deltas_set)
            // {
            //     sparse_delta *= extent;
            //     dense_delta *= extent;
            //     deltas_set = true;

            //     //Set up the dense sampling bounds as well
            //     for(unsigned i=0; i<state_space->get_dimension(); ++i)
            //     {
            //         dense_sample_bounds.push_back( new bounds_t() );
            //         dense_sample_bounds[i]->set_upper_bound(dense_delta);
            //         dense_sample_bounds[i]->set_lower_bound(-dense_delta);
            //     }
            // }
        }

        void spars2_t::step()
        {
            valid_random_sample();
            checked_add_node(random_point);
            update_k(num_vertices);
        }


        bool spars2_t::checked_add_node(const space_point_t* n_state)
        {
            if( metric->get_nr_points() > 0 )
            {
                const spars2_node_t* node = metric->single_query(n_state)->as<spars2_node_t > ();
                if( node != NULL && metric->distance_function(n_state, node->point) <= PRX_DISTANCE_CHECK )
                {
                    PRX_DEBUG_COLOR("The point is already in the graph : " << state_space->print_point(n_state,4),PRX_TEXT_BROWN);
                    return false;
                }
            }

            if(!check_add_coverage(n_state))
            {
                if(!check_add_connectivity(n_state))
                {
                    if(!check_add_interface(n_state))
                    {
                        //Things probably have to happen here...
                        //if(!check_add_quality(n_state))
                        //{
                            //Failed to add the node for any reason, report such
                            return false;
                        //}
                    }
                }
            }

            return true;
        }


        bool spars2_t::check_add_coverage(const space_point_t* n_state)
        {
            const undirected_node_t* node;
            neighbors.clear();
            visible_neighbors.clear();

            std::vector< const util::abstract_node_t* > nn = metric->radius_query(n_state, sparse_delta);

            for(unsigned i=0; i<nn.size(); ++i)
            {
                neighbors.push_back( dynamic_cast<const prm_star_node_t*>(nn[i]) );
            }

            for(unsigned i=0; i<neighbors.size(); ++i)
            {
                node = neighbors[i]->as< undirected_node_t >();
                new_plan.clear();

                local_planner->steer(n_state, node->point, new_plan, path1);
                //If the path is valid
                if( new_plan.size() != 0 && validity_checker->is_valid(path1) )
                {
                    //Then the node is visible, put it in the visible list
                    visible_neighbors.push_back(neighbors[i]);
                }

                path1.clear();
            }

            //If anybody can see him, don't add him
            if(visible_neighbors.size() != 0)
                return false;
            //Visible by no-one, add state
            internal_add_node(n_state);
            return true;
        }

        bool spars2_t::check_add_connectivity(const space_point_t* n_state)
        {
            //If there is only one (or no) visible neighbor, don't add it.
            if(visible_neighbors.size() <= 1)
                return false;

            int num = boost::connected_components(graph.graph, graph.components);

            //If there's only 1 CC, nothing to do
            if( num == 1 )
                return false;

            std::vector< undirected_vertex_index_t > near_verts;
            //Otherwise, get the component of the first visible neighbor.
            int rep = graph.components[visible_neighbors[0]->index];
            //Now, for each other visible neighbor
            for(unsigned i=1; i<visible_neighbors.size(); ++i)
            {
                if(rep != graph.components[visible_neighbors[i]->index])
                {
                    near_verts.push_back(visible_neighbors[i]->index);
                }
            }

            //If the set has things
            if(near_verts.size())
            {
                //First, push back the first visible neighbor, since he's our indicator
                near_verts.push_back(visible_neighbors[0]->index);
                //We know we need to add a new vertex to connect things.
                undirected_vertex_index_t u = internal_add_node(n_state);
                //Then connect him to ebidibody
                for(unsigned i=0; i<near_verts.size(); ++i)
                {
                    internal_add_edge(u, near_verts[i]);
                }
                //And report that we added a node
                return true;
            }
            //Didn't add him for this reason, report that
            return false;
        }

        bool spars2_t::check_add_interface(const space_point_t* n_state)
        {
            //First of all, we must have a sufficient number of neighbors
            if(visible_neighbors.size() < 2 || neighbors.size() < 2)
                return false;

            //If the two nearest visible neighbors are also the two nearest, there's an interface!
            if( visible_neighbors[0]->index == neighbors[0]->index &&
                visible_neighbors[1]->index == neighbors[1]->index)
            {
                //Generate the straight-line path between these neighbors
                new_plan.clear();

                local_planner->steer(visible_neighbors[0]->point, visible_neighbors[1]->point, new_plan, path1);
                //If the path is valid
                if( new_plan.size() != 0 && validity_checker->is_valid(path1) )
                {
                    //Then the neighbors can see each other, add the edge
                    internal_add_edge(visible_neighbors[0]->index, visible_neighbors[1]->index); //TODO: make sure this does error checking
                }
                else
                {
                    //Need to add this dude as a node
                    undirected_vertex_index_t v = internal_add_node(n_state);
                    //And connect him to these neighbors
                    internal_add_edge(v, visible_neighbors[0]->index);
                    internal_add_edge(v, visible_neighbors[1]->index);
                }

                path1.clear();
                return true;
            }

            return false;
        }

        bool spars2_t::check_add_quality(const space_point_t* n_state)
        {
            bool interface_found = false;
            undirected_vertex_index_t vp;
            undirected_vertex_index_t v = visible_neighbors[0]->index;
            const prm_star_node_t* rep;

            //First, see if there is an interface near this point by sampling around
            for(unsigned i=0; i<state_space->get_dimension() && !interface_found; ++i)
            {
                //Sample the random near point
                state_space->uniform_sample_near(random_point, near_point, dense_sample_bounds);

                //Determine its nearest neighbor.
                rep = compute_representative(near_point);
                //If it has no representative, it must be added as a new guard
                if(!rep)
                {
                    internal_add_node(n_state);
                    return true;
                }
                //Then, if the new point is actually reachable
                local_planner->steer(random_point, near_point, new_plan, path1);
                if(validity_checker->is_valid(path1))
                {
                    // Get the rep's index
                    vp = rep->index;
                    //If it has a different representative, there's an interface here, so we're in business.
                    if(vp != visible_neighbors[0])
                    {
                        interface_found = true;
                    }
                }
            }
            //If no interface was found, don't add the node, report failure.
            if(!interface_found)
            {
                return false;
            }

            //TODO: Need to update the point information here

            //For each other vertex, v", such that v,v" is an edge, v,v" has an interface, and v',v" does NOT have an edge
            foreach(undirected_vertex_index_t vpp, boost::adjacent_vertices(v, graph.graph))
            {
                if( !boost::edge(vp,vpp,graph.graph).second && share_interface(v,vpp) )
                {
                    //Grab the specific point record for this pair of vp and vpp
                    point_record_t* record = (graph.get_vertex_as<spars2_node_t>(v))->get_point_record(vp, vpp);
                    //Now, only if this record has a valid pair of points
                    if(record->valid)
                    {
                        //Compute the maximum spanner path
                        double max_spanner_dist = compute_max_spanner_path(v, vp, vpp);
                        //Then, if the spanner property is violated
                        if(stretch * metric->distance_function(record->q, record->qp) < max_spanner_dist)
                        {
                            //TODO: Add the path which fixes this suboptimality.
                        }
                    }
                }
            }

            return false;
        }

        const prm_star_node_t* spars2_t::compute_representative(const space_point_t* n_state)
        {
            const undirected_node_t* node;
            const std::vector< const abstract_node_t* > neighbors = metric->radius_query(n_state, sparse_delta);
            if(neighbors.size() == 0)
            {
                //There are no neighbors, so just abort out.
                return NULL;
            }
            //Otherwise, check them in order to see who is closest
            for(unsigned i=0; i<neighbors.size(); ++i)
            {
                node = neighbors[i]->as< undirected_node_t >();
                new_plan.clear();

                local_planner->steer(n_state, node->point, new_plan, path1);
                //If the path is valid
                if( new_plan.size() != 0 && validity_checker->is_valid(path1) )
                {
                    //Then the node is visible, it must be the representative
                    path1.clear();
                    return node->as< prm_star_node_t >();
                }

                path1.clear();
            }

            return NULL;
        }

        bool spars2_t::share_interface(undirected_vertex_index_t v, undirected_vertex_index_t vpp)
        {
            return false; //TODO
        }

        double spars2_t::compute_max_spanner_path(undirected_vertex_index_t v, undirected_vertex_index_t vp, undirected_vertex_index_t vpp)
        {
            return 0.0; //TODO
        }

        undirected_vertex_index_t spars2_t::internal_add_node(const space_point_t* n_state)
        {
            undirected_vertex_index_t v = graph.add_vertex< prm_star_node_t >();
            ++num_vertices;
            graph.get_vertex_as< prm_star_node_t > (v)->init_node(state_space, n_state);
            metric->add_point(graph[v]);
            return v;
        }

        undirected_edge_index_t spars2_t::internal_add_edge(undirected_vertex_index_t u, undirected_vertex_index_t v)
        {
            //Add the edge to the graph
            double dist = metric->distance_function(graph[u]->point, graph[v]->point);
            undirected_edge_index_t e = graph.add_edge< prm_star_edge_t > (u, v, dist);
            graph.get_edge_as<prm_star_edge_t > (e)->id = num_edges;
            ++num_edges;
            //TODO: have to figure out what I'm doing here, unfortunately... may have to always store due to things
            if( visualize_graph )
                graph.get_edge_as< prm_star_edge_t >(e)->path = path1;
            return e;
        }

        const statistics_t* spars2_t::get_statistics()
        {
            statistics->as<spars2_statistics_t > ()->num_vertices = boost::num_vertices(graph.graph);
            statistics->as<spars2_statistics_t > ()->num_edges = boost::num_edges(graph.graph);
            //TODO: update the other information

            return statistics;
        }

        }
    }
}
