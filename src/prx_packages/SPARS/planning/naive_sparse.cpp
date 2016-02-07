/**
 * @file naive_sparse.cpp
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

#include "planning/naive_sparse.hpp"

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

#include <math.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::spars::naive_sparse_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace spars
        {

        naive_sparse_t::naive_sparse_t()
        {
            //plan::prm_star_t::prm_star_t();
        }

        naive_sparse_t::~naive_sparse_t()
        {
        }

        void naive_sparse_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            plan::prm_star_t::init(reader, template_reader);
            
            mean_valence = parameters::get_attribute_as<double>("mean_valence", reader, template_reader, 5);
            dev_valence = parameters::get_attribute_as<double>("dev_valence", reader, template_reader, 0.7);
        }

        void naive_sparse_t::reset()
        {
            plan::prm_star_t::reset();
        }

        void naive_sparse_t::setup()
        {
            plan::prm_star_t::setup();
        }

        bool naive_sparse_t::execute()
        {
            //Do all the sampling in a batch manner
            PRX_ASSERT(input_specification != NULL);
            try
            {
                do
                {  
                    valid_random_sample();
                    internal_add_node(random_point);
                    PRX_DEBUG_COLOR("Has nodes: " << num_vertices << " : " << boost::num_vertices(graph.graph), PRX_TEXT_MAGENTA);
                }
                while( !input_specification->get_stopping_criterion()->satisfied() );
            }
            catch( stopping_criteria_t::stopping_criteria_satisfied e )
            {
                update_k(num_vertices);

                //Then, attempt to randomly add a few edges here and there.
                connect_graph();

                PRX_DEBUG_COLOR("Has edges: " << num_edges, PRX_TEXT_CYAN );
            }

            return succeeded();
        }

        void naive_sparse_t::connect_graph()
        {
            //Create the objects which will do the normal distribution generation crepe
            boost::mt19937 *rng = new boost::mt19937();
            rng->seed(time(NULL));

            boost::normal_distribution<> distribution(mean_valence, dev_valence);
            boost::variate_generator< boost::mt19937, boost::normal_distribution<> > dist(*rng, distribution);


            //For each node in the graph
            foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
            {
                //Do a knn search
                std::vector< const abstract_node_t* > neighbors = metric->multi_query(graph[v],k);
                unsigned k_nn = neighbors.size();

                //Determine how many neighbors v has
                unsigned num_neighbor = boost::out_degree(v, graph.graph);

                //Determine how many neighbors we want v to have.
                unsigned des_neighbor = round(dist());

                //Now, while this node still needs more neighbors
                while(num_neighbor < des_neighbor && k_nn)
                {
                    //Pick a random neighbor to try
                    unsigned r = rand()%k_nn;
                    undirected_vertex_index_t u = neighbors[r]->as< prm_star_node_t >()->index;

                    //If these two guys aren't already adjacent
                    if(!boost::edge(v, u, graph.graph).second)
                    {
                        //Test the connection
                        new_plan.clear();

                        local_planner->steer(graph[v]->point, graph[u]->point, new_plan, path1);
                        //If the path is valid
                        if( new_plan.size() != 0 && validity_checker->is_valid(path1) )
                        {
                            //Then these guys can totally be neighbors, add the GD edge...
                            internal_add_edge(v, u);
                            //And increment the number of neighbors this node has
                            ++num_neighbor;
                        }

                        path1.clear();

                    }
                    //Then, this neighbor has already been tested, so remove it from the list.
                    neighbors[r] = neighbors[--k_nn];
                }
            }
        }


        undirected_vertex_index_t naive_sparse_t::internal_add_node(const space_point_t* n_state)
        {
            undirected_vertex_index_t v = graph.add_vertex< prm_star_node_t >();
            ++num_vertices;
            graph.get_vertex_as< prm_star_node_t > (v)->init_node(state_space, n_state);
            metric->add_point(graph[v]);
            return v;
        }

        undirected_edge_index_t naive_sparse_t::internal_add_edge(undirected_vertex_index_t u, undirected_vertex_index_t v)
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

        }
    }
}
