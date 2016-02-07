/**
 * @file prm.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/motion_planners/manipulation_mp.hpp"
#include "planning/problem_specifications/manipulation_mp_specification.hpp"
#include "planning/queries/manipulator_mp_query.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "planning/modules/system_name_validity_checker.hpp"
#include "planning/modules/obstacle_aware_astar.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>


PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::manipulation_mp_t, prx::plan::planner_t)


namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace baxter;

        namespace rearrangement_manipulation
        {

            manipulation_mp_t::manipulation_mp_t()
            {
                graph_deserialization_file = "";
                graph_deserialize_flag = false;

                char* w = std::getenv("PRACSYS_PATH");

                prx_output_dir = std::string(w) + "/prx_output/";
                prx_input_dir = std::string(w) + "/prx_input/";
            }

            manipulation_mp_t::~manipulation_mp_t() { }

            void manipulation_mp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing manipulation motion planner... ");
                prm_star_t::init(reader, template_reader);

                if( parameters::has_attribute("graph_deserialization_file", reader, template_reader) )
                {
                    graph_deserialization_file = parameters::get_attribute_as<std::string > ("graph_deserialization_file", reader, template_reader);
                    graph_deserialize_flag = true;
                }

                lazy_runs = parameters::get_attribute_as<int>("lazy_runs", reader, template_reader, 0);

            }

            void manipulation_mp_t::reset()
            {
                prm_star_t::reset();
            }

            void manipulation_mp_t::link_specification(specification_t* new_spec)
            {
                prm_star_t::link_specification(new_spec);
                specs = (manipulation_mp_specification_t*)new_spec;
            }

            void manipulation_mp_t::link_query(plan::query_t* new_query)
            {
                prm_star_t::link_query(new_query);
                in_query = dynamic_cast<manipulator_mp_query_t*>(new_query);
            }

            void manipulation_mp_t::setup()
            {
                random_point = state_space->alloc_point();
                path1.link_space(state_space);
                path2.link_space(state_space);
                new_plan.link_control_space(control_space);
                new_plan2.link_control_space(control_space);

                astar->link_spaces(state_space, control_space);
                astar->link_modules(validity_checker, local_planner);
                astar->link_distance_metric(metric);
                oa_astar = dynamic_cast<obstacle_aware_astar_t*>(astar);
                PRX_ASSERT(oa_astar != NULL);
                oa_astar->link_valid_constraints(specs->valid_constraints);
            }

            bool manipulation_mp_t::execute()
            {
                //Read a graph from a file (specs->graph_deserialization_file), inform the graph and serialize it to a new file.
                if( specs->graph_deserialization_file != "" )
                {
                    if( deserialize_graph(specs->graph_deserialization_file) )
                    {
                        inform_graph(specs->get_poses());
                        if( specs->serialization_file != "" )
                        {
                            serialization_file = specs->serialization_file;
                        }
                        else
                        {
                            PRX_WARN_S("serialization_file is empty. The same graph_deserialization_file name will be used to store the informed graph.");
                            serialization_file = specs->graph_deserialization_file;
                        }

                        return serialize();
                    }
                    else
                    {
                        PRX_FATAL_S("The file " << specs->graph_deserialization_file << " does not contains a correct graph.");
                    }
                }
                    //When the manipulation motion planner will read an informed graph from a file to use it. 
                else
                {
                    deserialization_file = specs->deserialization_file;
                    deserialize();

                    std::vector<double> pos(specs->collision_object_space->get_dimension());
                    specs->collision_object_space->copy_point_to_vector(specs->get_poses()->at(0).second, pos);
                    pos[0] = 100;
                    pos[2] = -100;
                    specs->collision_object_space->set_from_vector(pos);

                    if( specs->object_space != NULL )
                    {
                        pos[0] = -100;
                        specs->object_space->set_from_vector(pos);
                    }

                    //Add the new points because of the query poses. 
                    std::vector<state_t*>& seeds = specs->get_seeds();
                    std::vector<bool>& valid_seeds = specs->get_validity();
                    for( unsigned i = 0; i < seeds.size(); ++i )
                    {
                        if( validity_checker->is_valid(seeds[i]) )
                        {
                            valid_seeds[i] = true;
                            add_node(seeds[i]);
                            update_k(num_vertices);
                        }
                        else
                        {
                            valid_seeds[i] = false;
                        }
                    }

#ifndef NDEBUG
                    for( unsigned i = 0; i < valid_seeds.size(); ++i )
                        PRX_ASSERT(valid_seeds[i]);

                    PRX_DEBUG_COLOR("query poses: " << specs->get_query_poses()->size() << "     poses: " << specs->get_poses()->size() << "     new_edges:" << new_edges.size(), PRX_TEXT_BROWN);
#endif
                    //Inform all the graph with the new poses from initial and target poses.
                    inform_graph(specs->get_query_poses());
                    //Inform the new edges with the already existed poses.                    
                    inform_edges(new_edges, specs->get_poses());
                }
                return true;
            }

            void manipulation_mp_t::resolve_query()
            {
                std::vector<space_point_t*> goals = in_query->get_goal()->get_goal_points();
                std::vector<undirected_vertex_index_t> v_goals;

                if( in_query->q_type == motion_planning_query_t::PRX_ADD_QUERY_POINTS_NO_COLLISIONS )
                    no_collision_query_type = true;
                else if( in_query->q_type == motion_planning_query_t::PRX_NEAR_QUERY_POINTS )
                    near_query_type = true;

                undirected_vertex_index_t v_start;
                boost::tie(remove_start, v_start) = add_node(in_query->get_start_state());

                undirected_vertex_index_t v_g;
                bool remove_goal;

                foreach(space_point_t* g, goals)
                {
                    boost::tie(remove_goal, v_g) = add_node(g);
                    v_goals.push_back(v_g);
                    remove_goals.push_back(remove_goal);
                }

                bool good_to_go = false;
                for( unsigned i = 0; i < v_goals.size(); ++i )
                {
                    if( graph.components[v_start] == graph.components[v_goals[i]] )
                    {
                        good_to_go = true;
                        break;
                    }
                }

                if( !good_to_go )
                    return;

                astar->link_graph(&graph);
                std::deque< undirected_vertex_index_t > path_vertices;
                if( in_query->q_collision_type == motion_planning_query_t::PRX_LAZY_COLLISIONS )
                {
                    bool found_path = false;
                    int runs = 0;
                    PRX_ASSERT(lazy_runs != 0);
                    sys_clock_t clock;
                    clock.reset();
                    astar->set_astar_mode(astar_module_t::PRX_NO_EDGE_INFO);
                    while( !found_path && runs < lazy_runs )
                    {
                        runs++;
                        if( astar->solve(v_start, v_goals) )
                        {
                            found_path = true;
                            path_vertices.clear();
                            astar->extract_path(v_start, astar->get_found_goal(), path_vertices);
                            PRX_DEBUG_COLOR("ASTAR FOUND A PATH size:" << path_vertices.size() << "   runs:" << runs, PRX_TEXT_BLUE);
                            bool once = true;
                            for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                            {
                                undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                                manipulation_edge_t* edge = graph.get_edge_as<manipulation_edge_t > (e);
                                if( !edge->is_valid(specs->valid_constraints) )
                                {
#ifndef NDEBUG
                                    if( once )
                                    {
                                        std::string const_str = "CONSTRAINTS: ";

                                        foreach(unsigned c, *(specs->valid_constraints))
                                        {
                                            const_str += int_to_str((int)c) + " , ";
                                        }
                                        PRX_DEBUG_COLOR(const_str, PRX_TEXT_CYAN);
                                        once = false;
                                    }
                                    PRX_DEBUG_COLOR("Edge with constraints: " << edge->print_constraints(), PRX_TEXT_MAGENTA);
#endif
                                    found_path = false;
                                    astar->block_edge(e);
                                }

                            }
                        }
                        else
                        {
                            //There is no path and we have to stop searching
                            PRX_DEBUG_COLOR("No solution to astar", PRX_TEXT_RED);
                            runs = lazy_runs;
                            break;
                        }
                    }

                    if( runs >= lazy_runs && !found_path )
                    {
                        astar->restart();
                        if( astar->solve(v_start, v_goals) )
                        {
                            found_path = true;
                            path_vertices.clear();
                            astar->extract_path(v_start, astar->get_found_goal(), path_vertices);
                            PRX_DEBUG_COLOR("ASTAR FOUND A PATH size:" << path_vertices.size() << "   runs:" << runs, PRX_TEXT_RED);
                        }
                    }

                    in_query->clear();
                    if( found_path )
                    {
                        for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                        {
                            undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                            manipulation_edge_t* edge = graph.get_edge_as<manipulation_edge_t > (e);
                            edge->get_valid_constraints(in_query->constraints, specs->valid_constraints);
                            in_query->full_constraints.insert(edge->constraints.begin(), edge->constraints.end());
                            path1.clear();
                            new_plan.clear();
                            local_planner->steer(graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, path1);
                            in_query->plan += new_plan;
                        }
                        in_query->solution_cost = oa_astar->get_path_distance() + in_query->constraints.size() * oa_astar->get_collision_penalty();
                    }
#ifndef NDEBUG
                    PRX_DEBUG_COLOR("", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("====    Found path : " << found_path << "   size: " << in_query->plan.size() << "   |c|: " << in_query->constraints.size() << "   in " << clock.measure() << " sec    ====", PRX_TEXT_RED);
                    if( in_query->constraints.size() != 0 )
                    {
                        std::string const_str = "CONSTRAINTS: ";

                        foreach(unsigned c, in_query->constraints)
                        {
                            const_str += int_to_str((int)c) + " , ";
                        }
                        PRX_DEBUG_COLOR(const_str, PRX_TEXT_RED);
                    }
                    PRX_DEBUG_COLOR("================================================================\n", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("", PRX_TEXT_BROWN);
#endif                    
                }
                else if( in_query->q_collision_type == motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES )
                {
                    astar->set_astar_mode(astar_module_t::PRX_REUSE_EDGE_INFO);
                    if( astar->solve(v_start, v_goals) )
                    {
                        astar->extract_path(v_start, astar->get_found_goal(), path_vertices);
                        PRX_DEBUG_COLOR("ASTAR FOUND A PATH size:" << path_vertices.size(), PRX_TEXT_BLUE);

                        for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                        {
                            undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                            manipulation_edge_t* edge = graph.get_edge_as<manipulation_edge_t > (e);
                            edge->get_valid_constraints(in_query->constraints, specs->valid_constraints);
                            in_query->full_constraints.insert(edge->constraints.begin(), edge->constraints.end());
                            path1.clear();
                            new_plan.clear();
                            local_planner->steer(graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, path1);
                            in_query->plan += new_plan;
                        }
                        in_query->solution_cost = oa_astar->get_path_distance() + in_query->constraints.size() * oa_astar->get_collision_penalty();

                    }
                    else
                    {
                        //There is no path and we have to stop searching
                        PRX_DEBUG_COLOR("No solution to astar", PRX_TEXT_RED);
                    }

                }
                //                else
                //                {
                //                    PRX_ERROR_S("This method is not tested yet!!!");
                //                    prm_star_t::resolve_query();
                //                    if( in_query->plan.size() > 0 )
                //                    {
                //                        in_query->solution_cost = oa_astar->get_path_distance();
                //                        oa_astar->get_path_constraints(in_query->constraints);
                //                    }
                //                }

                if( remove_start )
                {

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v_start, graph.graph))
                    {
                        undirected_edge_index_t e = boost::edge(v_start, u, graph.graph).first;
                        graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                        e = boost::edge(u, v_start, graph.graph).first;
                        graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                    }

                    metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_start));
                    graph.clear_and_remove_vertex(v_start);
                    num_vertices--;
                }

                for( size_t i = 0; i < v_goals.size(); ++i )
                {
                    if( remove_goals[i] )
                    {
                        v_g = v_goals[i];

                        foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v_g, graph.graph))
                        {
                            undirected_edge_index_t e = boost::edge(v_g, u, graph.graph).first;
                            graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                            e = boost::edge(u, v_g, graph.graph).first;
                            graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                        }
                        metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_g));
                        graph.clear_and_remove_vertex(v_g);
                        num_vertices--;
                    }
                }

                v_goals.clear();
                remove_goals.clear();

                no_collision_query_type = false;
                near_query_type = false;
            }

            bool manipulation_mp_t::succeeded() const
            {
                if( input_specification->get_stopping_criterion()->satisfied() )
                    return true;
                return false;
            }

            std::pair<bool, util::undirected_vertex_index_t > manipulation_mp_t::add_node(const space_point_t * n_state)
            {
                if( metric->get_nr_points() > 0 )
                {
                    const manipulation_node_t* node = metric->single_query(n_state)->as<manipulation_node_t > ();
                    if( node != NULL && (near_query_type || metric->distance_function(n_state, node->point) <= similarity_threshold) )
                    {
                        PRX_DEBUG_COLOR("The point is already in the graph : " << state_space->print_point(n_state, 4), PRX_TEXT_BROWN);
                        return std::make_pair(false, node->index);
                    }
                }

                v_new = graph.add_vertex<manipulation_node_t > ();
                num_vertices++;
                graph.get_vertex_as<manipulation_node_t > (v_new)->init_node(state_space, n_state);
                //                PRX_INFO_S ("New node: " << state_space->print_point(graph[v_new]->point));

                if( delta_prm )
                    connect_node(v_new, r_n);

                else
                    connect_node(v_new);

                return std::make_pair(true, v_new);
            }

            void manipulation_mp_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
            {
                const undirected_node_t* node;

                path1.clear();
                for( size_t i = 0; i < neighbors.size(); i++ )
                {
                    node = neighbors[i]->as< undirected_node_t > ();
                    new_plan.clear();

                    local_planner->steer(graph[v]->point, node->point, new_plan, path1);
                    //If the path is valid
                    if( new_plan.size() != 0 && is_valid_trajectory(path1) )
                    {
                        //Add the edge to the graph
                        double dist = metric->distance_function(graph[v]->point, node->point);
                        undirected_edge_index_t e = graph.add_edge< manipulation_edge_t > (v, node->index, dist);
                        graph.get_edge_as<manipulation_edge_t > (e)->plan = new_plan;
                        graph.get_edge_as<manipulation_edge_t > (e)->id = num_edges;
                        num_edges++;
                        new_edges.push_back(e);

                        if( visualize_graph )
                            graph.get_edge_as< manipulation_edge_t > (e)->path = path1;
                    }
                    path1.clear();
                }
            }

            void manipulation_mp_t::inform_graph(const std::vector< std::pair<unsigned, sim::state_t*> >* poses)
            {
                PRX_DEBUG_COLOR("Informing the graph", PRX_TEXT_CYAN);
                int cc = boost::connected_components(graph.graph, graph.components);
                std::vector<unsigned> cc_num(cc);

                foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
                {
                    cc_num[graph.components[v]]++;
                }
                std::string cc_str = "";
                for( unsigned i = 0; i < cc; ++i )
                    cc_str += int_to_str(cc_num[i]) + " , ";
                PRX_DEBUG_COLOR("With " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges ... " << cc << " connected components   " << cc_str, PRX_TEXT_CYAN);
                std::vector<double> pos;
                if( specs->object_space != NULL )
                    pos.resize(specs->object_space->get_dimension());

                for( unsigned p = 0; p < poses->size(); ++p )
                {
                    specs->collision_object_space->copy_from_point(poses->at(p).second);

                    foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                    {
                        path1.clear();
                        new_plan.clear();
                        if( specs->object_space != NULL )
                        {
                            state_space->copy_from_point(graph[boost::source(e, graph.graph)]->point);
                            specs->_manipulator->get_end_effector_position(pos);
                            specs->object_space->set_from_vector(pos);
                        }
                        local_planner->steer(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point, new_plan, path1);
                        PRX_ASSERT(dynamic_cast<system_name_validity_checker_t*>(validity_checker) != NULL);
                        if( !validity_checker->is_valid(path1) )
                        {
                            graph.get_edge_as<manipulation_edge_t > (e)->add_constraint(poses->at(p).first);
                        }
                    }
                }
            }

            void manipulation_mp_t::inform_edges(const std::vector<undirected_edge_index_t>& edges, const std::vector< std::pair<unsigned, sim::state_t*> >* poses)
            {
                PRX_DEBUG_COLOR("Inform Edges", PRX_TEXT_CYAN);
                std::vector<double> pos;
                if( specs->object_space != NULL )
                    pos.resize(specs->object_space->get_dimension());

                for( unsigned p = 0; p < poses->size(); ++p )
                {
                    specs->collision_object_space->copy_from_point(poses->at(p).second);

                    foreach(undirected_edge_index_t e, edges)
                    {
                        path1.clear();
                        new_plan.clear();
                        if( specs->object_space != NULL )
                        {
                            state_space->copy_from_point(graph[boost::source(e, graph.graph)]->point);
                            specs->_manipulator->get_end_effector_position(pos);
                            specs->object_space->set_from_vector(pos);
                        }
                        local_planner->steer(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point, new_plan, path1);
                        if( !validity_checker->is_valid(path1) )
                        {
                            graph.get_edge_as<manipulation_edge_t > (e)->add_constraint(poses->at(p).first);
                        }
                    }
                }
            }

            bool manipulation_mp_t::serialize()
            {
                PRX_DEBUG_COLOR(" Inside manipulation motion planner serialization now, saving to file: " << serialization_file, PRX_TEXT_CYAN);
                std::ofstream fout(serialization_file.c_str());
                PRX_ASSERT(fout.is_open());
                graph.serialize(fout, state_space);
                fout.close();
                return true;

            }

            bool manipulation_mp_t::deserialize()
            {
                PRX_DEBUG_COLOR(" Inside manipulation motion planner deserialization now, opening file: " << deserialization_file, PRX_TEXT_CYAN);
                std::ifstream fin;
                if( !graph.deserialize<manipulation_node_t, manipulation_edge_t > (deserialization_file, fin, state_space) )
                {
                    PRX_FATAL_S("File could not deserialize!");
                    return false;
                }
                int counter = 0;
                //    int blah;

                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    double dist = metric->distance_function(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point);
                    graph.set_weight(e, dist);
                    graph.get_edge_as<manipulation_edge_t > (e)->id = counter;
                    graph.get_edge_as<manipulation_edge_t > (e)->plan.link_control_space(this->control_space);
                    graph.get_edge_as<manipulation_edge_t > (e)->path.link_space(this->state_space);
                    counter++;
                }

                double val_mu = (double)boost::num_edges(graph.graph) / (double)boost::num_vertices(graph.graph);
                double diff;
                double val_dev = 0.0;

                foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                {
                    metric->add_point(graph[nd]);
                    diff = (double)boost::out_degree(nd, graph.graph) - val_mu;
                    val_dev += diff * diff;
                }
                val_dev = sqrt(val_dev);
                update_k(boost::num_vertices(graph.graph));
                fin.close();

                int cc = boost::connected_components(graph.graph, graph.components);
                PRX_DEBUG_COLOR("Deserialized roadmap with " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges ... " << cc << " connected components", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("Average Valence: " << val_mu << "   Valence Deviation: " << val_dev, PRX_TEXT_GREEN);

                return true;
            }

            bool manipulation_mp_t::deserialize_graph(std::string graph_file)
            {
                std::ifstream input_stream(graph_file.c_str());
                if( input_stream.good() )
                {
                    int num_edges, num_vertices;
                    input_stream >> num_vertices;
                    hash_t<int, undirected_vertex_index_t> node_map;
                    for( int i = 0; i < num_vertices; i++ )
                    {
                        undirected_vertex_index_t new_vertex = graph.add_vertex<manipulation_node_t > ();
                        manipulation_node_t* node = graph.get_vertex_as<manipulation_node_t > (new_vertex);
                        node->point = state_space->alloc_point();
                        node->deserialize(input_stream, state_space);
                        node_map[i] = new_vertex;
                    }
                    input_stream >> num_edges;
                    int counter = 0;
                    for( int i = 0; i < num_edges; i++ )
                    {
                        int from, to;
                        input_stream >> from >> to;
                        undirected_edge_index_t e = graph.add_edge<manipulation_edge_t > (node_map[from], node_map[to]);
                        double dist = metric->distance_function(graph[node_map[from]]->point, graph[node_map[to]]->point);
                        graph.set_weight(e, dist);
                        manipulation_edge_t* edge = graph.get_edge_as<manipulation_edge_t > (e);
                        edge->id = counter++;
                        edge->plan.link_control_space(this->control_space);
                        edge->path.link_space(this->state_space);
                    }

                    PRX_ASSERT(boost::num_vertices(graph.graph) == num_vertices);
                    PRX_ASSERT(boost::num_edges(graph.graph) == num_edges);
                    double val_mu = num_edges / num_vertices;
                    double diff;
                    double val_dev = 0.0;

                    foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                    {
                        metric->add_point(graph[nd]);
                        diff = (double)boost::out_degree(nd, graph.graph) - val_mu;
                        val_dev += diff * diff;
                    }
                    val_dev = sqrt(val_dev);
                    update_k(num_vertices);
                    input_stream.close();

                    PRX_DEBUG_COLOR("Deserialized roadmap with " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges.", PRX_TEXT_MAGENTA);
                    PRX_DEBUG_COLOR("Average Valence: " << val_mu << "   Valence Deviation: " << val_dev, PRX_TEXT_GREEN);

                    return true;
                }
                return false;
            }
        }
    }
}
