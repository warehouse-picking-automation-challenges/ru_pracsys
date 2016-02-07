/**
 * @file dprm_motion_planner.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "dprm_motion_planner.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"

#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>

#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS( prx::packages::dynamic_prm::dprm_motion_planner_t, prx::plan::planner_t)



namespace prx
{
    namespace packages
    {
        namespace dynamic_prm
        {

            dprm_motion_planner_t::dprm_motion_planner_t() : graphBuilt(false)
            {
                PRX_WARN_S("Creating a DPRM motion planner");
            }

            dprm_motion_planner_t::~dprm_motion_planner_t() { }

            bool dprm_motion_planner_t::updateObstacles(util::hash_t<std::string, sim::plant_t*> & plants)
            {
                util::hash_t<std::string, sim::plant_t*>::iterator it;
                util::space_point_t *plantState;
                bool replanningNeeded = false;
                double edgeLength;
                util::directed_edge_index_t e;
                plan::prm_star_edge_t *edge;

                /*for(it = plants.begin(); it != plants.end(); ++it)
                {
                        if(it->first != "manipulator")
                        {
                                plantState = it->second->get_state_space()->alloc_point();
						
                                foreach(e, pathEdges)
                                {
                                        edge = graph.get_edge_as<plan::prm_star_edge_t>(e);
							
                                        if(isColliding(plantState, edge, edgeLength))
                                        {
                                                PRX_ERROR_S("A plant is is colliding with a path edge!");
                                                graph.weights[e] = PRX_INFINITY;
                                                replanningNeeded = true;
                                        }
                                        else
                                        {
                                                graph.weights[e] = edgeLength;
                                        }
                                }
						
                                it->second->get_state_space()->free_point(plantState);
                        }
                }//for (plants)
                 */

                typedef boost::graph_traits<util::directed_graph_type>::edge_iterator EdgeIterator;
                typedef std::pair<EdgeIterator, EdgeIterator> EdgePair;

                EdgePair ep;
                util::directed_vertex_index_t u, v;
                //int blockedEdges = 0, openEdges = 0;

                for( ep = edges(graph.graph); ep.first != ep.second; ++ep.first )
                {
                    edge = graph.get_edge_as<plan::prm_star_edge_t > (*(ep.first));

                    for( it = plants.begin(); it != plants.end(); ++it )
                    {
                        //if(it->first != "manipulator")
                        //{
                        plantState = it->second->get_state_space()->alloc_point();

                        if( isColliding(plantState, edge, edgeLength) )
                        {
                            //PRX_ERROR_S("A plant is is colliding with a roadmap edge!");
                            //if(distanceToSystem2(plantState) < 50.0 * 50.0)
                            //{
                            graph.weights[*(ep.first)] = PRX_INFINITY;
                            replanningNeeded = true;
                            //}
                            break;
                        }
                        else
                        {
                            graph.weights[*(ep.first)] = edgeLength;
                        }

                        it->second->get_state_space()->free_point(plantState);
                        //}
                    }

                    //if(graph.weights[*(ep.first)] == PRX_INFINITY)
                    //	++blockedEdges;
                    //else
                    //	++openEdges;
                }

                //PRX_WARN_S("Processed " << blockedEdges << " blocked edges and " << openEdges << " open edges.");

                return replanningNeeded;
            }//updateObstacles()

            void dprm_motion_planner_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                prm_star_t::init(reader, template_reader);
            }

            void dprm_motion_planner_t::setup()
            {
                prm_star_t::setup();

                systemState = state_space->alloc_point();

                previousPlan.link_control_space(control_space);
            }

            double dprm_motion_planner_t::distanceToSystem2(sim::state_t *state)
            {
                double dx, dy;
                state_space->copy_to_point(systemState);
                dx = systemState->at(0) - state->at(0);
                dy = systemState->at(1) - state->at(1);
                return dx * dx + dy*dy;
            }

            bool dprm_motion_planner_t::isColliding(util::space_point_t *plantState, plan::prm_star_edge_t *edge, double& edgeLength)
            {
                util::directed_node_t *source, *target;
                util::directed_vertex_index_t u, v;
                double x1, y1, x2, y2, px, py;
                double length2, t, projx, projy;
                double dist;
                const double threshold = 70.0;

                u = boost::source(edge->index, graph.graph);
                v = boost::target(edge->index, graph.graph);
                source = graph.get_vertex_as<util::directed_node_t > (u);
                target = graph.get_vertex_as<util::directed_node_t > (v);
                edgeLength = metric->distance_function(source->point, target->point);
                x1 = source->point->at(0);
                y1 = source->point->at(1);
                x2 = target->point->at(0);
                y2 = target->point->at(1);
                px = plantState->at(0);
                py = plantState->at(1);

                length2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

                if( length2 == 0.0 )
                    return (sqrt((x1 - px) * (x1 - px) + (y1 - py) * (y1 - py)) < threshold);

                t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / length2;

                if( t < 0.0 )
                    return (sqrt((x1 - px) * (x1 - px) + (y1 - py) * (y1 - py)) < threshold);

                if( t > 1.0 )
                    return (sqrt((x2 - px) * (x2 - px) + (y2 - py) * (y2 - py)) < threshold);

                projx = x1 + t * (x2 - x1);
                projy = y1 + t * (y2 - y1);

                dist = sqrt((projx - px) * (projx - px) + (projy - py) * (projy - py));

                //TODO: Make this detect obstacle radius at some point so I don't have to hard-code it.
                return (dist < threshold);
            }//isColliding()

            void dprm_motion_planner_t::resolve_query()
            {
                util::sys_clock_t resolve_timer;
                resolve_timer.reset();
                std::vector<util::space_point_t*> goals = input_query->get_goal()->get_goal_points();
                std::vector<util::directed_vertex_index_t> v_goals;
                plan::prm_star_edge_t *currentEdge;
                bool pathFound = false;
                sim::state_t *startState = state_space->clone_point(input_query->get_start_state());

                input_query->plan.clear();

                if( input_query->get_goal()->satisfied(input_query->get_start_state()) )
                    return;

                /*state_space->copy_from_point(input_query->get_start_state());
                foreach(util::directed_vertex_index_t v, boost::vertices(graph.graph))
                {
                        std::vector<double> vec;
                        for(unsigned i=0;i<state_space->get_dimension();i++)
                        {
                                vec.push_back(graph[v]->point->at(i));
                        }
                        state_space->set_from_vector(vec);
                        state_space->copy_to_point(graph[v]->point);
                }*/



                //Prediction method.
                //Check needed because an assert in trim() breaks my code.
                if( predictedTime > 0.0 )
                    previousPlan.trim(predictedTime);

                local_planner->propagate_step(input_query->get_start_state(), previousPlan, startState);





                util::directed_vertex_index_t v_start = add_node(startState);
                PRX_INFO_S("Edges to start: " << boost::out_degree(v_start, graph.graph));
                PRX_ERROR_S("Start state: " << state_space->print_point(startState));

                foreach(util::space_point_t* g, goals)
                {
                    PRX_ERROR_S("Added goal: " << state_space->print_point(g));
                    v_goals.push_back(add_node(g));
                }

                /*query_heuristic = new plan::default_prm_star_distance_heuristic(metric);
                query_visitor = new plan::default_prm_star_astar_visitor();
				
                query_heuristic->set(&graph, v_goals);
                query_visitor->set_new_goals(&v_goals);
                try
                {
                        util::astar_search<
                                        util::directed_graph_type,
                                        util::directed_graph_t,
                                        util::directed_vertex_index_t,
                                        plan::default_prm_star_distance_heuristic,
                                        plan::default_prm_star_astar_visitor
                                        > (graph, v_start, query_heuristic, query_visitor);
                }
                catch(plan::prm_star_found_goal fg)
                {
                        pathFound = true;
                        std::deque<util::directed_vertex_index_t> path_vertices;
                        for(util::directed_vertex_index_t v = fg.v_goal; v != v_start; v = graph.predecessors[v] )
                        {
                                path_vertices.push_front(v);
                                PRX_DEBUG_COLOR("v : " << state_space->print_point(graph[v]->point), PRX_TEXT_MAGENTA);
                        }
                        path_vertices.push_front(v_start);
					
                        for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                        {
                                util::directed_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                                sim::plan_t* ctrls = &graph.get_edge_as<plan::prm_star_edge_t>(e)->plan;
						
                                foreach(sim::plan_step_t step, *ctrls)
                                {
                                        PRX_INFO_S("step duration for the plan : " << step.duration << "    control : " << control_space->print_point(step.control, 2));
                                        input_query->plan.copy_onto_back(step.control, step.duration);
                                }
                                input_query->path += graph.get_edge_as<plan::prm_star_edge_t>(e)->path;
                                pathEdges.push_back(e);
                        }//for
                }//catch
                 */

                astarSearch.set_graph(&graph);
                astarSearch.setMetric(metric);
                if( astarSearch.solve(v_start, v_goals) )
                {
                    pathFound = true;
                    std::deque<util::directed_vertex_index_t> path_vertices;
                    astarSearch.extract_path(v_start, astarSearch.get_found_goal(), path_vertices);

                    for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                    {
                        util::directed_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                        sim::plan_t* ctrls = &graph.get_edge_as<plan::prm_star_edge_t > (e)->plan;

                        foreach(sim::plan_step_t step, *ctrls)
                        {
                            //PRX_INFO_S("step duration for the plan : " << step.duration << "    control : " << control_space->print_point(step.control, 2));
                            input_query->plan.copy_onto_back(step.control, step.duration);
                        }
                        input_query->path += graph.get_edge_as<plan::prm_star_edge_t > (e)->path;
                        pathEdges.push_back(e);
                    }//for
                }

                PRX_INFO_S("Path found: " << pathFound);

                foreach(util::directed_vertex_index_t u, boost::adjacent_vertices(v_start, graph.graph))
                {
                    util::directed_edge_index_t e = boost::edge(v_start, u, graph.graph).first;
                    graph.get_edge_as<plan::prm_star_edge_t > (e)->clear(control_space);
                    e = boost::edge(u, v_start, graph.graph).first;
                    graph.get_edge_as<plan::prm_star_edge_t > (e)->clear(control_space);
                }

                metric->remove_point(graph.get_vertex_as<util::directed_node_t > (v_start));
                graph.clear_and_remove_vertex(v_start);

                foreach(util::directed_vertex_index_t v, v_goals)
                {

                    foreach(util::directed_vertex_index_t u, boost::adjacent_vertices(v, graph.graph))
                    {
                        util::directed_edge_index_t e = boost::edge(v, u, graph.graph).first;
                        graph.get_edge_as<plan::prm_star_edge_t > (e)->clear(control_space);
                        e = boost::edge(u, v, graph.graph).first;
                        graph.get_edge_as<plan::prm_star_edge_t > (e)->clear(control_space);
                    }
                    metric->remove_point(graph.get_vertex_as<util::directed_node_t > (v));
                    graph.clear_and_remove_vertex(v);
                }

                //delete query_heuristic;
                //delete query_visitor;

                previousPlan = input_query->plan;
                state_space->free_point(startState);
            }//resolve_query()

            bool dprm_motion_planner_t::execute()
            {
                try
                {
                    motion_planner_t::execute();
                }
                catch( plan::stopping_criteria_t::stopping_criteria_satisfied& ex )
                {
                    graphBuilt = true;
                    throw;
                }

                return true;
            }

            void dprm_motion_planner_t::step()
            {
                prm_star_t::step();
                PRX_INFO_S("Motion planner stepped.");
            }

            bool dprm_motion_planner_t::isGraphBuilt() const
            {
                return graphBuilt;
            }

            void dprm_motion_planner_t::setPredictedTime(double t)
            {
                //PRX_WARN_S("Predicted time: " << t);
                predictedTime = t;
            }

            void dprm_motion_planner_t::setWorldModel(plan::world_model_t *wm)
            {
                worldModel = wm;
            }

            void dprm_motion_planner_t::update_vis_info() const
            {
                //PRX_WARN_S("visualization_bodies.size() : " << visualization_bodies.size() << "     visualize_graph : " << visualize_graph);
                if( visualization_bodies.size() <= 0 )
                    return;

                std::vector<util::geometry_info_t> geoms;
                std::vector<util::config_t> configs;
                util::hash_t<std::string, std::vector<double> > map_params;
                std::vector<double> params;
                int count;

                if( visualize_graph )
                {
                    count = 0;
                    std::vector<std::string> system_names;
                    system_names.push_back(visualization_bodies[0]);
                    //PRX_INFO_S("visualize_graph : " << visualize_graph << "     systems_name size: " << system_names.size() << "    name: " << system_names[0]);

                    foreach(util::directed_edge_index_t e, boost::edges(graph.graph))
                    {
                        std::string name = ros::this_node::getName() + visualization_graph_name + "/edge_" + util::int_to_str(count);
                        params.clear();

                        //PRX_WARN_S("edge size: " << graph.get_edge_as<plan::prm_star_edge_t> (e)->path.size());

                        map_params.clear();
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(graph[boost::source(e, graph.graph)]->point, system_names, map_params);
                        params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                        map_params.clear();
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(graph[boost::target(e, graph.graph)]->point, system_names, map_params);
                        params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                        if( graph.weights[e] == PRX_INFINITY )
                            geoms.push_back(util::geometry_info_t(visualization_bodies[0], name, util::PRX_LINESTRIP, params, "red"));
                        else
                            geoms.push_back(util::geometry_info_t(visualization_bodies[0], name, util::PRX_LINESTRIP, params, graph_color));

                        configs.push_back(util::config_t());
                        count++;
                    }

                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_graph_name] = geoms;
                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_graph_name] = configs;
                    geoms.clear();
                    configs.clear();
                }

                if( visualize_solutions && input_query->path.size() > 0 )
                {
                    util::hash_t<std::string, std::vector<double> > to_map_params;
                    count = 0;
                    for( size_t i = 0; i < input_query->path.size() - 1; i++ )
                    {
                        map_params.clear();
                        to_map_params.clear();

                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(input_query->path[i], visualization_bodies, map_params);
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(input_query->path[i + 1], visualization_bodies, to_map_params);

                        for( size_t s = 0; s < visualization_bodies.size(); s++ )
                        {
                            params.clear();
                            params.insert(params.end(), map_params[visualization_bodies[s]].begin(), map_params[visualization_bodies[s]].end());
                            params.insert(params.end(), to_map_params[visualization_bodies[s]].begin(), to_map_params[visualization_bodies[s]].end());

                            std::string name = ros::this_node::getName() + visualization_solutions_name + "/" + visualization_bodies[s] + "/path_" + util::int_to_str(count);

                            geoms.push_back(util::geometry_info_t(visualization_bodies[s], name, util::PRX_LINESTRIP, params, solutions_colors[s % solutions_colors.size()]));
                            configs.push_back(util::config_t());
                            count++;
                        }
                    }

                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_solutions_name] = geoms;
                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_solutions_name] = configs;
                    geoms.clear();
                    configs.clear();
                }
            }//update_vis_info()
        }//namespace dynamic_prm
    }//namespace packages
}//namespace prx
