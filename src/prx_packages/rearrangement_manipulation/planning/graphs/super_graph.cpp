/**
 * @file super_graph.cpp
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

#include <vector>
#include <set>
#include <algorithm>
#include <boost/graph/compressed_sparse_row_graph.hpp>

#include "planning/graphs/super_graph.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "boost/graph/connected_components.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace rearrangement_manipulation
        {
            // =============
            //  H.Node Data
            // =============

            hnode_data_t::hnode_data_t() { }

            hnode_data_t::hnode_data_t(const hnode_data_t& other)
            {
                *this = other;
            }

            hnode_data_t::hnode_data_t(util::undirected_graph_t* pebble_graph)
            {
                link_pebble_graph(pebble_graph);
            }

            hnode_data_t::~hnode_data_t()
            {
                p_graph = NULL;
                cc_data.clear();
                signature.clear();
                cc_sizes.clear();
                positions.clear();
            }

            void hnode_data_t::link_pebble_graph(util::undirected_graph_t* pebble_graph)
            {
                p_graph = pebble_graph;
                PRX_DEBUG_COLOR(" == Calling link pebble graph ==   p_graph:" << boost::num_vertices(p_graph->graph) << "   ingraph:" << boost::num_vertices(pebble_graph->graph), PRX_TEXT_CYAN);
                int num = boost::connected_components(p_graph->graph, p_graph->components);
                cc_sizes.clear();
                cc_data.clear();
                signature.clear();
                positions.clear();
                cc_sizes.resize(num, 0);
                cc_data.resize(num);

                //                //TODO: Delete these lines
                //                for( int i = 0; i < num; ++i )
                //                    PRX_ASSERT(cc_sizes[i] == 0);
                //                ////////// std::fill_n(cc_sizes.begin(), cc_sizes.end(), 0);

                foreach(util::undirected_vertex_index_t v, boost::vertices(p_graph->graph))
                {
                    cc_sizes[p_graph->components[v]]++;
                    int position = p_graph->get_vertex_as< pebble_node_t > (v)->position_index;
                    cc_data[p_graph->components[v]].insert(position);
                    positions.insert(position);
                }
            }

            void hnode_data_t::link_pebble_graph(util::undirected_graph_t* pebble_graph, const std::vector< unsigned >& insigma)
            {
                link_pebble_graph(pebble_graph);
                //link_pebble_graph is cleaning the signature vector.
                signature = insigma;
            }

            bool hnode_data_t::is_useful()
            {
                if( cc_sizes.size() != positions.size() )
                    for( unsigned i = 0; i < signature.size(); ++i )
                        if( signature[i] != 0 && cc_sizes[i] != signature[i] )
                            return true;

                return false;
            }

            std::vector< std::vector< unsigned > > hnode_data_t::generate_signatures(unsigned in_k) const
            {
                return generate_signatures(in_k, cc_sizes);
            }

            void hnode_data_t::generate_valid_arrangement(std::vector< unsigned >& arrangement) const
            {
                for( unsigned i = 0; i < signature.size(); ++i )
                {
                    unsigned sign = signature[i];
                    //If there are pebbles in this CC, do things
                    if( sign > 0 )
                    {
                        //Get the cc size
                        unsigned cc_size = cc_sizes[i];
                        //Also extract the cc_data into a random-accessible thing
                        std::vector< unsigned > cc_nodes(cc_data[i].begin(), cc_data[i].end());

                        //Let's generate the possible arrangements of the pebbles in this CC
                        std::vector< std::vector< unsigned > > poss_arr = generate_arrangements(cc_size, sign);
                        //Now, let's randomly select one
                        unsigned index = uniform_int_random(0, poss_arr.size() - 1);
                        //Now, for each element in that arrangement
                        for( unsigned t = 0; t < cc_size; ++t )
                        {
                            //If we are to add it
                            if( poss_arr[index][t] )
                            {
                                arrangement.push_back(cc_nodes[t]);
                            }
                        }
                    }
                }
            }

            void hnode_data_t::update_signature(const std::vector<unsigned>& positions)
            {
                //                for( int i = 0; i < signature.size(); ++i )
                //                    signature[i] = 0;
                int num = boost::connected_components(p_graph->graph, p_graph->components);
                if( signature.size() == 0 )
                    signature.resize(num, 0);
                else
                    std::fill(signature.begin(), signature.end(), 0);

                foreach(undirected_vertex_index_t v, boost::vertices(p_graph->graph))
                {
                    pebble_node_t* tmp_node = p_graph->get_vertex_as<pebble_node_t > (v);
                    if( std::find(positions.begin(), positions.end(), tmp_node->position_index) != positions.end() )
                        signature[p_graph->components[v]]++;
                }
            }

            void hnode_data_t::replace_signature(const std::vector<unsigned>& new_signature)
            {
                signature.clear();
                signature = new_signature;
            }

            const hnode_data_t& hnode_data_t::operator=(const hnode_data_t& other)
            {
                link_pebble_graph(other.p_graph, other.signature);
                positions.clear();
                positions = other.positions;
                return *this;
            }

            bool hnode_data_t::operator!=(const hnode_data_t& other)
            {
                return !operator==(other);
            }

            bool hnode_data_t::operator==(const hnode_data_t& other) // I think this need additional pylo... testing.
            {
                //First, do easy sanity checks: make sure signatures are at least the same size
                if( signature.size() != other.signature.size() )
                    return false;
                PRX_DEBUG_COLOR("--- Nodes have the same signature sizes: " << signature.size() << " - " << other.signature.size(), PRX_TEXT_CYAN);

                for( unsigned i = 0; i < signature.size(); ++i )
                    if( signature[i] != other.signature[i] )
                        return false;

                PRX_DEBUG_COLOR("--- Nodes have the same signatures: ", PRX_TEXT_CYAN);

                for( unsigned i = 0; i < signature.size(); ++i )
                {
                    PRX_DEBUG_COLOR("------ " << signature[i] << " - " << other.signature[i], PRX_TEXT_LIGHTGRAY);
                }

                //Next, vertex count
                if( boost::num_vertices(p_graph->graph) != boost::num_vertices(other.p_graph->graph) )
                {
                    PRX_DEBUG_COLOR("The two graphs have different numbers of nodes " << boost::num_vertices(p_graph->graph) << " versus " << boost::num_vertices(other.p_graph->graph), PRX_TEXT_MAGENTA);
                    PRX_FATAL_S("The two graphs have different numbers of nodes ");
                }

                PRX_DEBUG_COLOR("--- Nodes have the same #nodes: " << boost::num_vertices(p_graph->graph) << " - " << boost::num_vertices(other.p_graph->graph), PRX_TEXT_CYAN);

                int our_id;

                foreach(undirected_vertex_index_t v, boost::vertices(p_graph->graph))
                {
                    const pebble_node_t* our_node = p_graph->get_vertex_as< pebble_node_t > (v);
                    our_id = our_node->position_index;

                    //Now, we have to find such an id in our friend's graph
                    undirected_vertex_index_t vprime = other.get_vertex_index(our_id);

                    if( vprime == NULL )
                        return false;
                    PRX_DEBUG_COLOR("----- Position " << our_id << " exist in the other graph!", PRX_TEXT_CYAN);
                    //                    if( boost::in_degree( v, p_graph->graph ) != boost::in_degree( vprime, other.p_graph->graph ) )
                    //                        return false;
                    std::set< int > our_adjacent;
                    std::set< int > other_adjacent;

                    //Get all of our adjacent dudes in order

                    foreach(undirected_vertex_index_t d, boost::adjacent_vertices(v, p_graph->graph))
                    {
                        our_adjacent.insert(p_graph->get_vertex_as< pebble_node_t > (d)->position_index);
                    }
                    //Get all of his adjacent dudes in order

                    foreach(undirected_vertex_index_t d, boost::adjacent_vertices(vprime, other.p_graph->graph))
                    {
                        other_adjacent.insert(other.p_graph->get_vertex_as< pebble_node_t > (d)->position_index);
                    }
                    //Can this way check sizes of adjacent things
                    if( our_adjacent.size() != other_adjacent.size() )
                        return false;
                    //Then, iterate over these sets, and check if anything doesn't match
                    std::set< int >::iterator it = our_adjacent.begin();
                    std::set< int >::iterator oit = other_adjacent.begin();
                    for(; it != our_adjacent.end(); it++, oit++ )
                    {
                        if( *it != *oit )
                            return false;
                    }
                }

                return true;
            }

            bool hnode_data_t::is_equal(const hnode_data_t* other)
            {
                return *this == *other;
            }

            unsigned hnode_data_t::same_poses(std::set<unsigned> poses)
            {
                unsigned count = 0;

                foreach(unsigned p, poses)
                {
                    if( positions.count(p) )
                    {
                        ++count;
                    }
                }
                return count;
            }

            bool hnode_data_t::have_the_same_poses(std::set<unsigned> poses)
            {
                return positions == poses;
            }

            bool hnode_data_t::is_data_ok(std::string str) const
            {

                PRX_DEBUG_COLOR(" ======================================================== ", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("          Check Data for  " << str, PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR(" ======================================================== ", PRX_TEXT_RED);

                foreach(undirected_vertex_index_t v, boost::vertices(p_graph->graph))
                {
                    if( p_graph->get_vertex_as< pebble_node_t > (v)->point == NULL )// || p_graph->get_vertex_as< pebble_node_t > (v)->point->parent == NULL )
                        PRX_DEBUG_COLOR(p_graph->get_vertex_as< pebble_node_t > (v)->position_index << ": Has bad point" << v, PRX_TEXT_RED);
                }

                foreach(undirected_edge_index_t e, boost::edges(p_graph->graph))
                {
                    pebble_edge_t* pe = p_graph->get_edge_as< pebble_edge_t > (e);

                    int index = 0;

                    foreach(state_t* s, pe->path)
                    {
                        if( s == NULL )// || s->parent == NULL )
                            PRX_DEBUG_COLOR("Edge goes from:  " << pe->p_source << "  -to-  " << pe->p_target << "    path: " << pe->path.size() << "    plan: " << pe->plan.size() << "  has bad Trajectory at pose: " << index, PRX_TEXT_RED);
                        index++;
                    }

                    index = 0;

                    foreach(plan_step_t p, pe->plan)
                    {
                        if( p.control == NULL ) //|| p.control->parent == NULL )
                            PRX_DEBUG_COLOR("Edge goes from:  " << pe->p_source << "  -to-  " << pe->p_target << "    path: " << pe->path.size() << "    plan: " << pe->plan.size() << "  has bad Plan at pose: " << index, PRX_TEXT_RED);
                        index++;
                    }
                }
                return true;
            }

            std::string hnode_data_t::print_signature() const
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, signature)
                {
                    output << i << " , ";
                }
                return output.str();
            }

            std::string hnode_data_t::print_cc_data() const
            {
                std::stringstream output(std::stringstream::out);

                foreach(std::set< unsigned > d, cc_data)
                {
                    output << std::endl;

                    foreach(unsigned v, d)
                    {
                        output << v << " , ";
                    }
                }
                return output.str();
            }

            std::string hnode_data_t::print_positions() const
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, positions)
                {
                    output << i << " , ";
                }
                return output.str();
            }

            void hnode_data_t::print(std::string additional_string) const
            {
                //Debug: Print out the CC info
                PRX_DEBUG_COLOR(" ======================================================== ", PRX_TEXT_RED);
                if( additional_string.empty() )
                {
                    PRX_DEBUG_COLOR("          Hnode Print " << this, PRX_TEXT_BROWN);
                }
                else
                {
                    PRX_DEBUG_COLOR("       " << additional_string, PRX_TEXT_BROWN);
                }
                PRX_DEBUG_COLOR(" ======================================================== ", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("Node Signature:", PRX_TEXT_MAGENTA);

                foreach(unsigned i, signature)
                {
                    PRX_DEBUG_COLOR(i, PRX_TEXT_LIGHTGRAY);
                }
                PRX_DEBUG_COLOR("CC Size Data:", PRX_TEXT_MAGENTA);

                foreach(unsigned i, cc_sizes)
                {
                    PRX_DEBUG_COLOR(i, PRX_TEXT_LIGHTGRAY);
                }
                PRX_DEBUG_COLOR("Connected Components information:  " << cc_data.size(), PRX_TEXT_MAGENTA);

                foreach(std::set< unsigned > d, cc_data)
                {
                    PRX_DEBUG_COLOR("CC:", PRX_TEXT_CYAN);

                    foreach(unsigned v, d)
                    {
                        PRX_DEBUG_COLOR(v, PRX_TEXT_LIGHTGRAY);
                    }
                }
                PRX_DEBUG_COLOR("Position Membership information: ", PRX_TEXT_MAGENTA);

                foreach(unsigned p, positions)
                {
                    PRX_DEBUG_COLOR(p, PRX_TEXT_LIGHTGRAY);
                }
                PRX_DEBUG_COLOR("This graph has 'dis many nodes: " << boost::num_vertices(p_graph->graph), PRX_TEXT_MAGENTA);

                foreach(undirected_vertex_index_t v, boost::vertices(p_graph->graph))
                {
                    PRX_DEBUG_COLOR(p_graph->get_vertex_as< pebble_node_t > (v)->position_index << ": " << v, PRX_TEXT_LIGHTGRAY);
                }
                PRX_DEBUG_COLOR("This graph has 'dis many edges: " << boost::num_edges(p_graph->graph), PRX_TEXT_MAGENTA);

                foreach(undirected_edge_index_t e, boost::edges(p_graph->graph))
                {
                    pebble_edge_t* pe = p_graph->get_edge_as< pebble_edge_t > (e);
                    PRX_DEBUG_COLOR("Edge goes from:  " << pe->p_source << "  -to-  " << pe->p_target << "    path: " << pe->path.size() << "    plan: " << pe->plan.size(), PRX_TEXT_LIGHTGRAY);
                    PRX_DEBUG_COLOR("      pointers:  " << pe->v_source << "  -to-  " << pe->v_target, PRX_TEXT_LIGHTGRAY);
                    if( pe->constraints.size() > 0 )
                    {
                        PRX_DEBUG_COLOR("Edge also has constraints: ", PRX_TEXT_BROWN);

                        foreach(unsigned c, pe->constraints)
                        {
                            PRX_DEBUG_COLOR(" : " << c, PRX_TEXT_LIGHTGRAY);
                        }
                    }
                }
                PRX_DEBUG_COLOR(" ======================================================== ", PRX_TEXT_GREEN);
            }

            unsigned hnode_data_t::get_position_index(undirected_vertex_index_t v)
            {
                return p_graph->get_vertex_as< pebble_node_t > (v)->position_index;
            }

            undirected_vertex_index_t hnode_data_t::get_vertex_index(int pi) const
            {
                //                undirected_vertex_index_t v = get_vertex_index(p_graph, pi);
                //                if( v == NULL )
                //                    PRX_FATAL_S("Unknown vertex index for get position.");

                return get_vertex_index(p_graph, pi);
            }

            undirected_vertex_index_t hnode_data_t::get_vertex_index(undirected_graph_t* graph, int position) const
            {

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    //If the neighbor vertex has the same position index
                    if( position == (int)graph->get_vertex_as< pebble_node_t > (v)->position_index )
                    {
                        return v;
                    }
                }
                return NULL;
            }

            void hnode_data_t::copy_pebble_graph(undirected_graph_t& destination, const undirected_graph_t& source)
            {
                //                PRX_DEBUG_COLOR("Copying the graph...", PRX_TEXT_GREEN );                
                destination.clear();
                //Copy each vertex

                foreach(undirected_vertex_index_t vp, boost::vertices(source.graph))
                {
                    undirected_vertex_index_t v = destination.add_vertex< pebble_node_t > ();
                    pebble_node_t* new_node = destination.get_vertex_as< pebble_node_t > (v);
                    pebble_node_t* source_node = source.get_vertex_as< pebble_node_t > (vp);
                    //Assign the basic information:
                    *new_node = *source_node;
                    new_node->index = v;
                    //                    PRX_DEBUG_COLOR("Generated vertex: " << new_node.position_index << ": " << v, PRX_TEXT_LIGHTGRAY);
                } //I think this accomplishes copying over all the relevant data...
                //Now... now, my friend.  Copy over the edges. :(
                //                PRX_DEBUG_COLOR("Done with vertices...", PRX_TEXT_BROWN );
                std::vector< undirected_edge_index_t > source_graph_edges(0);

                foreach(undirected_edge_index_t ep, boost::edges(source.graph))
                {
                    source_graph_edges.push_back(ep);
                }

                //foreach( undirected_edge_index_t ep, boost::edges( source.graph ) )
                for( unsigned t = 0; t < source_graph_edges.size(); ++t )
                {
                    undirected_edge_index_t ep = source_graph_edges[t];
                    undirected_vertex_index_t from_prime = boost::source(ep, source.graph);
                    undirected_vertex_index_t to_prime = boost::target(ep, source.graph);

                    unsigned src = source.get_vertex_as< pebble_node_t > (from_prime)->position_index;
                    unsigned tgt = source.get_vertex_as< pebble_node_t > (to_prime)->position_index;

                    undirected_vertex_index_t from = NULL;
                    undirected_vertex_index_t to = NULL;

                    foreach(undirected_vertex_index_t v, boost::vertices(destination.graph))
                    {
                        if( destination.get_vertex_as< pebble_node_t > (v)->position_index == src )
                        {
                            from = v;
                            //                            PRX_DEBUG_COLOR("Found source... " << from, PRX_TEXT_CYAN );
                        }
                        if( destination.get_vertex_as< pebble_node_t > (v)->position_index == tgt )
                        {
                            to = v;
                            //                            PRX_DEBUG_COLOR("Found target... " << to, PRX_TEXT_CYAN );
                        }
                    }

                    //Also need to get the actual pebble edge
                    pebble_edge_t* source_pe = source.get_edge_as< pebble_edge_t > (ep);

                    //Add dat friggin' edge...
                    undirected_edge_index_t e = destination.add_edge< pebble_edge_t > (from, to);

                    //Now, get the actual pebble edge thing to fill it up...
                    pebble_edge_t* pe = destination.get_edge_as< pebble_edge_t > (e);

                    //Add the path and the plan on this edge
                    *pe = *source_pe;

                    //Fantastico!  Now use ours l;djaf NO, ASSIGN MANUALLY FOR SANITY.
                    pe->p_source = src;
                    pe->p_target = tgt;

                    //But alas, we must also put in the appropriate vertex_index-based informationz. D:
                    pe->v_source = from;
                    pe->v_target = to;

                    //Now, v_constraints.... *le sigh*
                    pe->constraints.clear();

                    foreach(unsigned cons, source_pe->constraints)
                    {

                        foreach(undirected_vertex_index_t v, boost::vertices(destination.graph))
                        {
                            if( destination.get_vertex_as< pebble_node_t > (v)->position_index == cons )
                            {
                                pe->constraints.insert(cons);
                            }
                        }
                    }//DONE SON
                    //                    PRX_DEBUG_COLOR("Generated edge: " << src << " - " << tgt << " :: total edges now: " << boost::num_edges( destination.graph ), PRX_TEXT_LIGHTGRAY);
                }
            }

            void hnode_data_t::set(util::undirected_graph_t& ingraph)
            {
                PRX_DEBUG_COLOR(" == Calling set ==   pebble graph:" << boost::num_vertices(p_graph->graph) << "   ingraph:" << boost::num_vertices(ingraph.graph), PRX_TEXT_CYAN);
                copy_pebble_graph(*p_graph, ingraph);
                int num = boost::connected_components(p_graph->graph, p_graph->components);
                cc_sizes.clear();
                cc_data.clear();
                signature.clear();
                cc_sizes.resize(num);
                cc_data.resize(num);

                foreach(util::undirected_vertex_index_t v, boost::vertices(p_graph->graph))
                {
                    cc_sizes[p_graph->components[v]]++;
                    cc_data[p_graph->components[v]].insert(p_graph->get_vertex_as< pebble_node_t > (v)->position_index);
                }
                signature.resize(num);

                foreach(undirected_vertex_index_t v, boost::vertices(p_graph->graph))
                {
                    positions.insert(p_graph->get_vertex_as< pebble_node_t > (v)->position_index);
                }
            }

            void hnode_data_t::set(util::undirected_graph_t& ingraph, const std::vector< unsigned >& insigma)
            {
                set(ingraph);
                signature = insigma;
            }

            // =============
            //  Hyper Node
            // =============

            super_node_t::super_node_t()
            {
                data = new hnode_data_t();
            }

            super_node_t::~super_node_t()
            {
                delete data;
            }

            // =============
            //  Hyper Edge
            // =============

            void super_edge_t::init_as_motion(undirected_vertex_index_t v, const std::set< unsigned >& in_poses, const std::pair< unsigned, unsigned >& in_motion, const sim::plan_t& new_plan)
            {
                s_vertex = v;
                has_motion = true;
                motion = in_motion;
                plan = new_plan;
                poses.insert(in_poses.begin(), in_poses.end());
            }

            void super_edge_t::init_as_switch(const std::set< unsigned >& in_poses)
            {
                has_motion = false;
                poses = in_poses;
            }

            unsigned super_edge_t::get_source_position(util::undirected_vertex_index_t v)
            {
                if( v == s_vertex )
                    return motion.first;
                return motion.second;
            }
            
            unsigned super_edge_t::get_the_other_position(unsigned pose)
            {
                if(motion.first == pose)
                    return motion.second;
                return motion.first;
            }

            void super_edge_t::get_plan(util::undirected_vertex_index_t v, sim::plan_t& computed_plan)
            {
                if( v == s_vertex )
                {
                    computed_plan += plan;
                }
                else
                {
                    for( int i = (int)plan.size() - 1; i >= 0; --i )
                    {
                        computed_plan.copy_onto_back(plan[i].control, plan[i].duration);

                    }
                }
            }
        }
    }
}



