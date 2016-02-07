/**
 * @file manipulation_graph.hpp
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
#pragma once

#ifndef PRX_MANIPULATION_GRAPH_HPP
#define	PRX_MANIPULATION_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_graph.hpp"

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * @anchor manipulation_node_t
             *
             * Manipulation node is just a PRM* node.
             *
             * @brief <b> Node class used by the manipulation planning structure. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulation_node_t : public plan::prm_star_node_t
            {

              public:

                virtual ~manipulation_node_t(){ }
                
                double shortest_distance;
                int constraints_size;
            };

            /**
             * @anchor manipulation_edge_t
             *
             * Edges in the manipulation planning structure store everything that the PRM edge
             * will store. Moreover, will store the constraints for the edge.
             *
             * @brief <b> Edge class used in the manipulation planning structure. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulation_edge_t : public plan::prm_star_edge_t
            {

              public:
                /** @brief The cost associated with this edge. */
                std::set<unsigned> constraints;

                virtual ~manipulation_edge_t(){ }

                virtual void add_constraints(std::set<unsigned>& constraints)
                {
                    this->constraints.insert(constraints.begin(), constraints.end());
                }

                virtual void add_constraint(unsigned pose_id)
                {
                    constraints.insert(pose_id);
                }

                virtual bool is_valid()
                {
                    return constraints.size() == 0;
                }

                virtual bool is_valid(const std::set<unsigned>* valid_constraints)
                {

                    foreach(unsigned c, constraints)
                    {
                        if( valid_constraints->find(c) != valid_constraints->end() )
                            return false;
                    }
                    return true;
                }
                
                virtual void get_valid_constraints(std::set<unsigned>& edge_constraints, const std::set<unsigned>* valid_constraints)
                {
                    foreach(unsigned c, constraints)
                    {
                        if( valid_constraints->find(c) != valid_constraints->end() )
                            edge_constraints.insert(c);
                    }
                }

                virtual void serialize(std::ofstream& output_stream)
                {
                    prm_star_edge_t::serialize(output_stream);
                    output_stream << std::endl << constraints.size() << std::endl;

                    foreach(unsigned c, constraints)
                    {
                        output_stream << c << " ";
                    }
                }

                virtual void deserialize(std::ifstream& input_stream)
                {
                    unsigned size;
                    unsigned constraint;
                    std::string name_constraint;
                    input_stream >> size;
                    for( unsigned i = 0; i < size; ++i )
                    {
                        input_stream >> constraint;
                        constraints.insert(constraint);
                    }
                }

                virtual std::string print_constraints() const
                {
                    std::stringstream output(std::stringstream::out);

                    foreach(unsigned i, constraints)
                    {
                        output << i << " , ";
                    }
                    return output.str();
                }

            };
        }
    }
}

#endif
