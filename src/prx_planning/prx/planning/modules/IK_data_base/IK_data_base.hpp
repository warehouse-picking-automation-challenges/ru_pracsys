/**
 * @file IK_data_base.hpp 
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

#pragma once

#ifndef PRX_IK_DATA_BASE_HPP
#define PRX_IK_DATA_BASE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/graph/abstract_node.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/simulation/state.hpp"

#include <pluginlib/class_loader.h>
#include <boost/any.hpp>

namespace prx
{
    namespace plan
    {
        
        class IK_pair_t : public util::abstract_node_t
        {
          public:
            util::config_t end_effector;
            sim::state_t* manip_state;

            void serialize(std::ofstream& output_stream, const util::space_t* effector_space, const util::space_t* manip_space, unsigned precision = 8)
            {
                output_stream << effector_space->print_point(point,precision) << std::endl;                
                output_stream << manip_space->print_point(manip_state, precision) << std::endl;
                double x,y,z,w;
                end_effector.get_position(x,y,z);
                output_stream << x << " " << y << " " << z << " ";
                end_effector.get_orientation().get(x,y,z,w);
                output_stream << x << " " << y << " " << z << " " << w << std::endl;
            }
            
            void deserialize(std::ifstream& input_stream, const util::space_t* effector_space, const util::space_t* manip_space)
            {
                char trash;
                int num = effector_space->get_dimension();
                std::vector<double> tmp_vec(num);
                for(int i = 0; i<num; ++i)
                {
                    input_stream >> tmp_vec[i];
                    if( i < num - 1 )
                        input_stream >> trash;
                }
                point = effector_space->alloc_point();
                effector_space->set_from_vector(tmp_vec,point);

                num = manip_space->get_dimension();
                tmp_vec.resize(num);
                for(int i = 0; i < num; ++i)
                {
                    input_stream >> tmp_vec[i];
                    if( i < num - 1 )
                        input_stream >> trash;
                }
                manip_state = manip_space->alloc_point();
                manip_space->set_from_vector(tmp_vec,manip_state);

                double x,y,z,w;
                input_stream >> x >> y >> z;
                end_effector.set_position(x,y,z);
                input_stream >> x >> y >> z >> w;
                end_effector.set_orientation(x,y,z,w);
                end_effector.normalize_orientation();

                //ADDED AFTER ROBOT LEFT ZL: NORMALIZE THE QUATERNION FOR CORRECTNESS. ALSO THIS IS A DUMB WAY TO SER/DESERIALIZE DUE TO REPEATED INFORMATION
                tmp_vec.resize(7);
                end_effector.get_position(tmp_vec[0],tmp_vec[1],tmp_vec[2]);
                end_effector.get_orientation().get(tmp_vec[3],tmp_vec[4],tmp_vec[5],tmp_vec[6]);
                effector_space->set_from_vector(tmp_vec,point);
            }

            std::string print(const util::space_t* manip_space, int precision = 8)
            {
                std::stringstream out(std::stringstream::out);
                out << std::endl << end_effector.print() << std::endl << manip_space->print_point(manip_state, precision);
                return out.str();
            }

        };

        /**
         * @anchor IK_data_base
         *
         * 
         *
         * @brief <b> . </b>
         *
         * @author Athanasios Krontiris
         */
        class IK_data_base_t
        {
          public:
            IK_data_base_t();

            virtual ~IK_data_base_t();
            
            void init(const util::parameter_reader_t* reader);
            
            void clear();
            
            void add_pair(const util::space_t* manip_space, const util::config_t& effector_config, sim::state_t* state);
            
            void get_near_neighbors( std::vector< sim::state_t* >& states, const util::config_t& target_config, unsigned number );

            bool has_data();

            void serialize(std::ofstream& output_stream, const util::space_t* manip_space, unsigned precision = 8);
            
            void deserialize(std::ifstream& input_stream, const util::space_t* manip_space);
            
          private: 

            void effector_config_to_state(const util::config_t& config, sim::state_t* state);

            util::space_t* effector_space;
            std::vector<double*> effector_space_memory;
            sim::state_t* effector_point;
            std::vector<double> effector_vec;
            
            util::distance_metric_t* metric;
            std::vector<IK_pair_t*> all_pairs;
        };

    }
}

#endif
