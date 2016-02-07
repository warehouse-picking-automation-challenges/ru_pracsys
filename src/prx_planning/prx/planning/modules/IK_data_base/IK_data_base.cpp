/**
 * @file IK_data_base.cpp 
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

#include "prx/planning/modules/IK_data_base/IK_data_base.hpp"

namespace prx
{
    using namespace util;
    using namespace util::parameters;
    using namespace sim;

    namespace plan
    {

        IK_data_base_t::IK_data_base_t()
        {
            for( int i = 0; i < 7; ++i )
                effector_space_memory.push_back(new double());
            effector_space = new space_t("SE3", effector_space_memory);
            effector_point = effector_space->alloc_point();
            effector_vec.resize(effector_space_memory.size());
            metric = NULL;
        }

        IK_data_base_t::~IK_data_base_t() 
        {
            clear();
            
            effector_space->free_point(effector_point);
            delete effector_space;
            if( metric != NULL )
                delete metric;
        }

        void IK_data_base_t::clear()
        {
            if( metric != NULL )
                metric->clear();
            for( unsigned i=0; i<all_pairs.size(); ++i )
            {
                delete all_pairs[i];
            }
            all_pairs.resize(0);
        }

        void IK_data_base_t::init(const parameter_reader_t* reader)
        {
            PRX_PRINT("IK_data_base init ...", PRX_TEXT_CYAN);
            if(reader->has_attribute("distance_metric"))
            {
                metric = reader->initialize_from_loader<distance_metric_t>("distance_metric","prx_utilities");
            }
            else
            {
                PRX_FATAL_S("Missing distance_metric attribute for IK_data_base!");
            }
            metric->link_space(effector_space);
        }

        void IK_data_base_t::add_pair(const space_t* manip_space, const config_t& effector_config, state_t* state) 
        {
            IK_pair_t* node = new IK_pair_t();
            //PRX_PRINT("got new ik node, effector_space : " << effector_space->get_space_name(), PRX_TEXT_CYAN);
            node->point = effector_space->alloc_point();
            //PRX_PRINT("alloc point ", PRX_TEXT_CYAN);
            effector_config_to_state(effector_config,node->point);
            //PRX_PRINT("get the point from config: " << effector_space->print_point(node->point,6), PRX_TEXT_CYAN);
            //PRX_PRINT(" the manip space : " << manip_space->get_space_name(),PRX_TEXT_CYAN);
            node->manip_state = manip_space->clone_point(state);
            //PRX_PRINT("new manip state!",PRX_TEXT_CYAN);
            node->end_effector = effector_config;
            //PRX_PRINT("Got the config going for the me metric add", PRX_TEXT_CYAN);
            metric->add_point(node);
            //PRX_PRINT("added in metric",PRX_TEXT_BROWN);
            all_pairs.push_back(node);
            //PRX_PRINT("added in the vector!" , PRX_TEXT_BROWN);
        }

        void IK_data_base_t::get_near_neighbors( std::vector< state_t* >& states, const config_t& target_config, unsigned number )
        {
            effector_config_to_state(target_config, effector_point);
            std::vector< const abstract_node_t* > nodes = metric->multi_query( effector_point, number );

            foreach(const abstract_node_t* node, nodes)
            {
                states.push_back(dynamic_cast<const IK_pair_t *>(node)->manip_state);
            }
        }

        bool IK_data_base_t::has_data()
        {
            return metric->get_nr_points() > 0;
        }

        void IK_data_base_t::serialize(std::ofstream& output_stream, const space_t* manip_space, unsigned precision) 
        {
            output_stream << all_pairs.size() << std::endl;
            foreach(IK_pair_t* pair, all_pairs)
            {
                pair->serialize(output_stream, effector_space, manip_space, precision);
            }
            
        }

        void IK_data_base_t::deserialize(std::ifstream& input_stream, const space_t* manip_space)  
        {
            int num;
            input_stream >> num;
            
            PRX_PRINT("IK Database Loading [" << num << "] points...", PRX_TEXT_GREEN);
            
            for(int i = 0; i < num; ++i)
            {
                IK_pair_t* pair = new IK_pair_t();
                pair->deserialize(input_stream, effector_space, manip_space);

                metric->add_point(pair);
                all_pairs.push_back(pair);
                // PRX_PRINT(pair->print(manip_space),PRX_TEXT_CYAN);
            }

        }

        void IK_data_base_t::effector_config_to_state(const config_t& config, state_t* state)
        {
            config.get_position(effector_vec[0], effector_vec[1], effector_vec[2]);
            quaternion_t q = config.get_orientation();
            effector_vec[3] = q.get_x();
            effector_vec[4] = q.get_y();
            effector_vec[5] = q.get_z();
            effector_vec[6] = q.get_w();
            effector_space->set_from_vector(effector_vec, state);
        }
    }
}
