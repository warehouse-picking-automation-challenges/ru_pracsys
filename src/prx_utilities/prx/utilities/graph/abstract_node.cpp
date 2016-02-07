/**
 * @file abstract_node.hpp
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


#include "prx/utilities/graph/abstract_node.hpp"
#include "prx/utilities/spaces/embedded_space.hpp"
#include <fstream>

#include <boost/graph/adjacency_list.hpp>

namespace prx
{
    namespace util
    {

        void abstract_node_t::serialize(std::ofstream& output_stream, const space_t* point_space)
        {
            //    PRX_ERROR_S ("Node serialization");
            PRX_ASSERT(point);
            const embedded_space_t* check_embedded = dynamic_cast<const embedded_space_t*>(point_space);
            if( check_embedded != NULL )
            {
                embedded_point_t* embedded_point = dynamic_cast<embedded_point_t*>(point);
                output_stream << node_id << " " << check_embedded->get_preimage_space()->print_point(embedded_point->link);
            }
            else
            {
                output_stream << node_id << " " << point_space->print_point(point);
            }
        }

        // Accepts an ifstream and deserializes a node to output_node

        void abstract_node_t::deserialize(std::ifstream& input_stream, const space_t* point_space)
        {
            //    PRX_DEBUG_S ("Node deserialization");

            const embedded_space_t* check_embedded = dynamic_cast<const embedded_space_t*>(point_space);
            double value;
            char trash;
            input_stream >> node_id;
            std::vector<double> vals;

            if( check_embedded != NULL )
            {
                unsigned int space_dim = check_embedded->get_preimage_space()->get_dimension();
                for( unsigned int i = 0; i < space_dim; i++ )
                {
                    input_stream >> value;
                    PRX_DEBUG_S("Value " << value);
                    vals.push_back(value);
                    if( i < space_dim - 1 )
                        input_stream >> trash;
                    PRX_DEBUG_S("Trash : " << trash);
                }

                check_embedded->get_preimage_space()->set_from_vector(vals);
                point_space->copy_to_point(point);
            }
            else
            {

                for( unsigned int i = 0; i < point_space->get_dimension(); i++ )
                {
                    input_stream >> value;
                    PRX_DEBUG_S("Value " << value);
                    vals.push_back(value);
                    if( i < point_space->get_dimension() - 1 )
                        input_stream >> trash;
                    PRX_DEBUG_S("Trash : " << trash);
                }

                point_space->set_from_vector(vals, point);
            }

        }

    }
}
