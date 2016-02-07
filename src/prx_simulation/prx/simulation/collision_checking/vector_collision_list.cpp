/**
 * @file vector_collision_list.cpp 
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

#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "collision_checker.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::sim::vector_collision_list_t, prx::sim::collision_list_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        vector_collision_list_t::vector_collision_list_t() { }

        vector_collision_list_t::~vector_collision_list_t() { }

        void vector_collision_list_t::init(const parameter_reader_t * const reader, const parameter_reader_t* const template_reader )
        {

            foreach(const parameter_reader_t* list_reader, parameters::get_list("collision_list",reader,template_reader))
            {
                std::string bodyone = list_reader->get_attribute("body_one");
                std::string bodytwo = list_reader->get_attribute("body_two");

                collision_pair_t* cp = new collision_pair_t(bodyone, bodytwo);
                add_new_pair(*(cp));
                delete cp;
            }
        }

        void vector_collision_list_t::clear()
        {
            collision_pairs.clear();
        }

        void vector_collision_list_t::add_new_pair(collision_pair_t& new_collision_pair)
        {
            std::pair<std::string, std::string> other_pair = std::make_pair(new_collision_pair.second, new_collision_pair.first);

            if( !pair_in_list( new_collision_pair.first, new_collision_pair.second ) )
            {
                collision_pairs.push_back(new_collision_pair);
            }
            // else
            // {
            //     PRX_WARN_S("Attempting to add a duplicate pair!");
            // }

            // TODO: This might need to come back for add_systems, or something
            //    if(std::find(collision_pairs.begin(), collision_pairs.end(),new_collision_pair) == collision_pairs.end())
            //        if(std::find(collision_pairs.begin(), collision_pairs.end(),other_pair) == collision_pairs.end())    
        }

        void vector_collision_list_t::remove_pair(collision_pair_t& collision_pair)
        {
            std::pair<std::string, std::string> other_pair = std::make_pair(collision_pair.second, collision_pair.first);
            std::vector<collision_pair_t>::iterator iter = std::find(collision_pairs.begin(), collision_pairs.end(), collision_pair);
            std::vector<collision_pair_t>::iterator iter2 = std::find(collision_pairs.begin(), collision_pairs.end(), other_pair);

            if( iter != collision_pairs.end() )
                collision_pairs.erase(iter);
            else if( iter2 != collision_pairs.end() )
                collision_pairs.erase(iter2);
        }

        void vector_collision_list_t::remove_system(const std::string& system)
        {

            std::vector<collision_pair_t> tmp_vector = collision_pairs;
            collision_pairs.clear();

            foreach(collision_pair_t p, tmp_vector)
            if( p.first != system && p.second != system )
                collision_pairs.push_back(p);

            tmp_vector.clear();
        }

        double vector_collision_list_t::size() const
        {
            return collision_pairs.size();
        }

        collision_list_t::body_pair_range_t vector_collision_list_t::get_body_pairs()
        {
            return std::make_pair(collision_pairs.begin(), collision_pairs.end());
        }

        bool vector_collision_list_t::is_the_system_in_the_list(const std::string& system_name) const
        {
            foreach(collision_pair_t pair, collision_pairs)
            if( pair.first == system_name || pair.second == system_name )
                return true;

            return false;
        }

        bool vector_collision_list_t::pair_in_list(const std::string& system1, const std::string& system2) const
        {
            unsigned i, size;
            size = collision_pairs.size();
            for( i = 0; i < size; i++ )
                if( (collision_pairs[i].first == system1 && collision_pairs[i].second == system2) || (collision_pairs[i].first == system2 && collision_pairs[i].second == system1) )
                    return true;

            return false;
        }

        std::string vector_collision_list_t::print() const
        {
            std::stringstream output(std::stringstream::out);
            unsigned int i = 0;

            foreach(collision_pair_t pair, collision_pairs)
            {
                output << i << ") " << pair.first << "->" << pair.second << std::endl;
                i++;
            }

            return output.str();
        }


    }
}

