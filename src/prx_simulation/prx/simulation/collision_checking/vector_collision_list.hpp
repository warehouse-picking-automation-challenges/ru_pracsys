/**
 * @file vector_collision_list.hpp 
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

#ifndef PRX_VECTOR_COLLISION_LIST_HPP
#define	PRX_VECTOR_COLLISION_LIST_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * An implementation for a collision list. This class will maintain a vector list for the 
         * pairs of the systems that needs to be checked for collisions. 
         * 
         * @brief <b> A vector list \ref collision_list_t in order to maintain the pairs of the systems. </b>
         * 
         * @author Athanasios Krontiriss 
         */
        class vector_collision_list_t : public collision_list_t
        {

          public:

            vector_collision_list_t();
            virtual ~vector_collision_list_t();

            /** @copydoc collision_list_t::init(const util::parameter_reader_t* const) */
            void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t* const template_reader);

            /** @copydoc collision_list_t::clear()*/
            void clear();

            /** @copydoc collision_list_t::add_new_pair(collision_pair_t&) */
            void add_new_pair(collision_pair_t& new_collision_pair);

            /** @copydoc collision_list_t::remove_pair(collision_pair_t&)*/
            void remove_pair(collision_pair_t& collision_pair);

            /** @copydoc collision_list_t::remove_system(const std::string&)*/
            void remove_system(const std::string& system);

            /** @copydoc collision_list_t::size() const*/
            double size() const;

            /** @copydoc collision_list_t::get_body_pairs()*/
            body_pair_range_t get_body_pairs();

            /** @copydoc collision_list_t::pair_in_list( const std::string&, const std::string& ) const*/
            bool pair_in_list(const std::string& system1, const std::string& system2) const;

            /** @copydoc collision_list_t::is_the_system_in_the_list(const std::string&) const*/
            bool is_the_system_in_the_list(const std::string& system_name) const;

            /** @copydoc collision_list_t::print() const */
            std::string print() const;

          protected:
            std::vector<collision_pair_t> collision_pairs;

        };

    }
}

#endif

