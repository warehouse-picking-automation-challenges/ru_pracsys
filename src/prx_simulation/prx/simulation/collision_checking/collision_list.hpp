/**
 * @file collision_list.hpp 
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

#ifndef PRX_COLLISION_LIST_HPP
#define	PRX_COLLISION_LIST_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <boost/range.hpp>
#include <boost/range/any_range.hpp>
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace sim
    {

        /**
         * A pair of two systems that will be stored in the collision list to be
         * checked for collisions. The name of the systems is the full slash-delimited
         * pathname of the systems.
         * 
         * @brief A pair of two systems.
         */
        typedef std::pair<std::string, std::string> collision_pair_t;

        /**
         * Its a list that will maintain the pairs of systems to be checked for collisions.
         * 
         * @brief  <b> The list that maintains the pairs of systems to be checked for collisions. </b>
         * 
         * @author Athanasios Krontiris
         */
        class collision_list_t
        {

          public:
            /**
             * A generic iterator over a range of pairs of systems.
             * Use this when you want to pass multiple systems to be iterated over,
             * but you don't want to expose the underlying container that is used.
             * 
             * @sa http://www.boost.org/doc/libs/release/libs/range/index.html
             */
            typedef boost::any_range<collision_pair_t, boost::single_pass_traversal_tag, collision_pair_t, std::ptrdiff_t> body_pair_range_t;

            virtual ~collision_list_t(){ }

            /** 
             * Initializes \ref collision_list_t from the given parameters.
             * 
             * @brief Initializes \ref collision_list_t from the given parameters.
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this \ref collision_list_t.
             */
            virtual void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t* const template_reader) = 0; //TODO: we might wanna make the init more consistent 

            /**
             * Clears the collision list from all the pairs.
             * 
             * @brief Clears the collision list.     
             */
            virtual void clear() = 0;

            /**
             * Adds a new pair of systems in the list, if the pair does not already exists.
             * 
             * @brief Adds a new pair of systems in the list.
             * 
             * @param system1 The full slash-delimited path for the first system.
             * @param system2 The full slash-delimited path for the second system.
             */
            void add_pair(const std::string& system1, const std::string& system2);

            /**
             * Adds a new \ref collision_pair_t, if the pair doesn not already exists.
             * 
             * @brief Adds a new \ref collision_pair_t.
             * 
             * @param new_collision_pair The new \ref collision_pair_t to be added in the list.
             */
            virtual void add_new_pair(collision_pair_t& new_collision_pair) = 0;

            /**
             * Removes a \ref collision_pair_t from the list. If the pair does not exist nothing will
             * happen.
             * 
             * @brief Removes a \ref collision_pair_t. 
             * 
             * @param collision_pair The pair of the systems needs to be removed from the collision
             * list.
             */
            virtual void remove_pair(collision_pair_t& collision_pair) = 0;

            /**
             * Removes all the pairs that exists between the given system and all the other systems.
             * This is mostly when we are removing the systems from the simulation or if we do not 
             * need to check for collisions, anymore, with this system. 
             * 
             * @brief Removes all the pairs between the given system and all the other systems.
             * 
             * @param system  The fully slash-delimited path for the system that needs to be
             * removed from the collision list.
             */
            virtual void remove_system(const std::string& system) = 0;

            /**
             * Returns a \ref body_pair_range_t iterator for the body pairs in the collision list.
             * 
             * @brief Returns a \ref body_pair_range_t iterator.
             * 
             * @return A \ref body_pair_range_t iterator for the body pairs in the collision list.
             */
            virtual body_pair_range_t get_body_pairs() = 0;

            /**
             * Checks if the given system is in the collision list.
             * 
             * @brief Checks if the given system is in the collision list.
             * 
             * @param system_name The full slash-delimited path for the system.
             * 
             * @return True if the system is in the list. \n
             *         False if it is not.
             */
            virtual bool is_the_system_in_the_list(const std::string& system_name) const = 0;

            /**
             * Checks if the pair between the two systems is in the collision list or not.
             * 
             * @brief Checks if the pair between the two systems is in the collision list or not.
             * 
             * @param system1 The full slash-delimited path for the first system.
             * @param system2 The full slash-delimited path for the second system.
             * @return True if the pair is in the list. \n
             *         False if it is not.
             */
            virtual bool pair_in_list(const std::string& system1, const std::string& system2) const = 0;

            /**
             * Returns the size of the list.
             * 
             * @brief Returns the size of the list.
             * 
             * @return The size of the list.
             */
            virtual double size() const = 0;

            /**
             * Returns a string with all the information from the collision list. This information
             * will be in the form of a string in order to be ready to be printed. 
             * 
             * @brief Return a string with the information from the collision list.
             * 
             * @return A string with the information from the collision list.
             */
            virtual std::string print() const;

            /** @copydoc system_t::get_loader() */
            static pluginlib::ClassLoader<collision_list_t>& get_loader();

          private:

            /** @copydoc system_t::loader */
            static pluginlib::ClassLoader<collision_list_t> loader;
        };

    }
}

#endif
