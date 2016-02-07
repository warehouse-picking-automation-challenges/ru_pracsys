/**
 * @file hash.hpp 
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

#ifndef PRACSYS_HASH_T_HPP
#define	PRACSYS_HASH_T_HPP

#include <string>
#include <boost/unordered_map.hpp>
#include <boost/functional/hash.hpp>

#include "prx/utilities/definitions/defs.hpp"

namespace prx
{
    namespace util
    {

        /**
         * Wraps the \ref boost::unordered_map class
         * @brief <b> Wraps the \ref boost::unordered_map class</b>
         * @author Ilias Apostolopoulos, Athanasios Krontiris, Andrew Kimmel
         */
        template <class Key, class Value, class HashFunction = boost::hash<Key> >
        class hash_t
        {

          public:

            /** @brief Wraps the boost iterator */
            typedef typename boost::unordered_map<Key, Value, HashFunction>::iterator iterator;
            /** @brief Wraps the boost const iterator */
            typedef typename boost::unordered_map<Key, Value, HashFunction>::const_iterator const_iterator;

          public:

            hash_t(){ }

            hash_t(HashFunction hash_function) : hash(8, hash_function){ }

            virtual ~hash_t(){ }

            /**
             * Calls the function begin() on the hash
             * 
             * @brief Gets the iterator pointing to the beginning of the hash_t
             * @return The iterator pointing to the beginning of the hash_t
             */
            iterator begin(void)
            {
                return hash.begin();
            }

            /**
             * Calls the function begin() on the hash
             * 
             * @brief Gets the iterator pointing to the beginning of the hash_t
             * @return The const iterator pointing to the beginning of the hash_t
             */
            const_iterator begin(void) const
            {
                return hash.begin();
            }

            /**
             * Calls the function end() on the hash
             * 
             * @brief Gets the iterator pointing to the end of the hash_t
             * @return The iterator pointing to the end of the hash_t
             */
            iterator end(void)
            {
                return hash.end();
            }

            /**
             * Calls the function end() on the hash
             * 
             * @brief Gets the iterator pointing to the end of the hash_t
             * @return The const iterator pointing to the end of the hash_t
             */
            const_iterator end(void) const
            {
                return hash.end();
            }

            /**
             * Retrieves the size of the hash_t
             * 
             * @brief Retrieves the size of the hash_t
             * @return The size of the hash_t
             */
            size_t size(void) const
            {
                return hash.size();
            }

            /**
             * Retrieves the maximum size of the hash_t
             * 
             * @brief Retrieves the maximum size of the hash_t
             * @return The maximum size of the hash_t
             */
            size_t max_size(void) const
            {
                return hash.max_size();
            }

            /**
             * Checks if the hash is empty
             * 
             * @brief Checks if the hash is empty
             * @return True if the hash is empty, false otherwise
             */
            bool empty(void) const
            {
                return hash.empty();
            }

            hash_t& operator=(const hash_t& _hash)
            {
                hash = _hash.hash;
                return *this;
            }

            Value& operator[](Key k)
            {
                return hash[k];
            }

            /**
             * Indexes into the hash_t using a key, prints an
             * error message if the key does not have a mapped value.
             * 
             * @brief Indexes into the hash_t using a key
             * @param k Used to index into the hash_t
             * @return The value stored at the index in the hash_t
             */
            const Value& operator[](Key k) const
            {
                const const_iterator result = hash.find(k);
                if( result == hash.end() )
                    PRX_ERROR_S("Trying to access value \"" << k << "\" which does not exist in the hash.");

                return result->second;
            }

            /**
             * @brief Inserts a value into the hash_t
             * @param x The value to insert
             * @return A pair containing an iterator pointing at the inserted value and a bool from boost
             */
            std::pair<iterator, bool> insert(const Value& x)
            {
                return hash.insert(x);
            }

            /**
             * Removes an element from the hash_t at a specific revision
             * 
             * @brief Removes an element from the hash_t at a specific revision
             * @param pos The position of the element to remove
             */
            void erase(iterator pos)
            {
                hash.erase(pos);
            }

            /**
             * Removes an element from the hash_t at a specific revision
             * 
             * @brief Removes an element from the hash_t at a specific revision
             * @param pos The position of the element to remove
             * @return The size of the hash_t after removal
             */
            size_t erase(const Key& k)
            {
                return hash.erase(k);
            }

            /**
             * @brief Counts the number of instances of a key
             * @param k The key to count
             * @return The number of instances of the key
             */
            size_t count(Key& k)
            {
                return hash.count(k);
            }

            /**
             * @brief Counts the number of instances of a key
             * @param k The key to count
             * @return The number of instances of the key
             */
            size_t count(const Key& k) const
            {
                return hash.count(k);
            }

            /**
             * Clears the hash_t
             * 
             * @brief Clears the hash_t
             */
            void clear()
            {
                hash.clear();
            }

            /**
             * Searches for a key stored in the hash
             * 
             * @brief Searches for a key stored in the hash
             * @param k The key to search for
             * @return The const iterator pointing to the key
             */
            const_iterator find(const Key& k) const
            {
                return hash.find(k);
            }

            /**
             * Searches for a key stored in the hash
             * 
             * @brief Searches for a key stored in the hash
             * @param k The key to search for
             * @return The iterator pointing to the key
             */
            iterator find(const Key& k)
            {
                return hash.find(k);
            }

          private:
            /** @brief The boost unordered map */
            boost::unordered_map<Key, Value, HashFunction> hash;

        };

        /**
         * @brief <b> Used for hashing pointers </b>
         */
        template <typename Pointer>
        struct pointer_hash
        {

            size_t operator()(const Pointer* p) const
            {
                size_t hashvalue = 0;
                hashvalue = reinterpret_cast<unsigned long>(p);
                return hashvalue;
            }
        };

        /**
         * Wraps the \ref boost::unordered_map class, used for hashing pointers
         * @brief <b> Wraps the \ref boost::unordered_map class </b>
         * @author Ilias Apostolopoulos, Athanasios Krontiris, Andrew Kimmel
         */
        template <typename Key, typename Value>
        class hash_t<Key*, Value, pointer_hash<Key> >
        {

          public:
            /** @brief Wraps the boost iterator */
            typedef typename boost::unordered_map<Key*, Value, pointer_hash<Key> >::iterator iterator;
            /** @brief Wraps the boost const iterator */
            typedef typename boost::unordered_map<Key*, Value, pointer_hash<Key> >::const_iterator const_iterator;

          public:

            /** @copydoc hash_t::begin() */
            iterator begin(void)
            {
                return hash.begin();
            }

            /** @copydoc hash_t::begin() const */
            const_iterator begin(void) const
            {
                return hash.begin();
            }

            /** @copydoc hash_t::end() */
            iterator end(void)
            {
                return hash.end();
            }

            /** @copydoc hash_t::end() */
            const_iterator end(void) const
            {
                return hash.end();
            }

            Value& operator[](Key* k)
            {
                return hash[k];
            }

            /**
             * Indexes into the hash_t using a key, throws an
             * error if the key does not have a mapped value.
             * 
             * @brief Indexes into the hash_t using a key
             * @param k Used to index into the hash_t
             * @return The value stored at the index in the hash_t
             */
            const Value& operator[](Key* k) const
            {
                const const_iterator result = hash.find(k);
                if( result == hash.end() )
                    PRX_FATAL_S("Trying to access a value that does not exist in the hash");
                return result->second;
            }

            /** @copydoc hash_t::erase() */
            void erase(iterator pos)
            {
                hash.erase(pos);
            }

            /** @copydoc hash_t::erase() */
            size_t erase(Key* k)
            {
                return hash.erase(k);
            }

            /** @copydoc hash_t::find() */
            const_iterator find(const Key* k) const
            {
                return hash.find(k);
            }

            /** @copydoc hash_t::find() */
            iterator find(Key* k)
            {
                return hash.find(k);
            }

          private:
            /** @brief The boost unordered map */
            boost::unordered_map<Key*, Value, pointer_hash<Key> > hash;

        };

        /**
         * @brief <b> Used for hashing strings </b>
         */
        struct string_hash
        {

            size_t operator()(const std::string& str) const
            {
                size_t seed = 0, str_len = str.size();
                for( size_t i = 0; i < str_len; ++i )
                    boost::hash_combine(seed, str[i]);
                return seed;
            }
        };

        /**
         * Wraps the \ref boost::unordered_map class, used for string pointers
         * @brief <b> Wraps the \ref boost::unordered_map class </b>
         * @author Ilias Apostolopoulos, Athanasios Krontiris, Andrew Kimmel
         */
        template <typename Value>
        class hash_t<std::string, Value, string_hash>
        {

          public:
            /** @brief Wraps the boost iterator */
            typedef typename boost::unordered_map<std::string, Value, string_hash>::iterator iterator;
            /** @brief Wraps the boost const iterator */
            typedef typename boost::unordered_map<std::string, Value, string_hash>::const_iterator const_iterator;

          public:

            /** @copydoc hash_t::begin() */
            iterator begin(void)
            {
                return hash.begin();
            }

            /** @copydoc hash_t::begin() */
            const_iterator begin(void) const
            {
                return hash.begin();
            }

            /** @copydoc hash_t::end() */
            iterator end(void)
            {
                return hash.end();
            }

            /** @copydoc hash_t::end() */
            const_iterator end(void) const
            {
                return hash.end();
            }

            Value& operator[](const std::string& k)
            {
                return hash[k];
            }

            /**
             * Indexes into the hash_t using a key, prints an
             * error message if the key does not have a mapped value.
             * 
             * @brief Indexes into the hash_t using a key
             * @param k Used to index into the hash_t
             * @return The value stored at the index in the hash_t
             */
            const Value& operator[](const std::string& k) const
            {
                const const_iterator result = hash.find(k);

                if( result == hash.end() )
                    PRX_FATAL_S("Trying to access a value that does not exist in the hash");
                return result->second;
            }

            /** @copydoc hash_t::erase() */
            void erase(iterator pos)
            {
                hash.erase(pos);
            }

            /** @copydoc hash_t::erase() */
            size_t erase(const std::string& k)
            {
                return hash.erase(k);
            }

            /** @copydoc hash_t::find() */
            const_iterator find(const std::string& k) const
            {
                return hash.find(k);
            }

            /** @copydoc hash_t::find() */
            iterator find(const std::string& k)
            {
                return hash.find(k);
            }

            /** @copydoc hash_t::clear() */
            void clear()
            {
                hash.clear();
            }

            /**
             * Returns a string that has each value in the hash_t in it
             * 
             * @return A string representation of the values in the hash
             * @brief Returns a string that has each value in the hash_t in it
             */
            std::string print() const
            {
                std::stringstream out(std::stringstream::out);

                for( const_iterator val = begin(); val != end(); val++ )
                {
                    out << "Key: " << val->first << std::endl;
                }
                return out.str();

            }

          private:
            /** @brief The boost unordered map */
            boost::unordered_map<std::string, Value, string_hash> hash;

        };

    }
}

#endif

