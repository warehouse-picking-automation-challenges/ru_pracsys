/**
 * @file default_hashmap.hpp 
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
#pragma once

#ifndef PRX_DEFAULT_HASHMAP_HPP
#define PRX_DEFAULT_HASHMAP_HPP

#include <vector>



namespace prx
{
    namespace util
    {

        template<class K, class V, class H>
        class default_hashmap_t
        {

          public:

            default_hashmap_t(unsigned tableOrder = 4) : dummy(), size(0), items(1 << tableOrder){ }

            ~default_hashmap_t(){ }

            void insert(const K& key, const V& value)
            {
                H func;
                std::vector<Entry>& slot = items[func(key) & (items.size() - 1)];

                for( unsigned i = 0; i < slot.size(); ++i )
                {
                    if( slot[i].key == key )
                    {
                        slot[i].value = value;
                        return;
                    }
                }

                ++size;
                slot.push_back(Entry(key, value));

                if( size > items.size() )
                    expand();
            }

            void remove(const K& key)
            {
                H func;
                std::vector<Entry>& slot = items[func(key) & (items.size() - 1)];

                for( unsigned i = 0; i < slot.size(); ++i )
                {
                    if( slot[i].key == key )
                    {
                        slot[i] = slot.back();
                        slot.pop_back();
                        --size;
                        return;
                    }
                }
            }

            void erase(const K& key)
            {
                remove(key);
            }

            V get(const K& key) const
            {
                H func;
                std::vector<Entry>& slot = items[func(key) & (items.size() - 1)];

                for( unsigned i = 0; i < slot.size(); ++i )
                {
                    if( slot[i].key == key )
                    {
                        return slot[i].value;
                    }
                }

                return dummy;
            }

            bool update_key(const K& key, const K& newKey, V& value)
            {
                H func;
                std::vector<Entry>& slot = items[func(key) & (items.size() - 1)];

                for( unsigned i = 0; i < slot.size(); ++i )
                {
                    if( slot[i].key == key )
                    {
                        Entry temp(newKey, slot[i].value);

                        slot[i] = slot.back();
                        slot.pop_back();

                        std::vector<Entry>& newSlot = items[func(newKey) & (items.size() - 1)];
                        newSlot.push_back(temp);
                        value = temp.value;
                        return true;
                    }
                }

                return false;
            }

            bool find(const K& key, V& value) const
            {
                H func;
                const std::vector<Entry>& slot = items[func(key) & (items.size() - 1)];

                for( unsigned i = 0; i < slot.size(); ++i )
                {
                    if( slot[i].key == key )
                    {
                        value = slot[i].value;
                        return true;
                    }
                }

                return false;
            }

            bool find(const K& key) const
            {
                H func;
                const std::vector<Entry>& slot = items[func(key) & (items.size() - 1)];

                for( unsigned i = 0; i < slot.size(); ++i )
                {
                    if( slot[i].key == key )
                    {
                        return true;
                    }
                }

                return false;
            }

            void clear()
            {
                for( unsigned i = 0; i < items.size(); ++i )
                {
                    items[i].clear();
                }
            }

            V& operator[](const K& key)
            {
                H func;

                while( true )
                {
                    std::vector<Entry>& slot = items[func(key) & (items.size() - 1)];

                    for( unsigned i = 0; i < slot.size(); ++i )
                    {
                        if( slot[i].key == key )
                        {
                            return slot[i].value;
                        }
                    }

                    ++size;
                    slot.push_back(Entry(key, V()));

                    if( size > items.size() )
                    {
                        expand();
                    }
                }
            }



          private:

            struct Entry
            {

                K key;
                V value;

                Entry() : key(), value(){ }

                Entry(const K& k, const V & v) : key(k), value(v){ }
            };

            V dummy;
            unsigned size;
            std::vector <std::vector<Entry> > items;

            void expand()
            {
                std::vector<Entry> temp(size);
                unsigned newSize, i;
                H func;

                temp.clear();

                for( i = 0; i < items.size(); ++i )
                {
                    temp.insert(temp.end(), items[i].begin(), items[i].end());
                }

                newSize = items.size() << 2;
                items.clear();
                items.resize(newSize);

                for( i = 0; i < size; ++i )
                {
                    items[func(temp[i].key) & (items.size() - 1)].push_back(temp[i]);
                }
            }
        };
    }
}



#endif
