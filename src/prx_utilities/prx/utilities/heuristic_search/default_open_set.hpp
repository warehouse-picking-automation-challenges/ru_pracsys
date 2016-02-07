/**
 * @file default_open_set.hpp 
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

#ifndef PRX_DEFAULT_OPEN_SET_HPP
#define PRX_DEFAULT_OPEN_SET_HPP

#include "default_hashmap.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include <vector>



namespace prx
{
    namespace util
    {

        template<class T>
        class default_comparison_operator_t
        {

          public:

            bool operator()(const T& val1, const T& val2) const
            {
                return val1 < val2;
            }
        };

        template<class T>
        class default_hash_t
        {

          public:

            unsigned operator()(const T& val) const
            {
                return (unsigned)val;
            }
        };

        /**
         * @anchor default_open_set_t
         *
         * This is the default open set implementation for A*. It is templated for 
         * greater flexibility, but alternative implementations do not need to be. 
         * As long as an open set implementation provides the same interface, A* will 
         * attempt to use it. This implementation is based on the hashed heap originally 
         * created for the NASA Ames version of Field D*.
         * 
         * @brief <b> The default open set implementation for A*. </b>
         * 
         * @author Justin Cardoza
         */
        template <class T, class O = default_comparison_operator_t<T>, class H = default_hash_t<T> >
        class default_open_set_t
        {

          public:

            default_open_set_t() : finder(12)
            {
                heap.clear();
                T val;
                heap.push_back(val);
                heap_size = 1;
            }

            ~default_open_set_t(){ }

            void insert(const T& element)
            {
                unsigned i;
                if( finder.find(element, i) )
                {
                    PRX_WARN_S("Element already exists in open set, should be using update instead.");
                    return;
                }
                if( heap.size() > heap_size )
                {
                    heap[heap_size] = element;
                }
                else
                {
                    heap.push_back(element);
                }
                heap_size++;
                finder[element] = heap_size - 1;
                reheap_up(heap_size - 1);
                // heap.push_back(element);
                // finder[element] = heap.size() - 1;
                // reheap_up(heap.size() - 1);
            }

            T remove_min()
            {
                T return_val;

                if( heap_size > 1 )
                {
                    return_val = heap[1];
                    // heap[1] = heap.back();
                    heap[1] = heap[heap_size - 1];
                    finder[heap[1]] = 1;
                    finder.erase(return_val);
                    // heap.pop_back();
                    heap_size--;
                    reheap_down(1);
                }

                return return_val;
            }

            void update(const T& element, const T& new_element)
            {
                unsigned index;

                if( finder.update_key(element, new_element, index) )
                {
                    heap[index] = new_element;
                    reheap_up(index);
                    reheap_down(index);
                }
            }

            void remove(const T& element)
            {
                unsigned i;

                if( finder.find(element, i) )
                {
                    while( i != 1 )
                    {
                        swap(i, parent(i));
                        i = parent(i);
                    }

                    remove_min();
                }
            }

            bool contains(const T& element)
            {
                unsigned i;
                return finder.find(element, i);
            }

            const T& peek_min() const
            {
                return heap[1];
            }

            int size() const
            {
                // return heap.size() - 1;
                return heap_size - 1;
            }

            bool empty() const
            {
                return size() == 0;
            }

            void clear()
            {
                heap_size = 1;
                // heap.resize(1);
                finder.clear();
            }

            void get_items(std::vector<T>& items)
            {
                // items.assign(heap.begin()+1, heap.end());
                unsigned val = heap_size;
                typename std::vector<T>::iterator iter = heap.begin() + val;
                items.assign(heap.begin() + 1, iter);
            }


          private:

            void reheap_up(unsigned index)
            {
                O op;
                unsigned parent_index = parent(index);

                if( parent_index != 0 && op(heap[index], heap[parent_index]) )
                {
                    swap(index, parent_index);
                    reheap_up(parent_index);
                }
            }

            void reheap_down(unsigned index)
            {
                O op;
                unsigned child1_index = child1(index);
                unsigned child2_index = child2(index);

                if( child1_index != 0 )
                {
                    if( child2_index != 0 )
                    {
                        if( !op(heap[index], heap[child1_index]) || !op(heap[index], heap[child2_index]) )
                        {
                            if( op(heap[child1_index], heap[child2_index]) )
                            {
                                swap(child1_index, index);
                                reheap_down(child1_index);
                            }
                            else
                            {
                                swap(child2_index, index);
                                reheap_down(child2_index);
                            }
                        }
                    }
                    else
                    {
                        if( op(heap[child1_index], heap[index]) )
                        {
                            swap(index, child1_index);
                            reheap_down(child1_index);
                        }
                    }
                }
            }

            unsigned parent(unsigned index)
            {
                return index / 2;
            }

            unsigned child1(unsigned index)
            {
                unsigned val = index * 2;
                // if(val < heap.size())
                // 	return val;

                if( val < heap_size )
                    return val;
                return 0;
            }

            unsigned child2(unsigned index)
            {
                unsigned val = index * 2 + 1;
                // if(val < heap.size())
                // 	return val;
                if( val < heap_size )
                    return val;
                return 0;
            }

            void swap(unsigned val1, unsigned val2)
            {
                finder[heap[val1]] = val2;
                finder[heap[val2]] = val1;
                T temp = heap[val1];
                heap[val1] = heap[val2];
                heap[val2] = temp;
            }


            std::vector<T> heap;
            default_hashmap_t<T, unsigned int, H> finder;
            unsigned heap_size;
        };
    }
}



#endif