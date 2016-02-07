/**
 * @file pointer_open_set.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_POINTER_OPEN_SET_HPP
#define PRX_POINTER_OPEN_SET_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/heuristic_search/astar_node.hpp"
#include <vector>



namespace prx
{
    namespace util
    {

        /**
         * @anchor astar_open_set_t
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
        class astar_open_set_t
        {

          public:

            astar_open_set_t()
            {                
                heap.clear();
                heap.push_back(new astar_node_t());
                heap_size = 1;
            }

            ~astar_open_set_t(){ }

            void insert(astar_node_t* element)
            {
                if( finder.find(element) != finder.end() )
                {
                    PRX_WARN_S("Element already exists in open set, should be using update instead.");
                    return;
                }
//                PRX_DEBUG_COLOR("Heap size: " << heap.size() << "    heap_size: " << heap_size, PRX_TEXT_LIGHTGRAY);
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
            }

            astar_node_t* remove_min()
            {
                astar_node_t* return_val = NULL;

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

            void update( astar_node_t* element, astar_node_t* new_element)
            {
                PRX_ASSERT(finder.find(element) != finder.end());

                unsigned index = finder[element];
                finder[new_element] = index;
                finder.erase(element);
                
                heap[index] = new_element;
                reheap_up(index);
                reheap_down(index);
                delete element;              
            }

            void remove(astar_node_t* element)
            {
                if( finder.find(element) != finder.end() )
                {
                    unsigned i = finder[element];
                    while( i != 1 )
                    {
                        swap(i, parent(i));
                        i = parent(i);
                    }

                    astar_node_t* node = remove_min();
                    delete node;
                }
            }

            bool contains(astar_node_t* element)
            {
                return finder.find(element) != finder.end();
            }

            const astar_node_t* peek_min() const
            {
                return heap[1];
            }

            int size() const
            {
                return heap_size - 1;
            }

            bool empty() const
            {
                return size() == 0;
            }

            void clear()
            {

                for(unsigned i=1; i <heap_size; ++i)
                {
                    delete heap[i];
                }
                heap_size = 1;                
                finder.clear();
            }

            void get_items(std::vector<astar_node_t*>& items)
            {
                items.assign(heap.begin() + 1, heap.begin() + heap_size);
                //                unsigned val = heap_size;
                //                typename std::vector<astar_node_t*>::iterator iter = heap.begin() + val;
                //                items.assign(heap.begin() + 1, iter);
            }


          private:

            void reheap_up(unsigned index)
            {
                unsigned parent_index = parent(index);
//                PRX_DEBUG_COLOR("parent index : " << parent_index << "   current index: " << index, PRX_TEXT_GREEN);

                if( parent_index != 0 && *heap[index] < *heap[parent_index] )
                {                    
                    swap(index, parent_index);
                    reheap_up(parent_index);
                }
            }

            void reheap_down(unsigned index)
            {

                unsigned child1_index = child1(index);
                unsigned child2_index = child2(index);

                if( child1_index != 0 )
                {
                    if( child2_index != 0 )
                    {
                        if( !(*heap[index] < *heap[child1_index]) || !(*heap[index] < *heap[child2_index]) )
                        {
                            if( *heap[child1_index] < *heap[child2_index] )
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
                        if( *heap[child1_index] < *heap[index] )
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
//                PRX_DEBUG_COLOR("SWAP: " << val1 <<" - " << val2,PRX_TEXT_MAGENTA);
                finder[heap[val1]] = val2;
                finder[heap[val2]] = val1;
                astar_node_t* temp = heap[val1];
                heap[val1] = heap[val2];
                heap[val2] = temp;
            }


            std::vector<astar_node_t*> heap;
            hash_t<astar_node_t*, unsigned> finder;
            //            default_hashmap_t<astar_node_t*, unsigned int, H> finder;
            unsigned heap_size;
        };
    }
}



#endif