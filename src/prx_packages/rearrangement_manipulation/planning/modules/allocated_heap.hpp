/**
 * @file allocated_heap.hpp
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

#ifndef PRX_ALLOCATED_HEAP_HPP
#define	PRX_ALLOCATED_HEAP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/random.hpp"
#include <deque>

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * A pre-allocated heap to reduce the time for allocating deallocating a heap all the time.
             * TODO: its not fully tested. Works only push_back, pop_front, is_empty, size.
             */
            template <class T>
            class allocated_heap_t
            {

              public:

                typedef typename std::deque< T >::iterator iterator;
                typedef typename std::deque< T >::const_iterator const_iterator;

                allocated_heap_t(){ }

                allocated_heap_t(int size)
                {
                    start = 0;
                    final = 0;
                    no_elements = 0;
                    max_size = size;
                    heap.resize(size);
                }

                virtual ~allocated_heap_t(){ }

                void restart()
                {
                    start = 0;
                    final = 0;
                    no_elements = 0;
                }

                void resize(int size)
                {
                    restart();
                    max_size = size;
                    heap.resize(size);
                }

                bool empty()
                {
                    return start == final;
                }

                int size()
                {
                    return no_elements;
                    //                    if( start > final )
                    //                        return max_size - start + final;
                    //                    return final - start;
                }

                void push_back(T element)
                {
                    heap[final] = element;
                    final = adjust_counter(final + 1);
                    ++no_elements;
                    //                    PRX_INFO_S("push back :  start: " << start << "    final : " << final << "   no_elements: " << no_elements);
                    PRX_ASSERT(final != start);

                }

                void push_front(T element)
                {
                    start = adjust_counter(start - 1);
                    heap[start] = element;
                    ++no_elements;
                    //                    PRX_INFO_S("pus front start: " << start << "    final : " << final << "   no_elements: " << no_elements);
                    PRX_ASSERT(final != start);
                }

                T pop_front()
                {
                    T ret = heap[start];
                    start = adjust_counter(start + 1);
                    --no_elements;
                    //                    PRX_INFO_S("POP start: " << start << "    final : " << final << "   no_elements: " << no_elements);
                    return ret;
                }

                /**
                 * This function will pop a random element from the heap. When this function is used 
                 * the elements in the heap will not have the same order. The last element will always 
                 * replace the random element that has been selected. 
                 * 
                 * @return A random element from the heap.
                 */
                T pop_random()
                {
                    int index = util::uniform_random(0, no_elements);
                    //                    PRX_DEBUG_S("first time index :  " << index << "  no_elements: " << no_elements << "   start: " << start << "    final: " << final);
                    index = (start + index) % max_size;
                    //                    PRX_ERROR_S("index :  " << index);
                    T ret = heap[index];
                    --no_elements;
                    //because the index final pointing one spot after the last element, we need to 
                    //move the index one place before and get this element to replace the random 
                    //selected element.
                    final = adjust_counter(final - 1);
                    heap[index] = heap[final];
                    return ret;
                }

                T front()
                {
                    return heap[start];
                }

                void replace_with_last(T element)
                {
                    int i = start;
                    int final_prev = adjust_counter(final - 1);
                    bool not_done = true;
                    while( i != final && not_done )
                    {
                        if( heap[i] == element )
                        {
                            if( i != final_prev )
                            {
                                heap[i] = heap[final_prev];
                            }
                            final = final_prev;
                            not_done = false;
                        }
                        i = adjust_counter(++i);
                    }
                    --no_elements;
                }


                //    T pop_back()
                //    {
                //        T ret = heap[final];
                //        final=adjust_counter(final - 1);
                //        return ret;
                //    }

                bool has(T element)
                {
                    //        PRX_WARN_S("heap has : " << element);
                    int i = start;
                    while( i != final )
                    {
                        if( heap[i] == element )
                            return true;

                        i = adjust_counter(++i);
                    }
                    return false;
                }

                std::string print()
                {
                    std::stringstream out(std::stringstream::out);
                    int i = start;
                    out << "max size: " << max_size << "[" << start << "," << final << "]:";
                    while( i != final )
                    {
                        out << heap[i] << " , ";
                        i = adjust_counter(++i);
                    }
                    return out.str();
                }


              protected:

              private:
                int start;
                int final;
                int max_size;
                int no_elements;
                std::deque<T> heap;

                int adjust_counter(int counter)
                {
                    if( counter >= max_size )
                        return 0;
                    if( counter < 0 )
                        return max_size - 1;
                    return counter;
                }
            };
        }
    }
}

#endif	

