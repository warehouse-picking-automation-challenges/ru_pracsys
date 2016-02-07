/**
 * @file trajectory.cpp 
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

#include "prx/simulation/trajectory.hpp"

#include <ros/ros.h>
#include <fstream>
#include <sstream>

namespace prx
{
    using namespace util;
    namespace sim
    {

        trajectory_t::trajectory_t()
        {
            state_space = NULL;
            num_states = 0;
            max_num_states = 0;
            end_state = states.begin();
            const_end_state = states.begin();
        }

        trajectory_t::trajectory_t(const trajectory_t& t)
        {
            state_space = NULL;
            num_states = 0;
            max_num_states = 0;
            end_state = states.begin();
            const_end_state = states.begin();
            (*this) = t;
        }

        trajectory_t::trajectory_t(const space_t* new_space)
        {
            state_space = new_space;
            //allocate a number of initial points
            for( unsigned i = 0; i < 4; i++ )
            {
                states.push_back(state_space->alloc_point());
            }
            num_states = 0;
            max_num_states = 4;
            end_state = states.begin();
            const_end_state = states.begin();
        }

        trajectory_t::~trajectory_t()
        {
            if( state_space != NULL )
            {

                foreach(state_t* state, states)
                {
                    state_space->free_point(state);
                }
            }
            states.clear();
        }

        trajectory_t& trajectory_t::operator=(const trajectory_t& t)
        {
            if( state_space == NULL )
            {
                state_space = t.state_space;
            }

            if( this != &t )
            {
                //check the size of the two trajectories
                while( max_num_states < t.num_states )
                {
                    increase_buffer();
                }
                //clear the trajectory
                clear();

                foreach(state_t* state, t)
                {
                    state_space->copy_point(*end_state, state);
                    ++num_states;
                    ++end_state;
                    ++const_end_state;
                }
                collision_found = t.collision_found;
            }
            return *this;
        }

        trajectory_t& trajectory_t::operator+=(const trajectory_t& t)
        {
            PRX_ASSERT(state_space != NULL);
            unsigned new_size = t.num_states + num_states;
            while( max_num_states < new_size )
            {
                increase_buffer();

                end_state = states.begin();
                const_end_state = states.begin();
                std::advance(end_state, num_states);
                std::advance(const_end_state, num_states);
            }

            foreach(state_t* state, t)
            {
                state_space->copy_point(*end_state, state);
                ++num_states;
                ++end_state;
                ++const_end_state;
            }
            return *this;
        }

        double trajectory_t::length() const
        {
            if( state_space == NULL )
                return 0;
            double cost = 0.0;
            for( unsigned i = 1; i < this->size(); i++ )
            {
                cost += state_space->distance(this->states[i - 1], this->states[i]);
            }
            return cost;
        }

        void trajectory_t::clear()
        {
            end_state = states.begin();
            const_end_state = states.begin();
            num_states = 0;
            collision_found = false;
        }

        void trajectory_t::save_to_file(std::string filename)
        {
            PRX_ASSERT(state_space != NULL);
            PRX_DEBUG_COLOR("--Saving trajectory", PRX_TEXT_GREEN);

            std::ofstream fout;

            fout.open(filename.c_str());

            fout << state_space->get_space_name() << "\n";

            foreach(state_t* state, *this)
            {
                fout << state_space->print_point(state) << "\n";
            }

            fout.close();
        }

        void trajectory_t::save_to_stream(std::ofstream& output_stream)
        {
            PRX_ASSERT(state_space != NULL);
            PRX_DEBUG_COLOR("--Saving trajectory", PRX_TEXT_GREEN);

            //    output_stream<<state_space->get_space_name()<<"\n";
            output_stream << num_states << "\n";

            foreach(state_t* state, *this)
            {
                output_stream << state_space->serialize_point(state) << "\n";
            }

        }

        void trajectory_t::read_from_file(std::string filename)
        {
            PRX_ASSERT(state_space != NULL);
            PRX_DEBUG_COLOR("-- Reading trajectory" ,PRX_TEXT_GREEN);

            clear();

            std::ifstream fin;

            fin.open(filename.c_str());

            std::string space_name;
            std::getline(fin, space_name);
            if( strcmp(space_name.c_str(), state_space->get_space_name().c_str()) == 0 )
            {
                state_t* state = state_space->alloc_point();
                while( fin )
                {
                    std::string s;
                    if( !std::getline(fin, s) )
                        break;
                    std::istringstream ss(s);
                    std::vector<double> state_vals(state_space->get_dimension());
                    int i = 0;
                    while( ss )
                    {
                        std::string s;
                        if( !std::getline(ss, s, ',') )
                            break;
                        std::istringstream data(s);
                        data >> state_vals[i];
                        i++;
                    }
                    state_space->set_from_vector(state_vals, state);
                    copy_onto_back(state);
                }
                state_space->free_point(state);
            }
            else
            {
                PRX_ERROR_S("The file to read has space name: " << space_name << " while the state_space's name " << state_space->get_space_name());
            }
            fin.close();


        }

        void trajectory_t::read_from_stream(std::ifstream& input_stream)
        {
            PRX_ASSERT(state_space != NULL);

            clear();
            int num_nodes;
            std::string space_name;
            //    std::getline(input_stream, space_name);
            input_stream >> num_nodes;
            //    if(strcmp(space_name.c_str(), state_space->get_space_name().c_str())==0)
            {
                state_t* state = state_space->alloc_point();
                //        PRX_ERROR_S ("Node deserialization");
                double value;
                char trash;
                std::vector<double> vals;
                for( int current_num = 0; current_num < num_nodes; current_num++ )
                {
                    //            PRX_WARN_S ("Num nodes: " << num_nodes << " and we're at: " << current_num);
                    vals.clear();
                    for( unsigned int i = 0; i < state_space->get_dimension(); i++ )
                    {
                        input_stream >> value;
                        //                PRX_ERROR_S ("Value " << value);
                        vals.push_back(value);
                        if( i < state_space->get_dimension() - 1 )
                            input_stream >> trash;
                        //                PRX_ERROR_S( "Trash : " << trash);
                    }
                    state_space->set_from_vector(vals, state);
                    copy_onto_back(state);

                }
                state_space->free_point(state);
            }
            //    else
            //    {
            //        PRX_ERROR_S("The file to read has space name: "<<space_name<<" while the state_space's name "<<state_space->get_space_name());
            //    }


        }

        void trajectory_t::copy_onto_back(state_t* state)
        {
            PRX_ASSERT(state_space != NULL);
            if( (num_states + 1) >= max_num_states )
            {
                increase_buffer();

                end_state = states.begin();
                const_end_state = states.begin();
                std::advance(end_state, num_states);
                std::advance(const_end_state, num_states);

            }

            state_space->copy_point(*end_state, state);
            ++end_state;
            ++const_end_state;
            ++num_states;
        }

        void trajectory_t::copy_onto_front(state_t* state)
        {
            PRX_ASSERT(state_space != NULL);
            if( (num_states + 1) >= max_num_states )
                increase_buffer();

            state_t* new_state = states.back();
            states.pop_back();
            states.push_front(new_state);
            state_space->copy_point(*states.begin(), state);
            //    PRX_WARN_S ("Copy state onto trajectory!: " << state_space->print_point(state,3));
            ++num_states;
            end_state = states.begin();
            const_end_state = states.begin();
            std::advance(end_state, num_states);
            std::advance(const_end_state, num_states);

        }

        void trajectory_t::copy_segment(int start_index, int end_index, trajectory_t* copy_destination)
        {
            PRX_ASSERT(end_index <= (int)num_states);
            copy_destination->clear();
            copy_destination->link_space(this->state_space);
            for( int i = start_index; i < end_index; i++ )
            {
                copy_destination->copy_onto_back(this->states[i]);
            }
        }

        void trajectory_t::increase_buffer()
        {
            if( max_num_states != 0 )
            {
                for( unsigned i = 0; i < max_num_states; i++ )
                {
                    states.push_back(state_space->alloc_point());
                }
                max_num_states *= 2;
            }
            else
            {
                states.push_back(state_space->alloc_point());
                max_num_states = 1;
                num_states = 0;
                end_state = states.begin();
                const_end_state = states.begin();
            }
        }

        void trajectory_t::link_space(const space_t* in_space)
        {
            if( state_space == NULL )
            {
                state_space = in_space;
                //allocate a number of initial points
                for( unsigned i = 0; i < 4; i++ )
                {
                    states.push_back(state_space->alloc_point());
                }
                num_states = 0;
                max_num_states = 4;
            }
        }

        void trajectory_t::resize(unsigned num_size)
        {
            while( max_num_states < num_size )
            {
                increase_buffer();
            }
            num_states = num_size;
            end_state = states.begin();
            const_end_state = states.begin();
            std::advance(end_state, num_states);
            std::advance(const_end_state, num_states);
        }

        void trajectory_t::splice(unsigned int splice_begin, unsigned int splice_end, const trajectory_t& t)
        {
            PRX_ASSERT(state_space != NULL);
            PRX_ASSERT(splice_end < (num_states - 1) && splice_begin < (num_states - 1) && splice_begin <= splice_end);

            PRX_DEBUG_COLOR("###SPLICE " << splice_begin << " -> " << splice_end << " in numstates " << num_states, PRX_TEXT_RED);


            //            states.erase(states.begin() + splice_begin, states.begin() + splice_end + 1);

            //            for( unsigned i=splice_begin;i<num_states;i++)
            //                PRX_DEBUG_COLOR(i<<":::"<<state_space->print_point(states[i]),PRX_TEXT_MAGENTA);


            state_t* swap_point;

            for( unsigned i = splice_end + 1, j = splice_begin; i < num_states; ++i, ++j )
            {

                swap_point = states[i];

                states[i] = states[j];
                states[j] = swap_point;

            }

            num_states = num_states - (splice_end - splice_begin + 1);

            end_state = states.begin();
            const_end_state = states.begin();
            std::advance(end_state, num_states);
            std::advance(const_end_state, num_states);

            if( t.size() > 0 )
            {
                //states.insert(states.begin() + splice_begin + 1, t.begin(), t.end());

                if( state_space == t.state_space )
                {

                    unsigned new_size = num_states + t.size();
                    while( max_num_states < new_size )
                    {
                        increase_buffer();

                    }

                    for( unsigned i = num_states - 1, j = new_size - 1; i >= splice_begin; --i, --j )
                    {
                        swap_point = states[i];

                        states[i] = states[j];
                        states[j] = swap_point;

                    }

                    for( unsigned i = 0; i < t.size(); ++i )
                    {
                        state_space->copy_point(states[splice_begin + i], t[i]);
                    }

                    num_states = new_size;
                    end_state = states.begin();
                    const_end_state = states.begin();
                    std::advance(end_state, num_states);
                    std::advance(const_end_state, num_states);
                }
                else
                {
                    PRX_FATAL_S("State space mismatch between trajectories.");
                }
            }

            PRX_DEBUG_COLOR("###SPLICE numstates " << num_states << " size of trajectory " << t.size() << " max_num_states " << max_num_states, PRX_TEXT_BLUE);
            //            for( unsigned i=splice_begin;i<num_states;i++)
            //                PRX_DEBUG_COLOR(i<<":::"<<state_space->print_point(states[i]),PRX_TEXT_CYAN);

        }

        void trajectory_t::splice(unsigned int splice_begin, unsigned int splice_end)
        {
            trajectory_t empty_trajectory;
            splice(splice_begin, splice_end, empty_trajectory);
        }

        void trajectory_t::chop(unsigned size)
        {
            PRX_ASSERT(size <= num_states);
            num_states = size;

            end_state = states.begin();
            const_end_state = states.begin();

            std::advance(end_state, num_states);
            std::advance(const_end_state, num_states);
        }

        void trajectory_t::reverse_trajectory(const trajectory_t& t)
        {
            if( state_space == NULL )
            {
                state_space = t.state_space;
            }

            //check the size of the two trajectories
            while( max_num_states < t.num_states )
            {
                increase_buffer();
            }
            //clear the trajectory
            clear();

            for( int i = t.size() - 1; i >= 0; i-- )
            {
                state_space->copy_point(*end_state, t.at(i));
                ++num_states;
                ++end_state;
                ++const_end_state;
            }

        }

        std::string trajectory_t::print() const
        {
            std::stringstream out(std::stringstream::out);

            int counter = 0;

            foreach(const state_t* st, *this)
            {
                out << counter << ": [" << state_space->print_point(st, 5)
                        << "]" << std::endl;
                counter++;
            }
            return out.str();
        }
    }
}
