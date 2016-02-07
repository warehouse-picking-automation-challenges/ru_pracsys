/**
 * @file plan.cpp
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


#include "prx/simulation/plan.hpp"
#include <ros/ros.h>

#include <fstream>
#include <sstream>
#include <iostream>

namespace prx
{
    using namespace util;

    namespace sim
    {

        plan_t::plan_t()
        {
            control_space = NULL;
            state_space = NULL;
            end_state = NULL;
            max_num_steps = 0;
            num_steps = 0;
            const_end_step = steps.begin();
            end_step = steps.begin();
            end_state_set = false;
        }

        plan_t::plan_t(const space_t* new_space)
        {
            control_space = new_space;
            state_space = NULL;
            end_state = NULL;
            //allocate an initial step
            steps.push_back(plan_step_t(control_space->alloc_point(), 0));
            num_steps = 0;
            max_num_steps = 1;
            end_step = steps.begin();
            const_end_step = steps.begin();
            end_state_set = false;
        }

        plan_t::plan_t(const plan_t& other)
        {
            control_space = NULL;
            state_space = NULL;
            end_state = NULL;
            max_num_steps = 0;
            num_steps = 0;
            const_end_step = steps.begin();
            end_step = steps.begin();
            end_state_set = false;
            (*this) = other;
        }

        plan_t::~plan_t()
        {
            if( control_space != NULL )
            {
                int ss = 0;

                foreach(plan_step_t step, steps)
                {
                    //                    PRX_DEBUG_COLOR("step plan to delete: " << ss , PRX_TEXT_LIGHTGRAY);
                    control_space->free_point(step.control);
                    ++ss;
                }
                if( state_space != NULL && end_state != NULL )
                {
                    state_space->free_point(end_state);
                    end_state = NULL;
                }
                steps.clear();
            }

        }

        void plan_t::resize(unsigned num_size)
        {
            while( max_num_steps < num_size )
            {
                increase_buffer();
            }
            end_step = steps.begin();
            const_end_step = steps.begin();
            std::advance(end_step, num_steps);
            std::advance(const_end_step, num_steps);
        }

        void plan_t::link_control_space(const space_t* in_space)
        {
            if( control_space != NULL && control_space != in_space )
            {
                //Free memory in this plan
                for( unsigned i=0; i<steps.size(); ++i )
                {
                    control_space->free_point( steps[i].control );
                }
                steps.clear();

                control_space = NULL;
            }
            if( control_space == NULL )
            {
                control_space = in_space;
                steps.push_back(plan_step_t(control_space->alloc_point(), 0));
                num_steps = 0;
                max_num_steps = 1;
                end_step = steps.begin();
                const_end_step = steps.begin();
            }
        }

        void plan_t::link_state_space(const space_t* in_space)
        {
            if( state_space != NULL && state_space != in_space )
            {
                state_space->free_point( end_state );
                state_space = NULL;
            }
            if( state_space == NULL )
            {
                state_space = in_space;
                end_state = state_space->alloc_point();
                end_state_set = false;
            }
        }

        plan_t& plan_t::operator=(const plan_t& t)
        {
            if( state_space != NULL && t.state_space != NULL )
            {
                PRX_ASSERT(state_space->get_space_name() == t.state_space->get_space_name());
                if( t.end_state_set )
                {
                    state_space->copy_point(end_state, t.end_state);
                    end_state_set = true;
                }
                else
                    end_state_set = false;
            }
            else if( t.state_space != NULL )
            {
                link_state_space(t.state_space);
                if( t.end_state_set )
                {
                    state_space->copy_point(end_state, t.end_state);
                    end_state_set = true;
                }
                else
                    end_state_set = false;
            }

            if( control_space == NULL )
                control_space = t.control_space;

            if( this != &t )
            {
                //check the size of the two plans
                while( max_num_steps < t.num_steps )
                    increase_buffer();

                //clear the plan
                clear();

                foreach(plan_step_t step, t)
                {
                    control_space->copy_point((*end_step).control, step.control);
                    (*end_step).duration = step.duration;
                    ++num_steps;
                    ++end_step;
                    ++const_end_step;
                }
            }
            return *this;
        }

        plan_t& plan_t::operator+=(const plan_t& t)
        {
            PRX_ASSERT(control_space != NULL);
            unsigned new_size = t.num_steps + num_steps;
            //check the size of the two plans
            while( max_num_steps < new_size )
            {
                increase_buffer();
                end_step = steps.begin();
                const_end_step = steps.begin();
                std::advance(end_step, num_steps);
                std::advance(const_end_step, num_steps);
            }

            foreach(plan_step_t step, t)
            {
                control_space->copy_point((*end_step).control, step.control);
                (*end_step).duration = step.duration;
                ++num_steps;
                ++end_step;
                ++const_end_step;
            }
            return *this;
        }

        void plan_t::clear()
        {
            end_step = steps.begin();
            const_end_step = steps.begin();
            num_steps = 0;
        }

        void plan_t::save_to_file(std::string filename, unsigned prec) const
        {
            std::ofstream fout;
            fout.open(filename.c_str());
            save_to_stream(fout,prec);
            fout.close();
        }

        void plan_t::save_to_stream(std::ofstream& output_stream, unsigned prec) const
        {
            output_stream << num_steps << '\n';
            foreach(plan_step_t step, *this)
            output_stream << control_space->serialize_point(step.control, prec) << ',' << step.duration << '\n';

            if( end_state_set )
            {
                output_stream << "1\n";
                output_stream << state_space->print_point(end_state, prec) << "\n";
            }
            else
                output_stream << "0\n";
        }

        void plan_t::read_from_file(std::string filename)
        {
            PRX_ASSERT(control_space != NULL);

            std::ifstream input_stream;
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_input/");
            std::string file = dir + filename;
            PRX_DEBUG_S("Opening file: " << file);
            input_stream.open(file.c_str());
            read_from_stream(input_stream);
            input_stream.close();
        }

        void plan_t::read_from_stream(std::ifstream& input_stream)
        {
            PRX_ASSERT(control_space != NULL);
            if( input_stream.good() )
            {
                int number_of_steps;
                double value;
                char trash;
                std::vector<double> vals(control_space->get_dimension());
                input_stream >> number_of_steps;
                //                PRX_DEBUG_S("Number of steps: " << number_of_steps);
                control_t* control = control_space->alloc_point();
                for( int i = 0; i < number_of_steps; i++ )
                {
                    for( unsigned int j = 0; j < control_space->get_dimension(); j++ )
                    {
                        input_stream >> value;
                        //                        PRX_DEBUG_COLOR("Read in value: " << value, PRX_TEXT_CYAN);
                        vals[j] = value;
                        input_stream >> trash;
                        //                        PRX_DEBUG_COLOR("Read in trash: " << trash, PRX_TEXT_BLUE);
                    }
                    input_stream >> value; //read in duration
                    //                    PRX_DEBUG_COLOR("Duration: " << value, PRX_TEXT_CYAN);
                    //            std::cin >> trash;
                    control_space->set_from_vector(vals, control);
                    copy_onto_back(control, value);
                    //                    PRX_DEBUG_COLOR("Control read : " << control_space->print_point( control ), PRX_TEXT_LIGHTGRAY );
                }
                control_space->free_point(control);

                int end_state_check;
                input_stream >> end_state_check;
                if( end_state_check == 1 )
                {
                    PRX_ASSERT(state_space != NULL);
                    vals.clear();
                    vals.resize(state_space->get_dimension());
                    for( unsigned i = 0; i < state_space->get_dimension(); i++ )
                    {
                        input_stream >> value;
                        if( i < state_space->get_dimension() - 1 )
                            input_stream >> trash;
                        vals.push_back(value);
                    }
                    state_space->set_from_vector(vals, end_state);
                    end_state_set = true;
                }
                else
                {
                    end_state_set = false;
                }
            }
            else
            {
                PRX_WARN_S("Plan: File did not load successfully!");
            }
        }

        void plan_t::copy_onto_back(const control_t* control, double time)
        {
            PRX_ASSERT(control_space != NULL);
            if( (num_steps + 1) >= max_num_steps )
            {
                increase_buffer();
                end_step = steps.begin();
                const_end_step = steps.begin();
                std::advance(end_step, num_steps);
                std::advance(const_end_step, num_steps);
            }
            control_space->copy_point((*end_step).control, control);
            (*end_step).duration = time;
            ++end_step;
            ++const_end_step;
            ++num_steps;
        }

        void plan_t::copy_onto_front(const control_t* control, double time)
        {
            PRX_ASSERT(control_space != NULL);
            if( (num_steps + 1) >= max_num_steps )
                increase_buffer();

            plan_step_t new_step = steps.back();
            steps.pop_back();
            steps.push_front(new_step);
            control_space->copy_point((*steps.begin()).control, control);
            (*steps.begin()).duration = time;
            ++num_steps;
            end_step = steps.begin();
            const_end_step = steps.begin();
            std::advance(end_step, num_steps);
            std::advance(const_end_step, num_steps);
        }

        void plan_t::append_onto_front(double time)
        {
            PRX_ASSERT(control_space != NULL);
            if( (num_steps + 1) >= max_num_steps )
                increase_buffer();

            plan_step_t new_step = steps.back();
            steps.pop_back();
            steps.push_front(new_step);
            control_space->zero((*steps.begin()).control);
            (*steps.begin()).duration = time;
            ++num_steps;
            end_step = steps.begin();
            const_end_step = steps.begin();
            std::advance(end_step, num_steps);
            std::advance(const_end_step, num_steps);
        }

        void plan_t::append_onto_back(double time)
        {
            PRX_ASSERT(control_space != NULL);
            if( (num_steps + 1) >= max_num_steps )
            {
                increase_buffer();
                end_step = steps.begin();
                const_end_step = steps.begin();
                std::advance(end_step, num_steps);
                std::advance(const_end_step, num_steps);
            }
            control_space->zero((*end_step).control);
            (*end_step).duration = time;
            ++end_step;
            ++const_end_step;
            ++num_steps;
        }

        void plan_t::append_last_onto_back(double time)
        {
            PRX_ASSERT(control_space != NULL);
            //
            if( num_steps > 0 )
            {
                --end_step;
                (*end_step).duration += time;
                ++end_step;
            }
            else
            {
                control_space->zero((*end_step).control);
                (*end_step).duration = time;
            }

        }

        double plan_t::length() const
        {
            double sum = 0;

            foreach(const plan_step_t& step, *this)
            {
                sum += step.duration;
            }
            return sum;
        }

        std::string plan_t::print( unsigned precision ) const
        {
            std::stringstream out(std::stringstream::out);

            foreach(const plan_step_t& step, *this)
            {
                out << "[" << control_space->print_point(step.control, precision)
                        << " , " << step.duration << "s]" << std::endl;
            }
            return out.str();
        }

        control_t* plan_t::get_control_at(double time)
        {
            if( time < 0 )
            {
                PRX_FATAL_S("Plan : Trying to get a control in the past");
            }

            for( unsigned i = 0; i < num_steps; ++i )
            {
                time -= steps[i].duration;
                if( time <= 0 )
                {
                    return steps[i].control;
                }
            }

            PRX_FATAL_S("Plan : Trying to get control at an invalid time");
        }

        unsigned int plan_t::get_index_at(double time)
        {
            if( time < 0 )
            {
                PRX_FATAL_S("Plan : Trying to get a control in the past");
            }

            for( unsigned i = 0; i < num_steps; ++i )
            {
                time -= steps[i].duration;
                if( time <= 0 )
                {
                    return i;
                }
            }
            return num_steps - 1;
        }

        void plan_t::trim(double time)
        {
            PRX_ASSERT(time > 0);
            if( time > length() )
                return;

            double time_left = time;
            iterator step = steps.begin();
            // Search for the step where the trim time lies
            while( step != end_step && time_left > PRX_ZERO_CHECK )
            {
                time_left -= step->duration;
                if( time_left >= -PRX_ZERO_CHECK )
                    ++step;
            }
            // If it's in the middle of the step, keep part of the step
            if( time_left < -PRX_ZERO_CHECK )
                ++step;

            unsigned dist = std::distance(step, end_step);
            // discard the rest
            for( unsigned i = 0; i < dist; i++ )
            {
                pop_back();
            }

            // now set the final control duration to whatever is left.
            // this handles inf correctly
            if( length() > time )
            {
                back().duration = 0;
                back().duration = time - length();
                PRX_ASSERT(steps[num_steps - 1].duration > 0);
            }

            PRX_ASSERT(length() - time < PRX_ZERO_CHECK);
        }

        void plan_t::pop_front()
        {
            if( num_steps == 0 )
                PRX_FATAL_S("Trying to pop an empty plan");
            plan_step_t popped = steps[0];
            for( unsigned i = 0; i < num_steps - 1; ++i )
            {
                steps[i] = steps[i + 1];
            }
            num_steps--;
            steps[num_steps] = popped;
            end_step = steps.begin();
            const_end_step = steps.begin();
            std::advance(end_step, num_steps);
            std::advance(const_end_step, num_steps);
        }

        void plan_t::pop_back()
        {
            if( num_steps == 0 )
                PRX_FATAL_S("Trying to pop an empty plan");
            num_steps--;
            end_step = steps.begin();
            const_end_step = steps.begin();
            std::advance(end_step, num_steps);
            std::advance(const_end_step, num_steps);
        }

        control_t* plan_t::get_next_control(double time)
        {
            if( num_steps == 0 )
            {
                return NULL;
            }
            control_t* ret = steps.front().control;
            steps.front().duration -= time;
            if( steps.front().duration <= PRX_ZERO_CHECK )
            {
                pop_front();
            }
            return ret;
        }

        control_t* plan_t::consume_control(double time)
        {
            PRX_ASSERT(time > 0);
            if( num_steps == 0 )
            {
                PRX_FATAL_S("No steps in plan");
                return NULL;
            }

            control_t* ret;
            while( time > PRX_ZERO_CHECK )
            {
                ret = steps.front().control;
                double duration = steps.front().duration;
                steps.front().duration -= time;
                time -= duration;
                if( steps.front().duration <= PRX_ZERO_CHECK )
                {
                    pop_front();
                }

            }
            return ret;
        }

        plan_t& plan_t::create_augmented_plan(double plan_duration)
        {
            PRX_FATAL_S("Create Augmented Plan is not correct. It does not return the new plan");
            double actual_duration = length();
            // Initially copy over plan
            plan_t ret(control_space);

            // If we are asking for a longer plan, fill it with zero controls
            if( plan_duration >= actual_duration )
            {
                PRX_DEBUG_COLOR("Appending zero controls!", PRX_TEXT_GREEN);
                ret = (*this);
                control_t* zero_control = control_space->alloc_point();
                control_space->zero(zero_control);
                ret.copy_onto_back(zero_control, plan_duration - actual_duration);
                control_space->free_point(zero_control);
            }
                // Otherwise we are asking for a shorter plan, so we must prune it
            else
            {
                PRX_DEBUG_COLOR("Pruning plan!", PRX_TEXT_GREEN);
                // First, find the point in the plan that needs to be cut off
                int cutoff_point = 0;
                double budget = plan_duration;
                while( budget > 0 )
                {
                    budget -= steps[cutoff_point].duration;
                    cutoff_point++;
                }
                double current_duration = 0.0;
                // Append planning steps until we reach the cutoff point
                for( int i = 0; i < cutoff_point; i++ )
                {
                    ret.copy_onto_back(steps[i].control, steps[i].duration);
                    current_duration += steps[i].duration;
                }
                ret.copy_onto_back(steps[cutoff_point].control, plan_duration - current_duration);
                if( state_space )
                {
                    ret.link_state_space(state_space);
                    //TODO This seems wrong to me ZL.
                    ret.copy_end_state(end_state);
                }
            }
        }

        void plan_t::augment_plan(double plan_duration, bool use_zero_control)
        {
            double actual_duration = this->length();
            //int steps_counter = 0;
            // If we are asking for a longer plan, fill it with zero controls
            if( plan_duration >= actual_duration )
            {
                if( use_zero_control || steps.size() == 0 )
                {
                    control_t* zero_control = control_space->alloc_point();
                    control_space->zero(zero_control);
                    copy_onto_back(zero_control, plan_duration - actual_duration);
                }
                else
                {
                    int index = num_steps - 1;
                    if( index < 0 )
                        index = 0;
                    steps[index].duration += plan_duration - actual_duration;
                }
            }
                // Otherwise we are asking for a shorter plan, so we must prune it
            else
            {
                // First, find the point in the plan that needs to be cut off
                int cutoff_point = -1;
                double budget = plan_duration;
                double current_duration = 0.0;
                while( budget > PRX_ZERO_CHECK )
                {
                    cutoff_point++;
                    budget -= steps[cutoff_point].duration;
                    current_duration += steps[cutoff_point].duration;

                }
                current_duration -= steps[cutoff_point].duration;

                for( int i = num_steps - 1; i > cutoff_point; i-- )
                {
                    pop_back();
                }
                // Append planning steps until we reach the cutoff point
                steps[num_steps - 1].duration = plan_duration - current_duration;
                //        // If somehow the last step ends up with a 0s control
                //        if (steps[num_steps-1].duration <= PRX_ZERO_CHECK)
                //        {
                //            pop_back();
                //        }
            }
        }

        //It is only for kinematic systems.
        void plan_t::reverse_plan(const plan_t& t)
        {
            if( control_space == NULL )
            {
                control_space = t.control_space;
            }

            //check the size of the two plans
            while( max_num_steps < t.num_steps )
            {
                increase_buffer();
            }
            //clear the plan
            clear();


            for( int i = t.size() - 1; i >= 0; i-- )
            {
                control_space->copy_point((*end_step).control, t.at(i).control);
                (*end_step).duration = t.at(i).duration;
                ++num_steps;
                ++end_step;
                ++const_end_step;
            }
        }

        //It is only for kinematic systems.
        void plan_t::reverse()
        {
            iterator it = end_step;
            it--;
            plan_step_t tmp_step(control_space->alloc_point(), 0);
            for( int i = 0; i < size()/2; ++i )
            {
                tmp_step.copy_step(control_space, steps[i]);
                steps[i].copy_step(control_space,(*it));
                (*it).copy_step(control_space,tmp_step);
                it--;
            }
            control_space->free_point(tmp_step.control);
        }

        void plan_t::increase_buffer()
        {
            if( max_num_steps != 0 )
            {
                for( unsigned i = 0; i < max_num_steps; i++ )
                {
                    steps.push_back(plan_step_t(control_space->alloc_point(), 0));
                }
                max_num_steps *= 2;
            }
            else
            {
                steps.push_back(plan_step_t(control_space->alloc_point(), 0));
                max_num_steps = 1;
                num_steps = 0;
                end_step = steps.begin();
                const_end_step = steps.begin();
            }
        }

        void plan_t::copy_end_state(state_t* new_end_state)
        {
            PRX_ASSERT(state_space != NULL);
            state_space->copy_point(end_state, new_end_state);
            end_state_set = true;
        }

        state_t* plan_t::get_end_state() const
        {
            if( end_state_set )
                return end_state;
            else
                return NULL;
        }

        bool compare_plans(plan_t i, plan_t j)
        {
            return i.length() < j.length();
        }

        void plan_t::itemize(double simulation_step)
        {
            PRX_ASSERT(control_space != NULL);
            plan_t new_plan(control_space);

            foreach(plan_step_t step, *this)
            {
                int steps = (int)((step.duration / simulation_step) + .1);
                int i = 0;
                for( i = 0; i < steps; i++ )
                {
                    new_plan.copy_onto_back(step.control, simulation_step);
                }
            }
            *this = new_plan;

        }


    }
}
