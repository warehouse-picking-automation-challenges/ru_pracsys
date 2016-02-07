/**
 * @file plan.hpp
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

#ifndef PRACSYS_PLAN_HPP
#define PRACSYS_PLAN_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"

#include <deque>

namespace prx
{
    namespace sim
    {
        class distance_from_goal_t;
        /**
         * The struct that maintains the information for the plan.
         *
         * @brief <b> Maintains a step in the plan.</b>
         *
         * @author Zakary Littlefield
         */
        struct plan_step_t
        {

            /**
             * A constructor that initializes the variables of the struct.
             *
             * @param control The control of the current step.
             * @param duration The duration that the system has to execute this control.
             */
            plan_step_t(control_t* control, const double duration)
            : control(control), duration(duration){ }

            void copy_step(const util::space_t* space, const plan_step_t& step)
            {
                space->copy_point(control,step.control);
                duration = step.duration;
            }

            /** @brief The control of the current step.*/
            control_t* control;
            /** @brief The duration for that control in the plan.*/
            double duration;
        };

        /**
         * A data structure that maintains the plan. Mostly used in the planning node that a planner will
         * compute a plan for a system and then a \ref consumer_controller_t controller will follow this plan in order
         * to move the system. The plan is constructed by a sequence of controls and the duration of each
         * control. The duration is a multiple of the simulation step.
         *
         * @brief <b> A data structure that maintains a plan </b>
         *
         * @author Zakary Littlefield
         */
        class plan_t
        {

          friend class distance_from_goal_t;
          public:
            typedef std::deque<plan_step_t>::iterator iterator;
            typedef std::deque<plan_step_t>::const_iterator const_iterator;


            plan_t();

            /**
             * Constructor for creating a new plan with the appropriate \ref util::space_t for coping/deleting the
             * controls that the plan will maintain.
             *
             * @param new_space The \ref util::space_t that the plan is using for creating the controls it stores.
             */
            plan_t(const util::space_t* new_space);

            /**
             * Copy constructor, copies the plan from the \c other to the current plan.
             * @param t The source plan from where we need to copy the \ref plan_step_t s of the plan.
             */
            plan_t(const plan_t& other);

            /**
             * Frees all internal memory of the plan.
             */
            ~plan_t();

            /**
             * The current size of the plan.
             *
             * @brief The current size of the plan.
             *
             * @return The size of the plan in number of steps.
             */
            inline unsigned size() const
            {
                return num_steps;
            }
            
            /**
             * Checks if the control_space is linked.
             * 
             * @brief Checks if the control_space is linked.
             * 
             * @return True if the plan is setup with linked control_space, otherwise false.
             */
            inline bool is_initialized()
            {
                return control_space != NULL;
            }

            /**
             * Operator overload for the operator[].
             *
             * @brief Operator overload for the operator[].
             *
             * @param index The index on the plan for the \ref plan_step_t that we seek. This
             * index has to be less than \c num_steps.
             * @return A reference to the \ref plan_step_t in the position \c index.
             */
            inline plan_step_t& operator[](unsigned index)
            {
                PRX_ASSERT(index < num_steps);
                return steps[index];
            }

            /**
             * Returns a reference to the last element in the plan.
             *
             * @brief Returns a reference to the last element in the plan.
             *
             * @return A reference to the last element in the plan.
             */
            inline plan_step_t& back()
            {
                PRX_ASSERT(num_steps != 0);
                return steps[num_steps - 1];
            }

            /**
             * Returns an iterator to the first position of the plan.
             *
             * @brief Returns an iterator to the first position of the plan.
             *
             * @return An iterator to the first position of the plan.
             */
            inline iterator begin()
            {
                return steps.begin();
            }

            /**
             * Returns an iterator to the position after last of the plan, mimicking the standard library.
             *
             * @brief Returns an iterator to the  position after last of the plan.
             *
             * @return An iterator to the  position after last of the plan.
             */
            inline iterator end()
            {
                return end_step;
            }

            /**
             * Returns a \c const iterator to the first position of the plan.
             *
             * @brief Returns a \c const iterator to the first position of the plan.
             *
             * @return A \c const iterator to the first position of the plan.
             */
            inline const_iterator begin() const
            {
                return steps.begin();
            }

            /**
             * Returns a \c const iterator to the position after last of the plan, mimicking the standard library.
             *
             * @brief Returns a \c const iterator to the position after last of the plan.
             *
             * @return A \c const iterator to the position after last of the plan.
             */
            inline const_iterator end() const
            {
                return const_end_step;
            }

            /**
             * Returns a pointer to the element at the position \c index.
             *
             * @brief Returns a pointer to the element at the position \c index.
             *
             * @return A pointer to the element at the position \c index.
             */
            inline plan_step_t& at(unsigned index)
            {
                PRX_ASSERT(index < num_steps);
                return steps[index];
            }

            /**
             * Returns a pointer to the element at the position \c index.
             *
             * @brief Returns a pointer to the element at the position \c index.
             *
             * @return A pointer to the element at the position \c index.
             */
            const plan_step_t& at(unsigned index) const
            {
                PRX_ASSERT(index < num_steps);
                return steps[index];
            }

            /**
             * Allows the plan to allocate more memory. The maximum size of the plan will be expanded to hold \c num_size \ref plan_step_t's
             *
             * @brief Allocates more memory for the plan.
             *
             * @param num_size The new maximum size of the plan.
             */
            void resize(unsigned num_size);

            /**
             * Links the appropriate \ref util::space_t in the plan. Same work can be done through
             * the constructor. If during the construction the \ref util::space_t is not available then
             * you can link it to the plan through this function. It is mandatory for the
             * plan to store/delete the \ref plan_step_t that contains, and allocate memory.
             *
             * @brief Links the control \ref util::space_t in the plan.
             *
             * @param in_space The appropriate control \ref util::space_t for the plan.
             */
            void link_control_space(const util::space_t* in_space);

            /**
             * Links the appropriate state \ref util::space_t in the plan.
             *
             * @brief Links the state \ref util::space_t in the plan.
             *
             * @param in_space The appropriate state \ref util::space_t for the plan.
             */
            void link_state_space(const util::space_t* in_space);

            /**
             * Copies the plan steps into this plan's buffer. No ownership is passed.
             *
             * @brief Copies the plan steps into this plan's buffer.
             *
             * @param t The plan to copy plan steps from
             * @return Self for nested assignment.
             */
            plan_t& operator=(const plan_t& t);

            /**
             * Append the other plan's points by copying the values in.
             * No ownership is passed between the two plans.
             *
             * @brief Append the other plan's points by copying the values in.
             *
             * @param t The plan to copy points from.
             *
             * @return Self for nested statements.
             */
            plan_t& operator+=(const plan_t& t);

            /**
             * Marks the plan as empty. Does not delete any buffered memory.
             *
             * @brief Marks the plan as empty.
             *
             */
            void clear();

            /**
             * Saves the plan to a file.
             *
             * @brief Saves the plan to a file.
             *
             * @param filename The name of the file to create.
             */
            void save_to_file(std::string filename, unsigned prec = 25) const;

            /**
             * Saves the plan to a stream.
             *
             * @brief Saves the plan to a stream.
             *
             * @param output_stream The stream that we will save the plan.
             */
            void save_to_stream(std::ofstream& output_stream, unsigned prec = 25) const;

            /**
             * Reads a plan from a file. There is no error checking for control space compatibility.
             *
             * @brief Reads a plan from a file.
             *
             * @param filename The name of the file to read from.
             */
            void read_from_file(std::string filename);

            /**
             * Reads a plan from a stream. There is no error checking for control space compatibility.
             *
             * @brief Reads a plan from a stream.
             *
             * @param input_stream The stream that we will read the plan from.
             */
            void read_from_stream(std::ifstream& input_stream);

            /**
             * Copies the given control and duration to the back of the plan.
             * The caller is responsible for the passed point. Ownership is NOT passed to
             *    the buffered plan.
             *
             * @brief Copies the given control and duration to the back of the plan.
             *
             * @param control The control to copy.
             * @param time The duration to apply that control.
             */
            void copy_onto_back(const control_t* control, double time);

            /**
             * Copies the given control and duration to the front of the plan. Will shift the other points over.
             * The caller is responsible for the passed point. Ownership is NOT passed to
             *    the buffered plan.
             *
             * @brief Copies the given control and duration to the front of the plan.
             *
             * @param control The control to copy.
             * @param time The duration to apply that control.
             */
            void copy_onto_front(const control_t* control, double time);


            /**
             * Create a zero control plan step at the beginning of the plan.
             *
             * @brief Create a zero control plan step at the beginning of the plan.
             *
             * @param time The duration of the zero control
             *
             */
            void append_onto_front(double time);

            /**
             * Create a zero control plan step at the end of the plan.
             *
             * @brief Create a zero control plan step at the end of the plan.
             *
             * @param time The duration of the zero control
             *
             */
            void append_onto_back(double time);

			/**
             * Persist the last control of the plan for an additional time duration.
             *
             * @brief Persist the last control of the plan for an additional time duration.
             *
             * @param time The duration of the extended last control
             *
             */
            void append_last_onto_back(double time);

            /**
             * Pops the front step of the plan. Doesn't resize the already allocated memory
             *
             * @brief Pops the front step of the plan.
             */
            void pop_front();

            /**
             * Pops the back step of the plan. Doesn't resize the already allocated memory
             *
             * @brief Pops the back step of the plan.
             */
            void pop_back();


            /**
             * Copies the passed point as the end state of this plan. Ownership is NOT passed.
             *
             * @brief Copies the passed point as the end state of this plan.
             *
             * @param new_end_state The point to copy.
             */
            void copy_end_state(state_t* new_end_state);

            /**
             * Returns the end state.
             *
             * @brief Returns the end state.
             *
             * @return The end state.
             */
            state_t* get_end_state() const;


            /**
             * Calculates the total time to execute all valid controls in this plan.
             *
             * @brief Calculates the total time to execute all valid controls in this plan.
             *
             * @return The total length of the plan in seconds.
             */
            double length() const;

            /**
             * Print the plan out for human consumption.
             *
             * @brief Print the plan out for human consumption.
             *
             * @return A string for outputting the plan.
             */
            std::string print( unsigned precision = 20 ) const;

            /**
             * Gets the control at a specified point in time of the plan.
             *
             * @brief Gets the control at a specified point in time of the plan.
             *
             * @param time The point in time to retrieve the control.
             *
             * @return The control at time
             */
            control_t* get_control_at(double time);

            /**
             * Gets the index of the control at a specified point in time of the plan.
             *
             * @brief Gets the index of the control at a specified point in time of the plan.
             *
             * @param time The point in time to retrieve the control.
             *
             * @return The index of the control at time
             */
            unsigned int get_index_at(double time);

            /**
             * Gets the control at the start of the plan and returns it.  Consumes plan
             * duration.
             *
             * @brief Gets the control at the start of the plan and returns it.
             *
             * @param time The amount of time of the plan to consume.
             *
             * @return The next control of the plan
             */
            control_t* get_next_control(double time);

            /**
             * Gets the control at the start of the plan and returns it.  Consumes plan
             * duration.
             *
             * @brief Gets the control at the start of the plan and returns it.
             *
             * @param sim_step The amount of time of the plan to consume.
             *
             * @return The next control of the plan
             */
            control_t* consume_control(double time);

            /**
             * Truncate the plan so it lasts no longer than the given time. If the time
             * given is equal to or longer the the current length of the plan, nothing
             * is done.
             *
             * @brief Truncate the plan so it lasts no longer than the given time.
             *
             * @param time the new maximum length of the plan
             */
            void trim(double time);

            /**
             * Creates a plan that has the specified planning duration (length).
             * This has two potential effects: either the returned plan will be chopped
             * (i.e. will have less controls than it had), or it will be artificially
             * longer (i.e. will have more zero controls).
             *
             * @brief Creates a plan that has the specified planning duration (length).
             *
             * @param plan_duration The target duration of the plan
             *
             * @return A copy of the current plan, augmented to have the plan_duration
             */
            plan_t& create_augmented_plan(double plan_duration);

            /**
             * Sets the current plan to have the specified planning duration.
             * This has two potential effects: either the current plan will be chopped
             * (i.e. will have less controls than it had), or it will be artificially
             * longer (i.e. will have more zero controls).
             *
             * @brief Sets the current plan to have the specified planning duration.
             *
             * @param plan_duration The target duration of the plan
             */
            void augment_plan(double plan_duration, bool use_zero_control = true);

            /**
             * Reverses the plan without adding the last \c plan_step_t as first. After the revert the
             * user has to add onto back of the plan the final \c plan_step_t that the plan has to
             * execute in order the system will reach again its source position. If the plan already 
             * has as first control the initial position of the system then this will be the final 
             * control and the user does not have to add anything after reversing the plan.
             *
             * Warning! It is working only for kinematic systems.
             *
             * @brief Reverses the plan without adding the last \c plan_step_t as first.
             *
             * @param t the trajectory that we need to reverse.
             */
            void reverse_plan(const plan_t& t);

            /**
             * Reverses the current plan.
             *
             * Warning! It is working only for kinematic systems.
             *
             * @brief Reverses the current plan.
             */
            void reverse();


            /**
             * Splits the plan into steps of equal duration.
             */
            void itemize(double simulation_step);

          private:
            /**
             * Increases the size of the buffer, when the plan is bigger than the
             * already allocated buffer. It will increase the size by \c max_num_steps.
             *
             * @brief Increases the size of the buffer.
             */
            void increase_buffer();

            /** @brief  The corresponding control \ref util::space_t for the control points that the plan is maintaining. */
            const util::space_t* control_space;

            /** @brief  The iterator for the last element in the plan.*/
            iterator end_step;

            /** @brief  The \c const iterator for the last element in the plan*/
            const_iterator const_end_step;

            /** @brief  The current size of the plan.*/
            unsigned num_steps;

            /**
             * @brief The max size of the buffer. Also \ref plan_t::increase_buffer() will use this number for increasing the size of the buffer.
             */
            unsigned max_num_steps;

            /** The data structure that maintains the plan.*/
            std::deque<plan_step_t> steps;

            /** @brief The state space that \c end_state corresponds to. */
            const util::space_t* state_space;

            /** @brief The end state of this plan. Used as temporary storage as a convenient location. */
            state_t* end_state;

            /**
             * @brief Flag for whether the end state has been assigned to this plan.
             */
            bool end_state_set;

        };

        /**
         * Compares to plans and determines if they are the same.
         *
         * @brief Compares to plans and determines if they are the same.
         *
         * @param i The first plan.
         * @param j The second plan.
         * @return True if the two plans the same, else False.
         */
        bool compare_plans(plan_t i, plan_t j);

    }
}

#endif
