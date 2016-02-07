/**
 * @file trajectory.hpp 
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


#ifndef PRACSYS_BUFFERED_TRAJECTORY_HPP
#define PRACSYS_BUFFERED_TRAJECTORY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/state.hpp"

#include <deque>

namespace prx 
 { 
 namespace sim 
 {

/**
 * A class to maintain information about a trajectory. The trajectory will hold all the states of a system 
 * along an execution, or after a planner will plan for this system. 
 * 
 * @brief <b> Maintains a trajectory for a system </b>
 * 
 * @author Zakary Littlefield
 */
class trajectory_t
{

  public:
    typedef std::deque<state_t*>::iterator iterator;
    typedef std::deque<state_t*>::const_iterator const_iterator;

    trajectory_t();
    /**
     * Constructor of creating a new trajectory with the appropriate \ref util::space_t for coping/deleting the
     * states that the trajectory maintains. 
     * 
     * @param new_space The \ref util::space_t that the trajectory is using for the states it stores. 
     */
    trajectory_t(const util::space_t* new_space);

    /**
     * Copy constructor, copies the trajectory from the \c t to the current trajectory. 
     * @param t The source trajectory from where we need to copy the states of the trajectory. 
     */
    trajectory_t(const trajectory_t& t);

    ~trajectory_t();

    /**
     * Returns the size of the trajectory in number of states.
     * 
     * @brief Returns the size of the trajectory.
     * 
     * @return The size of the trajectory. 
     */
    inline unsigned size() const
    {
        return num_states;
    }

    /**
     * Operator overload for the operator[]. 
     * 
     * @brief Operator overload for the operator[].
     * 
     * @param index The index on the trajectory for the state that we seek. This 
     * index has to be less than \c num_states.
     * @return A pointer to the state in the position \c index.
     */
    inline state_t* operator[](unsigned index)const
    {
        PRX_ASSERT(index < num_states);
        return states[index];
    }

    /**
     * Returns a pointer to the last element in the trajectory. 
     * 
     * @brief Returns a pointer to the last element in the trajectory. 
     * 
     * @return A pointer to the last element in the trajectory. 
     */
    inline state_t* back()
    {
        PRX_ASSERT(num_states != 0);
        return states[num_states - 1];
    }

    /**
     * Returns an iterator to the first position of the trajectory. 
     * 
     * @brief Returns an iterator to the first position of the trajectory. 
     * 
     * @return An iterator to the first position of the trajectory. 
     */
    inline iterator begin()
    {
        return states.begin();
    }

    /**
     * Returns an iterator to the last position of the trajectory. 
     * 
     * @brief Returns an iterator to the last position of the trajectory. 
     * 
     * @return An iterator to the last position of the trajectory. 
     */
    inline iterator end()
    {
        return end_state;
    }

    /**
     * Returns a \c const iterator to the first position of the trajectory. 
     * 
     * @brief Returns a \c const iterator to the first position of the trajectory. 
     * 
     * @return A \c const iterator to the first position of the trajectory. 
     */
    inline const_iterator begin() const
    {
        return states.begin();
    }

    /**
     * Returns a \c const iterator to the last position of the trajectory. 
     * 
     * @brief Returns a \c const iterator to the last position of the trajectory. 
     * 
     * @return A \c const iterator to the last position of the trajectory. 
     */
    inline const_iterator end() const
    {
        return const_end_state;
    }

    /**
     * Returns a pointer to the element at the position \c index. 
     * 
     * @brief Returns a pointer to the element at the position \c index. 
     * 
     * @return A pointer to the element at the position \c index. 
     */
    state_t* at(unsigned index) const
    {
        PRX_ASSERT(index < num_states);
        return states[index];
    }

    /**
     * Links the appropriate \ref util::space_t in the trajectory. Same work can be done through 
     * the constructor. If during the construction the \ref util::space_t is not available then 
     * you can link it to the trajectory through this function. It is mandatory for the 
     * trajectory to store/delete the states that contains, and allocate memory.
     * 
     * @brief Links the \ref util::space_t in the trajectory.
     * 
     * @param in_space The appropriate \ref util::space_t for the trajectory. 
     */
    void link_space(const util::space_t* in_space);

    /**
     * Returns the sum of the distances between each point in the trajectory. 
     * This uses the default space distance function, so no custom distance functions
     * can be used here. 
     * 
     * @brief Returns the sum of the distances between each point in the trajectory.
     * 
     * @return The length of the path
     */
    double length() const;

    /**
     * Copies the points into this trajectory's buffer. No ownership is passed.
     * 
     * @brief Copies the points into this trajectory's buffer.
     * 
     * @param t The trajectory to copy points from
     * @return Self for nested assignment.
     */
    trajectory_t& operator=(const trajectory_t& t);

    /**
     * Append the other trajectory's points by copying the values in.
     * No ownership is passed between the two trajectories.
     * 
     * @brief Append the other trajectory's points by copying the values in.
     * 
     * @param t The trajectory to copy points from.
     * @return Self for nested statements.
     */
    trajectory_t& operator+=(const trajectory_t& t);


    /**
     * Marks the trajectory as empty. Does not delete any buffered memory.
     * 
     * @brief Marks the trajectory as empty.
     */
    void clear();

    /**
     * Saves the trajectory to a file.
     * 
     * @brief Saves the trajectory to a file.
     * 
     * @param filename The name of the file to create.
     */
    void save_to_file(std::string filename);

    /**
     * Saves the trajectory to a stream.
     * 
     * @brief Saves the trajectory to a stream.
     * 
     * @param output_stream The stream that the trajectory will be saved.
     */
    void save_to_stream(std::ofstream& output_stream);

    /**
     * Reads a trajectory from a file. There is no error checking for state_space compatibility.
     * 
     * @brief Reads a trajectory from a file.
     * 
     * @param filename The name of the file to read from.
     */
    void read_from_file(std::string filename);

    /**
     * Reads a trajectory from a stream. There is no error checking for state_space compatibility.
     * 
     * @brief Reads a trajectory from a stream.
     * 
     * @param input_stream The stream that we will read the trajectory.
     */
    void read_from_stream(std::ifstream& input_stream);

    /**
     * Copies the given state to the back of the trajectory.
     * The caller is responsible for the passed point. Ownership is NOT passed to 
     * the buffered trajectory.
     * 
     * @brief Copies the given state to the back of the trajectory.
     * 
     * @param state The state to copy.
     */
    void copy_onto_back(state_t* state);

    /**
     * Copies the given state to the front of the trajectory. Will shift the other points over.
     * The caller is responsible for the passed point. Ownership is NOT passed to 
     * the buffered trajectory.
     * 
     * @brief Copies the given state to the front of the trajectory.
     * 
     * @param state The state to copy.
     */
    void copy_onto_front(state_t* state);
    
    
    /**
     * Copies an interval (segment) of states from this trajectory
     * into a destination trajectory. Assumes empty destination trajectory.
     * 
     * @param start_index
     * @param end_index
     * @param copy_destination
     */
    void copy_segment(int start_index, int end_index, trajectory_t* copy_destination);

    /**
     * Resizes the trajectory. The new maximum size of the trajectory will be \c num_size.
     * 
     * @brief Resizes the trajectory.
     * 
     * @param num_size The new size of the trajectory. 
     */
    void resize(unsigned num_size);
    
    /**
     * 
     * @param splice_begin : Beginning index of the subtrajectory to remove
     * @param splice_end : End index of the subtrajectory to remove
     * @param t : Trajectory to insert in spliced section
     */
    void splice(unsigned int splice_begin, unsigned int splice_end, const trajectory_t& t);
    /**
     * 
     * @param splice_begin : Beginning index of the subtrajectory to remove
     * @param splice_end : End index of the subtrajectory to remove
     */
    void splice(unsigned int splice_begin, unsigned int splice_end);
    void chop(unsigned size);
    
    /**
     * Reverses the trajectory. Includes both the last and the first position. 
     * 
     * @param t the trajectory that we need to reverse. 
     */
    void reverse_trajectory(const trajectory_t& t);

    
    /**
     * Prints the trajectory out
     * 
     * @brief Prints the trajectory
     * @return The printed trajectory
     */
    std::string print() const;


    /**
     * Get the collision flag.
     *
     * @brief Get the collision flag.
     *
     */
    bool in_collision() const
    {
        return collision_found;
    }

    /**
     * Set the collision flag.
     *
     * @brief Set the collision flag.
     *
     */
    void set_collision(bool collision_value)
    {
        collision_found = collision_value;
    }

  private:

    /**
     *  @brief Used in planning, lets other planning modules know if this trajectory was generated with collisions.
     */
    bool collision_found;

    /**
     * Increases the buffer of the trajectory every time that the number of the states reaches the 
     * maximum size of the buffer.
     * 
     * @brief Increases the buffer of the trajectory
     *  
     */
    void increase_buffer();

    /** @brief The \ref util::space_t for the trajectory. */
    const util::space_t* state_space;

    /** @brief  The iterator pointing at the end of the trajectory. */
    iterator end_state;
    /** @brief  \c Const iterator for the final position of the trajectory.*/
    const_iterator const_end_state;
    /** @brief  The current number of states in the trajectory.*/
    unsigned num_states;
    /** @brief  The maximum number of states that the trajectory can store.*/
    unsigned max_num_states;
    /** @brief  The data structure that maintains the trajectory.*/
    std::deque<state_t*> states;
};

} 
 }

#endif
