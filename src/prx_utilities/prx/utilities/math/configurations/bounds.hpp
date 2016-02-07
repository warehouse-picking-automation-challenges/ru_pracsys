/**
 * @file bounds.hpp
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

#ifndef PRACSYS_BOUNDS_HPP
#define PRACSYS_BOUNDS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"

namespace prx
 {
 namespace util
 {

/**
 * This is the PRACSYS mathetmatical bounds class
 *
 * @brief <b> Mathematical bounds class </b>
 * @authors Athanasios Krontiris
 */

class bounds_t
{

  protected:
    /** @brief The lower bound */
    double lower;
    /** @brief The upper bound */
    double upper;

  public:
    bounds_t();

    /**
     * Default size constructor.  Creates a bounds object of size 1, and
     * initializes the bounds to the given low and high arguments.
     *
     * @brief Creates a new bounds object of size one with the given parameters.
     *
     * @param low The lower bound
     * @param high The upper bound
     */
    bounds_t(double low, double high);

    virtual ~bounds_t();

    /**
     * Clear.  Clears the lower and upper bounds vectors.
     *
     * @brief Clear function to clean the bounds.
     */
    void clear();

    /**
     * Sets the value of the upper and lower bounds at the given index to the given
     * values.
     *
     * @brief Sets an index of the upper and lower bounds to the given values.
     *
     * @param coord The index in the upper and lower bounds to set
     * @param low The value to set in the lower bounds vector
     * @param high The value to set in the upper bounds vector
     */
    void set_bounds(double low, double high);

    /**
     * Only sets the lower bound
     *
     * @brief Sets the lower bound
     * @param val The value to set the lower bound with
     */
    void set_lower_bound(double val);

    /**
     * Only sets the upper bound
     *
     * @brief Sets the upper bound
     * @param val THe value to set the upper bound with
     */
    void set_upper_bound(double val);

    bounds_t& operator=(const bounds_t& other);
    bool operator==(const bounds_t& other) const;
    bool operator!=(const bounds_t& other) const;

    // Get functions
    /**
     * Retrieves the value of the upper and lower bounds at the given index, and
     * copies them into the high and low arguments respectively.
     *
     * @brief Copies the value of the upper and lower bounds at the given index into
     * the given pointers.
     *
     * @param coord The index of the bounds to copy
     * @param low Pointer to save the lower bounds value into
     * @param high Pointer to save the upper bounds value into
     */
    void get_bounds(double& low, double& high) const;

    /**
     * Retrieves the value of the upper bound
     *
     * @brief Retrieves the value of the upper bound
     * @return The upper bound
     */
    double get_lower_bound() const;

    /**
     * Retrieves the value of the lower bound
     *
     * @brief Retrieves the value of the lower bound
     * @return The lower bound
     */
    double get_upper_bound() const;

    /**
     * Checks the coord coordinates of the upper and lower bounds vectors against the
     * given value.  If the given value is greater than the upper bound or less than
     * lower bound, the bounds value is saved into the value argument.
     *
     * @brief Checks if the given value is < lower[coord] or > upper[coord] and
     *        fixes accordingly.
     *
     * @param value Reference parameter to check against the upper and lower bound
     * @param coord The coordinate of the bound vectors to check against
     */
    double adjust(double check) const;

    /**
     * Adjusting a derivative value. A derivative should not only respect its own bounds
     * but it has to respect the bounds of the state variable. For example, if
     * x >= upper_bound[x], then x' has to be zero or negative. Similarly for x and the
     * lower bound of x.
     *
     * @brief Adjusts a value, which a derivative of a state variable, so that it also
     *        respects the constraints of the state variable.
     *
     * @param derivative   The derivative value to be adjusted
     * @param state_val    State variable to compare against the lower and upper bound
     * @param coord        The coordinate of the state variable in the bounds array
     * @param state_bounds These are the bounds for the state variable
     * @return The adjusted derivative given these constraints.
     */
    double derivative_adjust(double derivative, double state_val, const bounds_t& state_bounds) const;

    // Expand Bounds
    /**
     * Expand the "coord" coordinate of the bound vectors given the input value. e.g.,
     * If value < lower[coord], then lower[coord] = value. Similarly for the upper bound.
     *
     * @brief Expand the bounds so that they satisfy the input parameter.
     *
     * @param value Reference parameter to expand the bounds
     * @param coord The coordinate of the bound vectors to check against
     */
    void expand(double value);

    // Validity checking
    /**
     * Checks the given value to ensure that it lies between the first entry in
     * the upper and lower bounds vector.
     *
     * @brief Verifies that the given value is between the bounds vector.
     *
     * @param value The value to check against the bounds
     *
     * @return True if the given value is between the the bounds, false if not.
     */
    bool is_valid(double check) const;

    // Bounds combinations
    /**
     * Copies the given source bounds object into the values of this bounds object.
     * The lower and upper bounds values are copied from the source.
     *
     * @brief Copies the source bounds into this bounds object.
     *
     * @param source The source bounds object to copy the values from.
     */
    void copy(const bounds_t& source);

    /**
     * Intersects the bounds of this object with the given source object.  Sets the
     * lower bounds values to the maximum of this object and the source object for
     * each index of their lower bounds.  Similarly, sets the upper bounds values
     * to the minimum of this object and the source object for each index of their
     * upper bounds.
     *
     * @brief Sets the lower bounds to the max of this and the source.  Sets the
     * upper bounds to the min of this and the source.
     *
     * @param source The source vector to intersect with.
     */
    void intersect(const bounds_t& source);

    /**
     * Defines the union of the existing bounds objectand the inputobject.  Sets the
     * lower bounds values to the minimum of this object and the source object for
     * each index of their lower bounds.  Similarly, sets the upper bounds values
     * to the maximum of this object and the source object for each index of their
     * upper bounds.
     *
     * @brief Sets the lower bounds to the min of this and the source.  Sets the
     * upper bounds to the max of this and the source.
     *
     * @param source The source vector to intersect with.
     */
    void unite(const bounds_t& source);

    // Random generation of values within bounds

    /**
     * Returns a number from the uniform random distribution, bounded by the first
     * entries in the upper and lower bounds vectors.
     *
     * @brief Returns a uniform random number bounded by the limits in this class
     *
     * @param coord The coordinate of the value in the bounds' vectors.
     * @return Random number, bounded by the upper and lower bounds vectors
     */
    double uniform_random_bounds() const;

    /**
     * Returns a uniformly sampled derivative, bounded by this objects bounds and the
     * bounds related to the state variable.  If the given state_val is >= than the given upper state bound,
     * the random number will be upper bounded by zero.  Similarly, if the given state_val is <= the given
     * lower state bound, the random number will be bounded below by zero.
     *
     * @brief Returns a uniform random number for a derivative value, bounded by this
     * object's bounds or the bounds the bounds that arise from the state variable,
     * whichever dominates.
     *
     * @param state_val The value to check the bounds against
     * @param coord The coordinate of the derivative/state value in the bounds' vectors
     * @param state_bounds The bounds object for the state variable
     *
     * @return A uniform random number for the derivative value
     */
    double uniform_random_derivative(double state_val, const bounds_t& state_bounds) const;

    // Tree Verification process function
    /**
     * Used during the tree verification process.  Plants and controllers can call this function through
     * their respective spaces in order to verify that the bounds have not been invalidated through the
     * use of the set_param() function.
     *
     * @brief Verifies the validity of the bounds instance.
     */
    void verify() const;

    friend std::ostream& operator<<(std::ostream&, const bounds_t&);
};

std::ostream& operator<<(std::ostream&, const bounds_t&);

    namespace bounds
    {
        void verify(const std::vector<bounds_t*>& b);
        void verify(const std::vector<bounds_t>& b);
        void set_bounds(const std::vector<bounds_t*>& b,const std::vector<double>& lower_bounds, const std::vector<double>& upper_bounds);
        void intersect(const std::vector<bounds_t*>& b,const std::vector<bounds_t*>& lower_bounds, const std::vector<bounds_t*>& upper_bounds);
        void intersect(std::vector<bounds_t>& b,const std::vector<bounds_t>& lower_bounds, const std::vector<bounds_t>& upper_bounds);
    }



}
 }

#endif
