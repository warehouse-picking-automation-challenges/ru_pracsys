/**
 * @file random.hpp 
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

#ifndef PRACSYS_RANDOM_HPP
#define PRACSYS_RANDOM_HPP

#include "prx/utilities/definitions/defs.hpp"

namespace prx
{
    namespace util
    {

        /**
         * Initializes the uniform random number generator with the given seed.
         * 
         * @brief Initializes the uniform random number generator with the given seed.
         * @param seed The value to seed the RNG with.  Default is 10.
         */
        void init_random(int seed);

        /**
         * Returns a random number from the uniform distribution [0,1).
         *
         * @brief Returns a random number from the uniform distribution [0,1).
         * @return A double precision random number.
         */
        double uniform_random();


        /**
         * Returns a random number from a Gaussian Distribution
         *
         * @brief Returns a random number from a Gaussian Distribution.
         * @return A double precision random number.
         */
        double gaussian_random();

        /**
         * Returns a random number from the uniform distribution within the
         * given range.
         * 
         * @brief Returns a random number from the uniform distribution within the given range.
         * @param min The minimum random value to return.
         * @param max The maximum random value to return.
         *
         * @return A double precision random number in the given range.
         */
        double uniform_random(double min, double max);

        /**
         * Returns a random integer number from the uniform distribution within
         * the given range.
         *
         * @brief Returns a random integer number from the uniform distribution within the given range.
         * @param min The minimum random value to return.
         * @param max The maximum random value to return.
         *
         * @return An integer random number in the given range.
         */
        int uniform_int_random(int min, int max);

        /**
         * Given a set of weights, randomly roll a "dice" and obtain an event
         * 
         * For example, given a probability distribution {10%,10%,80%} this function
         * will roll a dice weighted with those probabilities and return the index
         * of the event that occurred.
         * 
         * @brief Given a set of weights, randomly roll a "dice" and obtain an event
         * @param weights A probability distribution signifying how likely an event will occur
         * @param use_boost_random  Determines whether we call rand() or boost::rand
         * @authors Andrew Kimmel
         * 
         * @return The index of the weighted event that happened
         */
        int roll_weighted_die(std::vector<double> weights, bool use_boost_random = false);

    }
}

#endif // PRACSYS_RANDOM_HPP
