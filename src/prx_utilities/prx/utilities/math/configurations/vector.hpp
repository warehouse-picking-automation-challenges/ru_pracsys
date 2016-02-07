/**
 * @file vector.hpp 
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

#ifndef PRACSYS_VECTOR_HPP
#define PRACSYS_VECTOR_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <boost/numeric/ublas/vector.hpp>

namespace prx 
{ 
    namespace util 
    {
        
        
        /**
         * The vector class we will be using is from the
         * <a href="http://www.boost.org/doc/libs/1_44_0/libs/numeric/ublas/doc/vector.htm#1Vector">
         * Boost uBLAS library</a>.  Its funcitonality is similar to an \c std::vector,
         * but it is primarily for mathematical purposes.
         *
         * For convenience, I will put some of the basic things you need to know to
         * start using the Boost vector right away, as well as wrapping the vector in
         * our own namespace for convenience.
         *
         *  Membership Access : identifier( index );
         *  = Unlike an array or std::vector, we can use parenthesis () to access members of 
         *  = the vector.  You can, however, still use brackets [].  
         *
         *  - Membership Access : \code identifier( index ); \endcode
         *    Unlike an array or std::vector, we must use parenthesis to access members
         *    of the vector.
         *
         *  - Other Functions : The vector supports most kinds of operators, including
         *    \c =, \c  +=, \c =, \c *=, \c /=, etc.  It also has support for
         *    \c std::iostream operators \c >> and \c <<, as well as many functions such
         *    as \c .clear(), \c .size(), \c .resize(size_type),  etc.
         * 
         * @brief <b> Mathetmatical vector class </b>
         *
         * @authors Andrew Dobson
         */
        
        class vector_t
        {
            
        protected:
            
            /** @brief Stores the memory for the vector */
            boost::numeric::ublas::vector<double> _vec;
            
        public:
            vector_t(unsigned int dim = 0);
            vector_t(double x, double y);
            vector_t(double x, double y, double z);
            vector_t(double x, double y, double z, double w);
            vector_t(unsigned int dim, const double* vals);
            vector_t(const vector_t& other);
            ~vector_t();
            
            /**
             * Calls clear on the vector memory and resizes it to 0
             * 
             * @brief Clears the vector and resizes to 0
             */
            void clear();
            
            /**
             * Gets the dimension of the vector
             * 
             * @brief Gets the dimension of the vector
             * @return The dimension of the vector
             */
            unsigned int get_dim() const;
            
            /**
             * Retrieves the value of the vector
             * 
             * @brief Retrieves the value of the vector
             * @param index The index used to retrieve the value
             * @return The value of the vector at the index
             */
            double at(int index) const;
            
            /**
             * Retrieves the entire vector and stores it in a double array
             * 
             * @brief Retrieves the entire vector and stores it in a double array
             * @param v The double array that will store the vector
             */
            void get(double* v) const;
            
            /**
             * Retrieves the values of a 3-dimensional vector
             * 
             * @brief Retrieves the values of a 3-dimensional vector
             * @param x The value of the x-coordinate in the vector
             * @param y The value of the x-coordinate in the vector
             * @param z The value of the x-coordinate in the vector
             */
            void get(double* x, double* y, double* z) const;
            
            /**
             * Retrieves the values of a 3-dimensional vector
             * 
             * @brief Retrieves the values of a 3-dimensional vector
             * @param x The value of the x-coordinate in the vector
             * @param y The value of the x-coordinate in the vector
             * @param z The value of the x-coordinate in the vector
             */
            void get(double& x, double& y, double& z) const;
            
            /**
             * Retrieves the values of the vector and stores in a std::vector
             * 
             * @brief Retrieves the values of the vector and stores in a std::vector
             * @param params Stores the values of the vector
             */
            void get(std::vector<double>& params) const;
            
            /**
             * Sets the vector to the values passed in the double array
             * 
             * @brief Sets the vector to the values passed in the double array
             * @param v Contains the values to set the vector with
             */
            void set(const double* v);
            
            /**
             * Sets a 3-dimensional vector
             * 
             * @brief Sets a 3-dimensional vector
             * @param x Contains the x-coordinate value to set the vector with
             * @param y Contains the y-coordinate value to set the vector with
             * @param z Contains the z-coordinate value to set the vector with
             */
            void set(const double x, const double y, const double z);
            
            /**
             * Sets the vector to the values passed in the double vector
             * 
             * @brief Sets the vector to the values passed in the double vector
             * @param v Contains the values to set the vector with
             */
            void set(const std::vector<double>& params);
            
            /**
             * Sets a single value in the vector at a specific index
             * 
             * @brief Sets a single value in the vector at a specific index
             * @param index The index of the vector to set
             * @param val The value to set the vector to
             */
            void set_at(int index, double val);
            
            vector_t& operator=(const vector_t & vec);
            bool operator==(const vector_t & vec) const;
            bool operator!=(const vector_t & vec) const;
            bool operator<=(const vector_t & vec) const;
            bool operator<(const vector_t & vec) const;
            bool operator>=(const vector_t & vec) const;
            bool operator>(const vector_t & vec) const;
            
            inline const double& operator[] (unsigned int index)const
            {
                PRX_ASSERT(_vec.size() > index);
                return _vec(index);
            }
            
            inline double& operator[] (unsigned int index)
            {
                PRX_ASSERT(_vec.size() > index);
                return _vec(index);
            }
            
            vector_t& operator+=(const vector_t & vec);
            vector_t& operator-=(const vector_t & vec);
            vector_t& operator*=(const vector_t & vec);
            vector_t& operator*=(double val);
            vector_t& operator/=(const vector_t & vec);
            vector_t& operator/=(double val);
            
            vector_t operator+(const vector_t & vec) const;
            vector_t operator-(const vector_t & vec) const;
            vector_t operator-();
            vector_t operator*(const vector_t & vec) const;
            vector_t operator*(double val)const;
            vector_t operator/(const vector_t & vec) const;
            vector_t operator/(double val)const;
            
            /**
             * Sets all the values in the vector to zero
             * 
             * @brief Sets all the values in the vector to zero
             */
            void zero();
            
            /**
             * Checks if the vector is zero
             * 
             * @brief Checks if the vector is zero
             * @return True if the vector is zero, false otherwise
             */
            bool is_zero() const;
            
            /**
             * Copies the values passed into the vector
             * 
             * @brief Copies the values passed into the vector
             * @param source Contains the vector to copy values from
             */
            void copy(const vector_t& source);
            
            /**
             * Randomizes the vector to a unit circle
             * 
             * @brief Randomizes the vector to a unit circle
             */
            void random();
            
            /**
             * Randomizes the vector in such a way that:
             * vector(i) is bounded by [alpha(i), beta(i)]
             * 
             * @brief Randomizes the vector with some bounds
             * @param alpha The lower bound of the vector values
             * @param beta The upper bound of the vector values
             */
            void random(const vector_t& alpha, const vector_t& beta);
            
            /**
             * Resizes the memory allocated to the vector
             * 
             * @brief Resizes the memory allocated to the vector
             * @param s How large the vector should be resized to
             */
            void resize(unsigned int s);
            
            /**
             * The vector is approximately equal to zero if
             * each of its values is < PRX_ZERO_CHECK
             * 
             * @brief Checks if the vector is approximately zero
             * @return True if the vector is approximately zero, false otherwise
             */
            bool is_approximate_zero() const;
            
            /**
             * This function compares to vectors for equality by
             * subtracting individual elements and ensuring that
             * they do not exceed PRX_ZERO_CHECK
             * 
             * @brief Checks two vectors for approximate equality
             * @param other This vector is compared against for equality
             * @return True if the vectors are equal, false otherwise
             */
            bool is_approximate_equal(const vector_t& other) const;
            
            /**
             * Normalizes the vector to [0,1]
             * 
             * @brief Normalizes the vector to [0,1]
             */
            void normalize();
            
            /**
             * Computes the dot product with another vector
             * 
             * @brief Computes the dot product with another vector
             * @param other Used to compute the dot product
             * @return The resulting dot product
             */
            double dot_product(const vector_t& other) const;
            
            /**
             * Computes the cross product between two vectors and stores the
             * result into the current vector (overwriting it)
             * 
             * @brief Computes the cross product between two vectors and stores the result
             * @param alpha One of the vectors of the cross product
             * @param beta One of the vectors of the cross product
             */
            void cross_product(const vector_t& alpha, const vector_t& beta);
            
            /**
             * Checks if each individual element of the vector is larger than a threshold
             * 
             * @brief Checks if each individual element of the vector is larger than a threshold
             * @param val The threshold 
             * @return True if all elements are larger than the threshold, false otherwise
             */
            bool greater_than(double val);
            
            /**
             * Checks if the absolute value of each individual element is larger than a threshold
             * 
             * @brief Checks if the absolute value of each individual element is larger than a threshold
             * @param val The threshold
             * @return True if all elements are larger than the threshold, false otherwise
             */
            bool fabs_greater_than(double val);
            
            /**
             * Checks if each individual element of the vector is small than or equal to a threshold
             * 
             * @brief Checks if each individual element of the vector is small than or equal to a threshold
             * @param val The threshold
             * @return True if all elements are small than or equal to a thresohld, false otherwise
             */
            bool less_equal_than(double val);
            
            /**
             * Checks if the  absolute value of each individual element of the vector is small than or equal to a threshold
             * 
             * @brief Checks if the absolute value of each individual element of the vector is small than or equal to a threshold
             * @param val The threshold
             * @return True if all elements are small than or equal to a threshold, false otherwise
             */
            bool fabs_less_equal_than(double val);
            
            /**
             * Computes the norm
             * 
             * @brief Computes the norm
             * @return The norm
             */
            double norm() const;
            
            /**
             * Computes the squared norm
             * 
             * @brief Computes the squared norm
             * @return The squared norm
             */
            double squared_norm() const;
            
            
            /**
             * Computes the Euclidean distance between the current vector and
             * the vector passed as a parameter
             * 
             * @brief Computes the distance between the current vector and the other vector
             * @param beta Used to compute the distance
             * @return The Euclidean distance between the two vectors
             */
            double distance(const vector_t& beta) const;
            
            /**
             * Computes the squared distance between the current vector and the vector
             * passed in as a parameter
             * 
             * @brief Computes the squared distance between the current vector and the other vector
             * @param beta Used to compute the squared distance
             * @return The squared distance
             */
            double squared_distance(const vector_t& beta) const;
            
            /**
             * Interpolates from a source vector to a target vector, storing the result in the current vector.
             * 
             * The amount if interpolation is determined by \c t, which is between 0 and 1.
             * 
             * @brief Interpolates from a source vector to a target vector, storing the result in the current vector
             * @param source The source vector for interpolation
             * @param target The target vector for interpolation
             * @param t The interpolation amount between 0 and 1
             */
            void interpolate(const vector_t& source, const vector_t& target, double t); 
            
            /**
             * @brief Interpolates the current vector to a target vector
             * @param target The target vector for interpolation
             * @param t The interpolation amount between 0 and 1
             */
            void interpolate(const vector_t& target, double t); 
            
            double get_angle_between(const vector_t& target) const;
            
        };
        
        std::istream& operator>>(std::istream& input, vector_t& v);
        std::ostream& operator<<(std::ostream& output, const vector_t& v);
        
    } 
}

#endif //PRACSYS_VECTOR_HPP



