/**
 * @file quaternion.hpp
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

#ifndef PRACSYS_QUATERNION_HPP
#define PRACSYS_QUATERNION_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <boost/array.hpp>


namespace prx
{
    namespace util
    {

        class vector_t;

        /**
         * A quaternion class for representing orientations in SO(3).
         *
         * @brief <b> A quaternion class for representing orientations in SO(3).  </b>
         *
         * @authors Athanasios Krontiris
         */

        class quaternion_t
        {
        private:
            /**
             * The memory for the quaternion in the order: x,y,z,w.
             * @brief The memory for the quaternion in the order: x,y,z,w.
             */
            boost::array<double,4> q;

        public:
            quaternion_t();
            /**
             * Initializes a quaternion with the given values.
             *
             * @brief Initializes a quaternion with the given values.
             *
             * @param x The x coordinate for the quaternion.
             * @param y The y coordinate for the quaternion.
             * @param z The z coordinate for the quaternion.
             * @param w The w coordinate for the quaternion.
             */
            quaternion_t( double x, double y, double z, double w );

            /**
             * Initializes a quaternion from the given Euler angles.
             *
             * @brief Initializes a quaternion from the given Euler angles.
             *
             * @param roll Euler angel for roll.
             * @param pitch Euler angel for pitch.
             * @param yaw Euler angel for yaw.
             */
            quaternion_t( double roll, double pitch, double yaw );

            /**
             * Initializes a quaternion from a given quaternion. Copy constructor.
             *
             * @brief Copy constructor.
             *
             * @param quat The other quaternion that will be used to initialize the
             * current quaternion.
             */
            quaternion_t( const quaternion_t& quat);

            /**
             * Initializes a quaternion from the given vector. The \ref vector_t \c v
             * has 4 elements that represent the x,y,z,w values for the quaternion.
             *
             * @brief Initializes a quaternion from the given vector.
             *
             * @param v A vector with the values to initialize the quaternion.
             */
            quaternion_t( const vector_t* v);

            /**
             * The quaternion will be initialized as the angle between the two
             * vectors.
             *
             * @brief The quaternion will be initialized as the angle between the two vectors.
             *
             * @param v1 The first vector.
             * @param v2 The second vector.
             */
            quaternion_t( const vector_t& v1, const vector_t& v2 );

            /**
             * Sets the axis and rotate them based on the angle.
             *
             * @brief Sets the axis and rotate them based on the angle.
             *
             * @param axis The axis x,y,z.
             * @param angle The angle that the axis will be rotated.
             */
            quaternion_t( const vector_t& axis, double angle);

            /**
             * Sets the quaternion to 0,0,0,1.
             *
             * @brief Sets the quaternion to 0,0,0,1.
             */
            void clear();

            /**
             * Returns a vector containing x,y,z of the quaternion.
             *
             * @brief Returns a vector containing x,y,z of the quaternion.
             * @return a vector containing x,y,z of the quaternion.
             */
            vector_t get_the_vector();

            /**
             * Sets the values in a quaternion using 4 doubles.
             *
             * @brief Sets a quaternion.
             * @param x x-value.
             * @param y y-value.
             * @param z z-value.
             * @param w w-value.
             */
            void set( double x, double y, double z, double w );

            /**
             * Sets a quaternion with order x,y,z,w from the given
             * array.
             *
             * @brief Sets a quaternion.
             * @param quat Used to set the quaternion.
             */
            void set_xyzw( const double* quat );

            /**
             * Sets a quaternion with order w,x,y,z from the given
             * array.
             *
             * @brief Sets a quaternion.
             * @param quat Used to set the quaternion.
             */
            void set_wxyz( const double* quat );

            /**
             * Sets a quaternion from the given vector.
             *
             * @brief Sets a quaternion.
             * @param vec Used to set the quaternion.
             */
            void set( const vector_t& vec );

            /**
             * Sets a quaternion based on a rotation around the axis.
             *
             * @brief Sets a quaternion based on a rotation around the axis.
             * @param axis The axis to rotate about.
             * @param angle The angle of rotation.
             */
            void set( const vector_t& axis, double angle);

            void set_from_euler( double roll, double pitch, double yaw );

            /**
             * Retrieves the x,y,z,w values of this quaternion and copies them into the
             * given x,y,z,w reference arguments.
             *
             * @brief Copies the quaternion values into the reference arguments.
             *
             * @param x Reference argument for the x value.
             * @param y Reference argument for the y value.
             * @param z Reference argument for the z value.
             * @param w Reference argument for the w value.
             */
            void get( double& x, double& y, double& z, double& w ) const;

            /**
             * Retrieves the x,y,z,w values of this quaternion and copies them into
             * an array of 4 values, ordered as x, y, z, w.
             *
             * @brief Copies the quaternion values into the reference argument.
             *
             * @param quat  The array where the x,y,z,w values are stored.
             */
            void get_xyzw( double* quat ) const;

            /**
             * Retrieves the x,y,z,w values of this quaternion and copies them into
             * an array of 4 values, ordered as w, x, y, z.
             *
             * @brief Copies the quaternion values into the reference argument.
             *
             * @param quat  The array where the w,x,y,z values are stored.
             */
            void get_wxyz( double* quat ) const;

            /**
             * Retrieves the x,y,z,w values of this quaternion and copies them into the
             * given argument vector.
             *
             * @brief Copies the quaternion values into the argument vector.
             *
             * @param vals The vector to copy the quaternion values into.
             */
            void get( vector_t& vec ) const;

            /**
             * Operator overload for the operator [] in order to use the quaternion as
             * an array with fix size 4.
             *
             * @brief Operator overload for the operator [].
             *
             * @param index The index of the value in the quaternion.
             * @return The value at the position \c index.
             */
            double operator[](const unsigned int index) const;

            /**
             * Operator overload for the operator [] in order to use the quaternion as
             * an array with fix size 4.
             *
             * @brief Operator overload for the operator [].
             *
             * @param index The index of the value in the quaternion.
             * @return The value at the position \c index.
             */
            double& operator[](const unsigned int index);

            /**
             * Gets the x-component.
             *
             * @brief Gets the x-component.
             * @return The x-component.
             */
            double get_x() const;

            /**
             * Gets the y-component.
             *
             * @brief Gets the y-component.
             * @return The y-component.
             */
            double get_y() const;

            /**
             * Gets the z-component.
             *
             * @brief Gets the z-component.
             * @return The z-component.
             */
            double get_z() const;

            /**
             * Gets the w-component.
             *
             * @brief Gets the w-component.
             * @return The w-component.
             */
            double get_w() const;

            quaternion_t& operator = ( const quaternion_t& quat);
            bool operator == ( const quaternion_t& quat) const;
            bool operator != ( const quaternion_t& quat) const;

            quaternion_t operator + ( const quaternion_t& quat) const;
            quaternion_t operator - ( const quaternion_t& quat) const;
            quaternion_t operator - (); //Unary Minus

            /**
             * Computes the product of the object quaternion and the given argument quaternion.
             *
             * @brief Multiplies this quaternion with the argument quaternion.
             *
             * @param quat The quaternion to multiply the values with.
             */
            quaternion_t operator * ( const quaternion_t& quat) const;
            quaternion_t operator * ( double val) const;

            /**
             * Computes the quotient of this quaternion and the given argument quaternion.
             * The computed quotient is then saved as this quaternion.
             *
             * @brief Divides this quaternion by the argument quaternion.
             *
             * @param quat Pointer to the quaternion to divide values from.
             */
            quaternion_t operator / ( const quaternion_t& quat) const;
            quaternion_t operator / ( double val) const;

            /**
             * Computes the conjugate of the quaternion.
             *
             * @brief Computes the conjugate of the quaternion.
             * @return The inverted quaternion.
             */
            quaternion_t conj () const;

            quaternion_t& operator += ( const quaternion_t& quat);
            quaternion_t& operator -= ( const quaternion_t& quat);
            quaternion_t& operator *= ( const quaternion_t& quat);
            quaternion_t& operator *= ( double val);
            quaternion_t& operator /= ( const quaternion_t& quat);
            quaternion_t& operator /= ( double val);


            /**
             * Zeroes out all four values of the quaternion.
             *
             * @brief Zeroes out all four values of the quaternion.
             */
            void zero();

            /**
             * Checks if the quaternion is zero.
             *
             * @brief Checks if the quaternion is zero.
             * @return True if the quaternion is zero, false otherwise.
             */
            bool is_zero() const;

            /**
             * Copies a source quaternion into the current quaternion.
             *
             * @brief Copies a source quaternion into the current quaternion.
             * @param source The source quaternion containing the values to be copied.
             */
            void copy( const quaternion_t& source);

            /**
             * Randomizes the values within the quaternion.
             *
             * @brief Randomizes the values within the quaternion.
             */
            void random();

            /**
             * Checks if the quaternion is approximately zero.
             *
             * @brief Checks if the quaternion is approximately zero.
             * @return True if the quaternion is approximately zero, fales otherwise.
             */
            bool is_approximate_zero() const;

            /**
             * Checks if the quaternion is approximately equal to another quaternion.
             *
             * @brief Checks if the quaternion is approximately equal to another quaternion.
             * @param other The quaternion to check for equality against.
             * @return True if the quaternions are approximately equal, false otherwise.
             */
            bool is_approximate_equal( const quaternion_t& other ) const;

            /**
             * Quaternion self-check to see if the quaternion's values constitute a
             * valid quaternion.
             *
             * @brief Checks the validity of the quaternion.
             * @return True if the quaternion is valid, otherwise false.
             */
            bool is_valid() const;

            /**
             * Rectifies the quaternion so that it is valid, but may make the rotation
             * not exactly what was originally intended.
             * @brief Rectifies the quaternion so that it is valid.
             */
            void rectify();

            /**
             * Computes the norm of the quaternion values.  This method computes the
             * square root of the sum of the squared values and returns it.
             *
             * @brief Computes and returns the norm of the quaternion values.
             *
             * @return Returns the square root of the sum of the squared values
             * of the quaternion.
             */
            double norm() const;

            /**
             * Computes the squared norm of the quaternion values.  This method computes the
             * sum of the squared values and returns it.
             *
             * @brief Computes and returns the squared norm of the quaternion values.
             *
             * @return Returns the sum of the squared values
             * of the quaternion.
             */
            double squared_norm() const;

            /**
             * Computes the square root of the norm of the quaternion values.
             *
             * @brief Computes and returns the square root of the norm of the quaternion.
             *
             * @return The square root of the norm of the quaternion.
             */
            double modulus();

            /**
             * Normalizes the values of the quaternion by dividing the values by
             * their modulus (square root of the norm).
             *
             * @brief Normalizes all values of the quaternion.
             */
            void normalize();

            /**
             * Computes and saves the inverse of the quaternion values to this one.  This
             * method divides the values of the quaternion by the norm of the quaternion.
             *
             * @brief Sets all quaternion values to their inverse.
             */
            void inverse();

            /**
             * Negates the y,z, and w values of the quaternion.
             *
             * @brief Negates the y,z, and w values of the quaternion.
             */
            void conjugate();

            /**
             * Performs conversion on the quaternion so that euler angles
             * can be produced. The result is stored into the vector_t \c u
             * that is passed by reference.
             *
             * @brief Converts the quaternion into euler angles.
             * @param u The converted quaternion angles.
             */
            void convert_to_euler (vector_t& u) const;

            /**
             * Computes the distance between this quaternion and the given argument
             * quaternion.
             *
             * @brief Returns the distance between this and the given quaternion.
             *
             * @param other Quaternion to compute the distance to.
             */
            double distance( const quaternion_t& other ) const;

            /**
             * Computes the interpolation between this quaternion and the given target
             * quaternion.  The quaternion computed by interpolating the two quaternions is
             * then saved as this quaternion.
             *
             * @brief Interpolates this quaternion with the target quaternion.
             *
             * @param target The quaternion to interpolate with.
             * @param f The interpolation constant.
             */
            void interpolate( const quaternion_t& target, double f ); //t is on [0,1]

            /**
             * Computes the interpolation between the source quaternion and the given target
             * quaternion.  The quaternion computed by interpolating the two quaternions is
             * then saved as this quaternion.
             *
             * @brief Interpolates this quaternion with the target quaternion.
             *
             * @param source The quaternion to interpolate from.
             * @param target The quaternion to interpolate to.
             * @param f The interpolation constant.
             */
            void interpolate( const quaternion_t& source, const quaternion_t& target, double f ); //t is on [0,1]

            /**
             * Rotates the quaternion using a vector_t and returns
             * the vector component of the resulting rotation.
             *
             * @brief Rotates the quaternion using a vector_t.
             * @param v The rotation vector_t.
             * @return The vector component of the rotated quaternion.
             */
            vector_t qv_rotation(const vector_t& v) const;

            /**
             * Computes the rotational angle between the two vectors.
             *
             * @brief Computes the rotational angle between the two vectors.
             *
             * @param v1 The first vector.
             * @param v2 The second vector.
             */
            void compute_rotation(const vector_t& v1, const vector_t& v2);

            friend std::istream& operator>>( std::istream&, quaternion_t& );
            friend std::ostream& operator<<( std::ostream&, const quaternion_t& );
        };

        std::istream& operator>>( std::istream& input, quaternion_t& q );
        std::ostream& operator<<( std::ostream& output, const quaternion_t& q );

        quaternion_t operator* ( const quaternion_t &q, const vector_t& v);
        quaternion_t operator* ( const vector_t& v, const quaternion_t &q);


    }
}

#endif

