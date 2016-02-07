/**
 * @file config.hpp
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

#ifndef PRACSYS_CONFIGURATION_HPP
#define PRACSYS_CONFIGURATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"
#include "prx/utilities/boost/hash.hpp"


namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /**
         * This class is used to track 3D configurations of objects. It uses a vector_t
         * to represent the position of the object and a quaternion for the orientation.
         *
         * @brief <b> Handles 3D configurations for objects </b>
         * @authors Athanasios Krontiris
         */
        class config_t
        {

          protected:
            /** @brief The Cartesian coordinates of this configuration */
            vector_t position;
            /** @brief The quaternion representing the orientation */
            quaternion_t orientation;

          public:
            config_t();
            config_t(const config_t& c);
            config_t(const vector_t& pos, const quaternion_t& orient);

            /**
             * Initializes the position and orientation using the parameter reader
             *
             * @brief Initializes the config_t using a parameter reader
             * @param reader The parameter reader
             */
            void init(const parameter_reader_t * const reader,const parameter_reader_t * const template_reader = NULL);

            /**
             * Calls /ref vector_t and /ref quaternion_t zero function
             *
             * @brief Zeros out the position and orientation
             */
            void zero();

            /**
             * Copies the values stored in \c source into the config
             *
             * @brief Copies a config
             * @param source The config_t to copy
             */
            void copy(const config_t& source);

            /**
             * Generates a random config such that the position is bounded
             * between [min_confg, max_config]. The orinetation is unaffected
             * by the bounds
             *
             * @note Bounding a quaternion_t makes no sense, thus, a random orientation is produced.
             *
             * @brief Generates a random config within some bounds
             * @param min_config The lower bound of the position
             * @param max_config The upper bound of the position
             */
            void random(const config_t& min_config, const config_t& max_config); // uniform

            /**
             * Returns whether the config_t is zero or not
             *
             * @brief Checks if the config_t is zero
             * @return True if the config_t is zero, false otherwise
             */
            bool is_zero() const;

            /**
             * Given another configuration, this function checks if the
             * current configuration is approximately equal by calling
             * vector_t and quaternion_t approximately equal functions.
             *
             * @brief Determines whether two configurations are approximately equal
             * @param conf The other config_t to check for approximate equality
             * @return True if the config_t's are approximately equal, false otherwise
             */
            bool is_approximate_equal(const config_t& conf) const;

            /**
             * Given a vector_t position and a quaternion_t orientation, this function
             * checks if the current configuration is approximately equal to both of these
             * parameters by calling their respective classes' approximately equal functions.
             *
             * @brief Determines whether the current configuration is approximately equal to position and orientation
             * @param pos The position to check for equality with \c position
             * @param orient The orientation to check for equality with \c orientation
             * @return True if both components are approximately equal, false otherwise
             */
            bool is_approximate_equal(const vector_t& pos, const quaternion_t& orient) const;

            /**
             * Returns the position of the config_t as a const reference
             *
             * @brief Returns the position of the config_t as a const reference
             * @return the position of the config_t as a const reference
             */
            const vector_t& get_position() const;

            /**
             * Returns the position of the config_t in the 3 coordinates
             *
             * @brief Returns the position of the config_t in the 3 coordinates
             * @param x The value of the x-coordinate in the configuration
             * @param y The value of the x-coordinate in the configuration
             * @param z The value of the x-coordinate in the configuration
             */
            void get_position(double* x, double* y, double* z) const;

            /**
             * Returns the position of the config_t in the 3 coordinates
             *
             * @brief Returns the position of the config_t in the 3 coordinates
             * @param x The value of the x-coordinate in the configuration
             * @param y The value of the x-coordinate in the configuration
             * @param z The value of the x-coordinate in the configuration
             */
            void get_position(double& x, double& y, double& z) const;

            /**
             * Returns the orientation of the config_t as a const reference
             *
             * @brief Returns the orientation of the config_t as a const reference
             * @return The orientation of the config_t as a const reference
             */
            const quaternion_t& get_orientation() const;

            /**
             * Retrieves the position and orientation of the config_t
             *
             * @brief Retrieves the position and orientation of the config_t
             * @param pos The retrieved position
             * @param quat The retrieved orientation
             */
            void get(vector_t& pos, quaternion_t& quat) const;

            /**
             * Retrieves the position
             *
             * @brief Retrives the position
             * @param pos The retrieved position
             */
            void get_position(double* pos) const;

            /**
             * Retrieves the position
             *
             * @brief Retrieves the position
             * @param pos Stores the retrieved position
             */
            void get_position(std::vector<double>& pos) const;

            /**
             * Retrieves the orientation with the following order: w,x,y,z
             *
             * @brief Retrieves the orientation
             * @param quat The retrieved orientation
             */
            void get_wxyz_orientation(double* quat) const;

            /**
             * Retrieves the orientation with the following order: x,y,z,w
             *
             * @brief Retrieves the orientation
             * @param quat The retrieved orientation
             */
            void get_xyzw_orientation(double* quat) const;

            /**
             * Sets the position using three doubles: x,y,z
             *
             * @brief Sets the position
             * @param x The x-coordinate
             * @param y The y-coordinate
             * @param z The z-coordinate
             */
            void set_position(double x, double y, double z);

            /**
             * Sets the position using another vector_t
             *
             * @brief Sets the position
             * @param pos Used to set the position
             */
            void set_position(const vector_t& pos);

            /**
             * Sets the position using a double array
             *
             * @brief Sets the position
             * @param pos Used to set the position
             */
            void set_position(const double* pos);

            /**
             * Sets the position at a specific index
             *
             * @brief Sets the position at a specific index
             * @param index The index of the position to set
             * @param val The value to set the position to
             */
            void set_position_at(int index, double val);

            /**
             * Sets the orientation using 4 doubles
             *
             * @brief Sets the orientation
             * @param x x-value
             * @param y y-value
             * @param z z-value
             * @param w w-value
             */
            void set_orientation(double x, double y, double z, double w);

            /**
             * Sets the orientation using another quaternion
             *
             * @brief Sets the orientation
             * @param quat Used to set the orientation
             */
            void set_orientation(const quaternion_t& quat);

            /**
             * Sets the orientation based on euler angles
             *
             * @brief Sets the orientation
             * @param roll Roll
             * @param pitch Pitch
             * @param yaw Yaw
             */
            void set_orientation(double roll, double pitch, double yaw);

            /**
             * Sets the orientation with the order w,x,y,z
             *
             * @brief Sets the orientation
             * @param quat Used to set the orientation
             */
            void set_wxyz_orientation(const double* quat);

            /**
             * Sets the orientation using four double with the order w,x,y,z
             *
             * @brief Sets the orientation
             * @param w w-value
             * @param x x-value
             * @param y y-value
             * @param z z-value
             */
            void set_wxyz_orientation(double w, double x, double y, double z);

            /**
             * Sets the orientation with a double array and order x,y,z,w
             *
             * @brief Sets the orientation
             * @param quat Used to set the orientation
             */
            void set_xyzw_orientation(const double* quat);

            /**
             * Sets the orientation using four doubles and order x,y,z,w
             *
             * @brief Sets the orientation
             * @param x x-value
             * @param y y-value
             * @param z z-value
             * @param w w-value
             */
            void set_xyzw_orientation(double x, double y, double z, double w);

            /**
             * Sets both the position and orientation using a vector_t and quaternion_t
             *
             * @brief Sets position and orientation
             * @param pos Used to set position
             * @param quat Used to set orientation
             */
            void set(const vector_t& pos, const quaternion_t& quat);

            /**
             * @brief Allows for position and orientation to be set with set_param
             * @param param_name The name of the parameter to be set
             * @param value The value to set the parameter with
             */
            void set_param(const std::string& param_name, const double value);

            void normalize_orientation();

            config_t& operator=(const config_t& conf);

            /**
             *  Configuration equivalence checker.
             *
             *  @brief Equivalence function
             *
             *  @param conf The input configuration to compare for equivalence
             *
             *  @return Whether the configurations are equivalent or not
             */
            bool operator ==(const config_t& conf) const;
            bool operator !=(const config_t& conf) const;

            /**
             * Add positions and multiplies orientation.
             *
             * @warning This function is not transitive!
             *
             * @param conf The configuration to add
             * @return The result of the addition
             */
            config_t operator +(const config_t& conf) const;

            /**
             * Subtracts positions and divides orientation.
             *
             * @warning This function is not transitive!
             *
             * @param conf The configuration to subtract
             * @return The result of the subtraction
             */
            config_t operator -(const config_t& conf) const;

            config_t& operator +=(const config_t& conf);
            config_t& operator -=(const config_t& conf);

            void copy_to_vector(std::vector<double>& config_vector);

            void copy_from_vector(std::vector<double>& config_vector);

            /**
             *  Interpolates for some intermediate duration from this configuration to a
             *  target configuration.
             *
             *  @brief Configuration interpolation funciton.
             *
             *  @param target The target configuration for this interpolation
             *  @param t The time on [0,1] to interpolate between the configurations.
             */
            void interpolate(const config_t& target, double t); // t is from 0 to 1

            /**
             * Interpolates for some intermediate duration from the start to the target
             * configuration and stores it to the current configuration.
             *
             * @param start The start configuration for this interpolation
             * @param target The target configuration for this interpolation
             * @param t The time on [0,1] to interpolate between the configurations.
             */
            void interpolate(const config_t& start, const config_t& target, double t); // t is from 0 to 1

            std::string print() const;

            friend std::ostream& operator<<( std::ostream& out, const config_t& config );

            /**
             * This configuration corresponds to a configuration relative to the input
             * root configuration.  Then, this function transforms the configuration
             * to a global coordinate frame.
             * @brief Transform this configuration to be a global configuration
             *
             * @param root_config The old origin configuration.
             */
            void relative_to_global(const config_t& root_config);

            /**
             * This configuration corresponds to a global configuration. Then,
             * this function transforms it to a configuration
             * relative to the input root configuration.
             * @brief Transform this configuration to be a relative configuration
             *
             * @param root_config The new origin configuration.
             */
            void global_to_relative(const config_t& root_config);

        };

        /**
         * A structure that maps a unique rigid body name to its configuration.
         *
         * A rigid body name should be its system's name with a rigid body name.
         * For example: \c car1/chassis
         */
        typedef std::vector<std::pair<std::string, config_t> > config_list_t;
        typedef std::pair<std::string, config_t> config_list_element_t;

        void augment_config_list(config_list_t& list, unsigned index);



    }
}

#endif
