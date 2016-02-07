/**
 * @file space.hpp
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

#ifndef PRX_SPACE_HPP
#define	PRX_SPACE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"

namespace prx
{
    namespace util
    {

        class space_t;
        class embedded_space_t;
        class mapping_function_t;
        class parameter_reader_t;

        /**
         * @brief <b> A structure that stores the current values that correspond to a \ref space_t. </b>
         */
        class space_point_t
        {

          public:

            virtual ~space_point_t(){}

            /**
             * @brief Get the memory location.
             * @param index The index of the memory to return.
             * @return The memory location.
             */
            virtual double& operator[](unsigned index)
            {
                return memory[index];
            }

            /**
             * @brief Get the memory location.
             * @param index The index of the memory to return.
             * @return The memory location.
             */
            virtual double& at(unsigned index)
            {
                return memory[index];
            }

            /**
             * @brief Get the memory location.
             * @param index The index of the memory to return.
             * @return The memory location.
             */
            virtual const double& operator[](unsigned index)const
            {
                return memory[index];
            }

            /**
             * @brief Get the memory location.
             * @param index The index of the memory to return.
             * @return The memory location.
             */
            virtual const double& at(unsigned index) const
            {
                return memory[index];
            }

#ifndef NDEBUG
            /**
             * @brief Used to check for correct space usage.
             */
            const space_t * const parent;
#endif

            /**
             * @brief The memory for this point. This is left public for time sensitive operations.
             * Should not be used generally to access memory locations. Should use \ref space_point_t::at or \ref space_point_t::operator[]
             */
            std::vector<double> memory;

          protected:

            friend class space_t;

#ifndef NDEBUG

            /**
             * @brief Constructor
             * @param in_parent The space that created this point.
             */
            space_point_t(const space_t * const in_parent) : parent(in_parent){ }
#endif

            /**
             * @brief This function should not be used.
             * @param other The point to copy.
             * @return this
             */
            space_point_t& operator=(space_point_t& other)
            {
                //TODO Check to make sure this is NOT being called.
                PRX_FATAL_S("This is a test to see if things are horribly broken");
                this->memory = other.memory;
                return *this;
            }

            /**
             * @brief C++ shouldn't allow this function to work.
             * @param other The point to copy.
             * @return this
             */
            space_point_t* operator=(space_point_t* other)
            {
                //TODO Check to make sure this is NOT being called.
                PRX_FATAL_S("This is a test to see if things are horribly broken");
                this->memory = other->memory;
                return this;
            }

        };

        /**
         * @brief <b> An encapsulation of all information needed to define a space. </b>
         *
         * An encapsulation of all information needed to define a space.
         *
         * To implement a new space, such as SE(2) or SE(3), edit the input file under
         * "PRACSYS_PATH/prx_input/prx_templates/space_types.yaml".  In this file, add a
         * name and corresponding topology for the space type. Then, when making input files,
         * specify the bounds and scales for the space.
         *
         * @author Andrew Dobson, Andrew Kimmel, Zakary Littlefield
         */
        class space_t
        {

          public:

            enum topology_t
            {

                EUCLIDEAN = 0,
                ROTATIONAL = 1,
                QUATERNION = 2,
                DISCRETE = 3
            };

            /**
             * @brief Constructor
             * @param name The name of the space to create.
             * @param addresses The memory locations to use for storage of space values.
             */
            space_t(const std::string& name, const std::vector<double*>& addresses);
            /**
             * @brief Constructor
             * @param spaces A vector of spaces to compose together. Will not recreate memory in other spaces, but will reuse the memory.
             * @param owned_space A space that will take ownwership of bounds and scale values.
             */
            space_t(const std::vector<const space_t*>& spaces, const space_t* owned_space = NULL);

            virtual ~space_t();

            /**
             * @brief Initializes topology information, bounds, and scales if available.
             * @param reader The parameter reader to request information from.
             */
            void init(const parameter_reader_t * const reader);

            /**
             * @brief Allocates a new point.
             * Allocates a new point. Will populate it with the current values stored in the space.
             * @return The new point.
             */
            virtual space_point_t* alloc_point() const;

            /**
             * @brief Frees the allocated memory in the point.
             * @param point The point to delete.
             */
            virtual void free_point(space_point_t * const point) const;

            /**
             * @brief Makes a copy of the point.
             * @param point The point to copy.
             * @return A newly allocated copy.
             */
            space_point_t* clone_point(const space_point_t * const point) const;

            /**
             * @brief Copy the internal memory values into a point.
             * @param point The point to copy into.
             */
            virtual void copy_to_point(space_point_t * const point) const;

            /**
             * @brief Copy from a point into the internal memory of the space.
             * @param point The point to copy from.
             * @oaran enforces_bounds Determines if copy from point enforces bounds
             */
            virtual void copy_from_point(const space_point_t * const point, bool enforces_bounds = true) const;

            /**
             * @brief Get the size of the space.
             * @return The size.
             */
            unsigned int get_dimension() const;

            /**
             * Finds the interval of indices that correspond to a given space. Gives the start
             * index and the index that is after the final index that is valid.  This is good
             * for looping operations.
             *
             * @brief Finds the interval of indices that correspond to a given space.
             * @param space The space to search for.
             * @return The interval that represents where \c space is in this space.
             */
            virtual std::pair<unsigned, unsigned> get_subspace(const space_t* space) const;

            /**
             * @brief Sets the bounds of the space.
             * @param inbounds The bounds to set.
             */
            virtual void set_bounds(const std::vector<bounds_t*>& inbounds);
            /**
             * @brief Sets the scales of the space.
             * @param inscales The scales to set.
             */
            virtual void set_scales(const std::vector<double>& inscales);

            /**
             * @brief Sets the values of a point from a vector of doubles.
             * @param values The values to copy into the point.
             * @param point the point to copy into.
             */
            virtual void set_from_vector(const std::vector<double>& values, space_point_t* point) const;

            /**
             * @brief Sets the values inside the space from a vector of doubles.
             * @param values The values to copy into the space.
             */
            virtual void set_from_vector(const std::vector<double>& values);

            virtual void set_from_vector(const std::vector<double>& values) const;


            /**
             * @brief Copies the values inside the space into the vector of doubles.
             * @param vector The vector to put the space information into.
             */
            virtual void copy_to_vector(std::vector<double>& vector) const;


            /**
             * @brief Copies values from a space_point into a std::vector
             * @param point The point to copy from.
             * @param vector The vector to copy into.
             */
            virtual void copy_point_to_vector(const space_point_t* const point, std::vector<double>& vector) const;

            /**
             * @brief Copies values from a std::vector into a space_point
             * @param vector The vector to copy from.
             * @param point The point to copy into.
             */
            virtual void copy_vector_to_point(const std::vector<double>& vector, space_point_t* const point) const;

            /**
             * @brief Sets the point to all zero values.
             * @param point The point to zero out.
             */
            virtual void zero(space_point_t * const point) const;

            /**
             * @brief Sets internal memory to all zero values.
             */
            virtual void zero();

            /**
             * @brief Asks the space if a certain space point satisfies the bounds of the space.
             * @param point The point to check.
             * @param equality True if boundary is included in valid region.
             * @return True if the point is inside the bounds, false otherwise.
             */
            virtual bool satisfies_bounds(const space_point_t * const point,bool equality = true) const;

            /**
             * @brief Makes the point satisfy the space bounds.
             * @param point The point to bound.
             * @param additional_bounds If provided, will override the space bounds and use those bounds instead.
             */
            virtual void enforce_bounds(space_point_t * const point, const std::vector<bounds_t*>* additional_bounds = NULL) const;

            /**
             * @brief Makes the memory of the space satisfy bounds.
             * @param additional_bounds If provided, will override the space bounds and use those bounds instead.
             */
            virtual void enforce_bounds(const std::vector<bounds_t*>* additional_bounds = NULL) const;

            /**
             * @brief Copies the values from one point into another point.
             * @param destination The point to copy into.
             * @param source The point to copy from.
             */
            virtual void copy_point(space_point_t * const destination, const space_point_t * const source) const;

            /**
             * @brief Determines if two points are equivalent.
             * @param point1 The first point to test.
             * @param point2 The second point to test.
             * @param epsilon The tolerance for equality. Helpful for numerical precision issues.
             * @return Whether the two points are within the specified tolerance or not.
             */
            virtual bool equal_points(const space_point_t * const point1, const space_point_t * const point2, double epsilon = PRX_ZERO_CHECK) const;

            /**
             * @brief Performs a simple Euler integration step using internal memory.
             * @param delta The derivative of the state.
             * @param t The amount of time to integrate.
             */
            virtual void integrate(const space_point_t * const delta, const double t) const;

            /**
             * @brief Performs a simple Euler integration step using a passed state.
             * @param source The start state.
             * @param delta The derivative of the state.
             * @param t The amount of time to integrate.
             */
            virtual void integrate(const space_point_t * const source, const space_point_t * const delta, const double t) const;

            /**
             * @brief Performs a linear interpolation between two points.
             * @param point1 The starting point (t=0)
             * @param point2 The end point (t=1)
             * @param t The scale factor (t=[0,1])
             * @param result The resulting interpolated point.
             */
            virtual void interpolate(const space_point_t * const point1, const space_point_t * const point2,
                                     const double t, space_point_t * const result) const;

            /**
             * @brief Performs a standard Euclidean distance measure between two points.
             * @param point1 The first point.
             * @param point2 The second point.
             * @return The distance.
             */
            virtual double distance(const space_point_t * const point1, const space_point_t* point2) const;

            /**
             * @brief Uniformly samples a point within this space.
             * @param point The point that will be populated with the random sample.
             */
            virtual void uniform_sample(space_point_t * const point) const;


            /**
             * @brief Uniformly samples a point within a sphere of this space's dimension centered at the origin.
             * @param point The point that will be populated with the random sample.
             * @param radius The radius of the sphere
             */
            virtual void uniform_sample_ball(space_point_t * const point, double radius) const;

            /**
             * @brief Sample uniformly around another space point within a set of bounds.
             * @param point The point that will be populated with the random sample.
             * @param near_point The point to sample around.
             * @param near_bounds The bounds around \c near_point.
             */
            virtual void uniform_sample_near(space_point_t * const point, space_point_t * const near_point, const std::vector<bounds_t*>& near_bounds) const;

            /**
             * @brief The measure of a ball in this space.
             * @param radius The radius of the ball.
             * @return The volume that ball encompasses.
             */
            virtual double n_ball_measure(double radius) const;

            /**
             * @brief Calculates an estimate of the volume of this space.
             * @return The volume.
             */
            virtual double space_size() const;

            /**
             * @brief Prints a points values.
             * @param point The point to print.
             * @param prec The precision to print values at.
             * @return A string containing the point values.
             */
            virtual std::string print_point(const space_point_t * const point, unsigned prec = 25) const;

            /**
             * @brief Outputs a points values for file I/O.
             * @param point The point to print.
             * @param prec The precision to print values at.
             * @return A string containing the point values.
             */
            virtual std::string serialize_point(const space_point_t * const point, unsigned prec = 25) const;

            /**
             * @brief Reads a point's values from a file I/O.
             *
             * @param input_stream The stream where the point is written.
             * @param point The point to store the values it will read.
             * @return A string containing the point values.
             */
            virtual space_point_t * deserialize_point(std::ifstream& input_stream) const;

            /**
             * @brief Print the internal memory values of the space.
             * @param prec The precision to print values at.
             * @return A string containing the memory values.
             */
            virtual std::string print_memory(unsigned prec = 25) const;

            /**
             * @brief Get the name of the space. (Same as from the input file)
             * @return The name.
             */
            std::string get_space_name() const;

            /**
             * @brief Return the bounds on the space.
             * @return The bounds.
             */
            std::vector<bounds_t*>& get_bounds();

            /**
             * @brief Return the bounds on the space.
             * @return The bounds.
             */
            const std::vector<bounds_t*>& get_bounds() const;


            /**
             * @brief Return the topology of the space.
             * @return The topology.
             */
            const std::vector<topology_t>& get_topology() const;

            /**
             * @brief Return the scale of the space.
             * @return The scale.
             */
            const std::vector<double*>& get_scales() const;

            /**
             * @brief Perform an element-wise offset.
             * @param start The stating point.
             * @param os The offset to apply.
             */
            void offset(space_point_t* start, const space_point_t* os) const;


            /**
             * @brief Perform an element-wise negative offset.
             * @param start The stating point.
             * @param os The offset to apply.
             */
            void offset_complement(space_point_t* start, const space_point_t* os) const;

            /**
             * @brief Check the validity of this space.
             */
            virtual void verify() const;

            /**
             * Access values from the internal memory of the space.
             * @param index The index to return.
             * @return The value at \c index.
             */
            virtual double operator[](unsigned index) const;

#ifndef NDEBUG
            /**
             * @brief Does a check to make sure that the point can be interpreted by this space.
             * @param point The point to check.
             */
            void is_compatible(const space_point_t * const point) const;
#endif

            /**
             * @brief Will be thrown if a system verification failed.
             */
            class invalid_space_exception : public std::runtime_error
            {

              public:

                invalid_space_exception(const std::string& msg) : std::runtime_error(msg){ };
            };

            friend std::ostream& operator<<(std::ostream&, const space_t&);
            friend class embedded_space_t;
            friend class mapping_function_t;

          protected:

            space_t(){ }

            /**
             * @brief Sets the topology of the space.
             * @param topology The topology to set.
             */
            virtual void set_topology(const std::vector<topology_t>& topology);

            /**
             * @brief Sets the topology of the space given a string of characters.
             * @param topology_string The topology to set.
             */
            virtual void set_topology_string(const std::string& topology_string);

            /**
             * @brief Sets scales to all 1's.
             */
            virtual void set_default_scale();
            /**
             * @brief Makes bounds infinite.
             */
            virtual void set_default_bounds();

            /**
             * @brief The internal memory locations for this space.
             */
            std::vector<double*> addresses;

            /**
             * @brief An offset for memory that this space itself is responsible for.
             */
            unsigned int local_offset;

            /**
             * @brief The bounds for this space.
             */
            std::vector<bounds_t*> bounds;

            /**
             * @brief The topology of this space.
             */
            std::vector<topology_t> topology;

            /**
             * @brief The scale of the elements in this space.
             */
            std::vector<double*> scale;

            /**
             * @brief The name of this space.
             */
            std::string space_name;

            /**
             * The number of dimensions in this space.
             */
            unsigned int dimension;

            /**
             * @brief Optimizes integration when quaternions aren't being used.
             */
            bool quaternion;

            /**
             * @brief Flag indicating whether bounds and scales are owned by this space
             */
            bool owned_bounds;
        };

        std::ostream& operator<<(std::ostream& output, const space_t& v);

    }
}

#ifndef NDEBUG
#define CHILD_CHECK(point) is_compatible(point);
#else
#define CHILD_CHECK(point)
#endif

#endif

