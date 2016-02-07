/**
 * @file embedded_space.hpp 
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

#ifndef PRX_EMBEDDED_SPACE_HPP
#define	PRX_EMBEDDED_SPACE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx
{
    namespace util
    {

        class embedded_space_t;

        /**
         * A structure that stores the current values of a point. This point also
         * carries a pointer to the point that was embedded into this point.
         * @brief <b> A structure that stores the current values of a point. </b>
         */
        class embedded_point_t : public space_point_t
        {

          public:
#ifndef NDEBUG

            /**
             * @copydoc space_point_t::space_point_t(const space_t* const)
             */
            embedded_point_t(const space_t * const parent) : space_point_t(parent)
            {
                link = NULL;
            }
#else

            embedded_point_t() : space_point_t()
            {
                link = NULL;
            }
#endif    
            /**
             * @brief The space point that was embedded to make this point.
             */
            space_point_t* link;

            friend class embedded_space_t;

        };

        /**
         * @brief <b> An encapsulation of all information needed to define a space. </b>
         *
         * An encapsulation of all information needed to define a space.
         * 
         * @author Zakary Littlefield
         * 
         */
        class embedded_space_t : public space_t
        {

          public:

            /**
             * Given a list of \ref mapping_function_t 's and a pre image space, a resulting 
             * embedded space can be created that uses the mapping functions to transform the pre image space
             * into this space. 
             *  
             * @brief Constructor
             * @param maps The list of mapping functions.
             * @param pispace The preimage space.
             */
            embedded_space_t(std::vector<mapping_function_t*> maps, space_t* pispace);

            virtual ~embedded_space_t(){ };

            /**
             * @copydoc space_t::init(const parameter_reader_t* const)
             */
            virtual void init(const parameter_reader_t * const reader);


            /**
             * @copydoc space_t::alloc_point() const
             */
            virtual space_point_t* alloc_point() const;

            /**
             * @copydoc space_t::free_point(space_point_t* const) const
             */
            virtual void free_point(space_point_t * const point) const;


            /**
             * @copydoc space_t::copy_to_point(space_point_t* const) const
             */
            virtual void copy_to_point(space_point_t * const point) const;

            /**
             * @copydoc space_t::copy_from_point(const space_point_t* const ) const
             */
            virtual void copy_from_point(const space_point_t * const point, bool enforces_bounds = true) const;

            //  
            /**
             * @copydoc space_t::set_from_vector(const std::vector<double>& values, space_point_t*) const
             */
            virtual void set_from_vector(const std::vector<double>& values, space_point_t* point) const;

            /**
             * @copydoc space_t::set_from_vector(const std::vector<double>&)
             */
            virtual void set_from_vector(const std::vector<double>& values);
            
            virtual void set_from_vector(const std::vector<double>& values) const;

            /**
             * @brief Copies values from a std::vector into a space_point
             * @param vector The vector to copy from.
             * @param point The point to copy into.
             */
            virtual void copy_vector_to_point(const std::vector<double>& vector, space_point_t* const point) const;
            /**
             * @copydoc space_t::zero(space_point* const) const
             */
            virtual void zero(space_point_t * const point) const;

            /**
             * @copydoc space_t::zero()
             */
            virtual void zero();

            /**
             * @copydoc space_t::copy_point(space_point_t* const, const space_point_t* const) const
             */
            virtual void copy_point(space_point_t * const destination, const space_point_t * const source) const;

            /**
             * @copydoc space_t::integrate(const space_point_t* const, const double) const
             */
            virtual void integrate(const space_point_t * const delta, const double t) const;

            /**
             * @copydoc space_t::integrate(const space_point_t* const, const space_point_t* const, const double) const
             */
            virtual void integrate(const space_point_t * const source, const space_point_t * const delta, const double t) const;

            /**
             * @copydoc space_t::interpolate(const space_point_t* const , const space_point_t* const, const double, space_point_t* const) const
             */
            virtual void interpolate(const space_point_t * const point1, const space_point_t * const point2,
                                     const double t, space_point_t * const result) const;

            /**
             * @copydoc space_t::print_point(const space_point_t* const, unsigned) const
             */
            virtual std::string print_point(const space_point_t * const point, unsigned prec = 25) const;

            /**
             * @copydoc space_t::serialize_point(const space_point_t* const, unsigned) const
             */
            virtual std::string serialize_point(const space_point_t * const point, unsigned prec = 25) const;

            /**
             * @copydoc space_t::offset(space_point_t*, const space_point_t*) const
             */
            void offset(space_point_t* start, const space_point_t* os) const;
            /**
             * @copydoc space_t::offset_complement(space_point_t*, const space_point_t*) const
             */
            void offset_complement(space_point_t* start, const space_point_t* os) const;


            /**
             * @copydoc space_t::uniform_sample(space_point_t* const) const
             */
            virtual void uniform_sample(space_point_t * const point) const;

            /**
             * @copydoc space_t::uniform_sample_near(space_point_t* const, space_point_t* const, const std::vector<bounds_t*>&) const
             */
            virtual void uniform_sample_near(space_point_t * const point, space_point_t * const near_point, const std::vector<bounds_t*>& near_bounds) const;

            /**
             * @brief Uniformly samples a point within a sphere of this space's dimension centered at the origin.
             * @param point The point that will be populated with the random sample.
             * @param radius The radius of the sphere
             */
            virtual void uniform_sample_ball(space_point_t * const point, double radius) const;

            /**
             * @copydoc space_t::verify() const
             */
            virtual void verify() const;

            /**
             * @brief Get the full space for this embedded space.
             * @return The full space.
             */
            space_t* get_preimage_space() const;

          protected:

            /**
             * @brief Return the full state that created the embedded point.
             * @param image The embedded point.
             * @return The full point.
             */
            space_point_t* get_preimage(embedded_point_t* image) const;

            /**
             * @brief Given a full state, perform the embedding.
             * @param image The full state.
             */
            void update_image(embedded_point_t * const image) const;

          private:

            /**
             * @brief The mappings for transforming from one space to another.
             */
            std::vector<mapping_function_t*> mappings;


            /**
             * @brief The full space that is transformed into this space.
             */
            space_t* preimage_space;
        };

    }
}

#endif

