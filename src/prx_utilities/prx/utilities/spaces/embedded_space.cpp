/**
 * @file embedded_space.cpp
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

#include "prx/utilities/spaces/embedded_space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <ros/ros.h>

namespace prx
{
    namespace util
    {

        embedded_space_t::embedded_space_t(std::vector<mapping_function_t*> maps, space_t* pispace)
        {
            mappings = maps;
            preimage_space = pispace;
            std::vector<space_t*> subspaces;

            foreach(mapping_function_t* map, mappings)
            {
                subspaces.push_back(map->get_embedded_subspace());
            }

            //construct the full embedded space
            this->addresses.clear();
            local_offset = 0;
            if( subspaces.size() == 0 )
                return;


            if( subspaces[0] != NULL )
            {

                foreach(double* val, subspaces[0]->addresses)
                {
                    this->addresses.push_back(val);
                }

                space_name = subspaces[0]->get_space_name();
            }
            for( unsigned i = 1; i < subspaces.size(); i++ )
            {

                if( subspaces[i] != NULL )
                {

                    foreach(double* val, subspaces[i]->addresses)
                    {
                        this->addresses.push_back(val);
                    }
                    if( !space_name.empty() )
                        space_name += "|";

                    space_name += subspaces[i]->get_space_name();
                }
            }

            foreach(const space_t* space, subspaces)
            {
                if( space != NULL )
                {

                    foreach(bounds_t* bound, space->bounds)
                    {
                        bounds.push_back(bound);
                    }

                    foreach(topology_t top, space->topology)
                    {
                        topology.push_back(top);
                    }

                    foreach(double* scale_element, space->scale)
                    {
                        scale.push_back(scale_element);
                    }
                }
            }
            dimension = this->addresses.size();

            unsigned interval_counter = 0;
            //now that the space has been created, we can create the image intervals in the mapping.

            foreach(mapping_function_t* map_function, mappings)
            {
                map_function->preimage_space = preimage_space;
                map_function->image_space = (space_t*)this;
                map_function->image_interval = std::make_pair<unsigned, unsigned>(interval_counter, interval_counter + map_function->range);
                interval_counter += map_function->range;
            }
        }

        space_t* embedded_space_t::get_preimage_space() const
        {
            return preimage_space;
        }

        void embedded_space_t::init(const parameter_reader_t * const reader)
        {
            PRX_WARN_S("Embedded space does not use init.");
        }

        space_point_t* embedded_space_t::alloc_point() const
        {
#ifndef NDEBUG
            embedded_point_t* point = new embedded_point_t(this);
#else
            embedded_point_t* point = new embedded_point_t();
#endif

            point->memory.resize(dimension);

            copy_to_point(point);

            return point;
        }

        void embedded_space_t::free_point(space_point_t* point) const
        {
            if( dynamic_cast<embedded_point_t*>(point)->link != NULL )
                preimage_space->free_point(dynamic_cast<embedded_point_t*>(point)->link);
            delete point;
        }

        void embedded_space_t::zero(space_point_t * const point) const
        {
            PRX_ASSERT(point != NULL);
            {
                CHILD_CHECK(point)

                for( unsigned int i = 0; i < get_dimension(); ++i )
                {
                    if( topology[i] == QUATERNION )
                    {
                        point->at(i) = 0;
                        point->at(i + 1) = 0;
                        point->at(i + 2) = 0;
                        point->at(i + 3) = 1;
                        i += 3;
                    }
                    else
                    {
                        point->at(i) = 0;
                    }
                }
                preimage_space->zero(dynamic_cast<embedded_point_t*>(point)->link);
            }
        }

        void embedded_space_t::zero()
        {
            for( unsigned int i = 0; i < get_dimension(); ++i )
            {
                if( topology[i] == QUATERNION )
                {
                    (*addresses[i]) = 0;
                    (*addresses[i + 1]) = 0;
                    (*addresses[i + 2]) = 0;
                    (*addresses[i + 3]) = 1;
                    i += 3;
                }
                else
                {
                    (*addresses[i]) = 0;
                }
            }
            preimage_space->zero();
        }

        void embedded_space_t::copy_point(space_point_t * const destination, const space_point_t * const source) const
        {
            CHILD_CHECK(destination);
            CHILD_CHECK(source);

            unsigned int size = get_dimension();
            for( unsigned int i = 0; i < size; ++i )
            {
                destination->at(i) = source->at(i);
            }
            embedded_point_t* embed_dest = dynamic_cast<embedded_point_t*>(destination);
            const embedded_point_t * const embed_source = dynamic_cast<const embedded_point_t * const>(source);
            if( embed_source->link != NULL )
            {
                if( embed_dest->link == NULL )
                {
                    embed_dest->link = preimage_space->alloc_point();
                }
                preimage_space->copy_point(embed_dest->link, embed_source->link);
            }
            else
            {
                if( embed_dest->link != NULL )
                {
                    preimage_space->free_point(embed_dest->link);
                    embed_dest->link = NULL;
                }
            }
        }

        void embedded_space_t::integrate(const space_point_t * const source, const space_point_t * const delta, const double t) const
        {
            PRX_FATAL_S("Integrate should not be called in embedded space until it is needed.");

            //    CHILD_CHECK(source)
            //    CHILD_CHECK(delta)
            //
            //    embedded_point_t* source_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(source));
            //    embedded_point_t* delta_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(delta));
            //
            //    if( result_em->link == NULL )
            //        result_em->link = preimage_space->alloc_point();
            //
            //    preimage_space->integrate(get_preimage(source_em), get_preimage(delta_em), t);
            //    update_image(result_em);
        }

        void embedded_space_t::integrate(const space_point_t * const delta, const double t) const
        {
            PRX_FATAL_S("Integrate should not be called in embedded space until it is needed.");

            //    CHILD_CHECK(source)
            //    CHILD_CHECK(delta)
            //
            //    embedded_point_t* source_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(source));
            //    embedded_point_t* delta_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(delta));
            //
            //    if( result_em->link == NULL )
            //        result_em->link = preimage_space->alloc_point();
            //
            //    preimage_space->integrate(get_preimage(source_em), get_preimage(delta_em), t);
            //    update_image(result_em);
        }

        void embedded_space_t::interpolate(const space_point_t * const point1, const space_point_t * const point2, const double t, space_point_t * const result) const
        {
            CHILD_CHECK(point1)
            CHILD_CHECK(point2)
            CHILD_CHECK(result)

            embedded_point_t* point1_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(point1));
            embedded_point_t* point2_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(point2));
            embedded_point_t* result_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(result));

            if( result_em->link == NULL )
                result_em->link = preimage_space->alloc_point();

            preimage_space->interpolate(get_preimage(point1_em), get_preimage(point2_em), t, result_em->link);
            update_image(result_em);

        }

        std::string embedded_space_t::print_point(const space_point_t * const point, unsigned prec) const
        {

            CHILD_CHECK(point)

            std::stringstream out(std::stringstream::out);
            if( dimension > 0 )
            {
                out << std::fixed << std::setprecision(prec); //<< '<';
                for( unsigned int i = 0; i < get_dimension() - 1; ++i )
                    out << point->at(i) << ',';
                out << point->at(get_dimension() - 1); //<< '>';
                //                out << "\t  Full state : " ;
                //                if( dynamic_cast<const embedded_point_t * const>(point)->link == NULL )
                //                    out << "Null";
                //                else
                //                    out << preimage_space->print_point(dynamic_cast<const embedded_point_t * const>(point)->link, prec);
            }
            else
            {
                PRX_WARN_S("Trying to print an empty space!");
            }

            return out.str();
        }

        std::string embedded_space_t::serialize_point(const space_point_t * const point, unsigned prec) const
        {
            CHILD_CHECK(point)

            std::stringstream out(std::stringstream::out);
            if( dimension > 0 )
            {
                return preimage_space->serialize_point(dynamic_cast<const embedded_point_t * const>(point)->link, prec);
            }
            else
            {
                PRX_WARN_S("Trying to print an empty space!");
            }

            return out.str();
        }

        void embedded_space_t::offset(space_point_t* source, const space_point_t* target) const
        {
            PRX_FATAL_S("Offset functions of embedded space are incorrect");
            //TODO: Handle rotational topology differently
            for( unsigned i = 0; i < get_dimension(); i++ )
            {
                source->at(i) = source->at(i) + target->at(i);
            }
        }

        void embedded_space_t::offset_complement(space_point_t* source, const space_point_t* target) const
        {
            PRX_FATAL_S("Offset functions of embedded space are incorrect");
            // TODO: Handle rotational topology differently
            for( unsigned i = 0; i < get_dimension(); i++ )
            {
                source->at(i) = source->at(i) - target->at(i);
            }
        }

        void embedded_space_t::copy_to_point(space_point_t * const point) const
        {

            CHILD_CHECK(point)

            foreach(mapping_function_t* func, mappings)
            {
                func->embed();
            }

            for( unsigned i = 0; i < dimension; i++ )
            {
                point->at(i) = *(this->addresses[i]);
            }
            embedded_point_t * const point_em = dynamic_cast<embedded_point_t * const>(point);
            if( point_em->link != NULL )
            {
                preimage_space->copy_to_point(point_em->link);
            }
            else
            {
                point_em->link = preimage_space->alloc_point();
            }

        }

        void embedded_space_t::copy_from_point(const space_point_t * const point, bool enforces_bounds) const
        {
            CHILD_CHECK(point)

            for( unsigned i = 0; i < dimension; i++ )
            {
                *addresses[i] = point->at(i);
            }
            if( enforces_bounds )
                enforce_bounds();

            embedded_point_t* point_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(point));
            if( point_em->link != NULL )
            {
                preimage_space->copy_from_point(point_em->link);
            }
            else
            {

                foreach(mapping_function_t* func, mappings)
                {
                    func->invert();
                }
                //This is the best we can do in general.
                point_em->link = preimage_space->alloc_point();
            }
        }

        void embedded_space_t::set_from_vector(const std::vector<double>& values, space_point_t* point) const
        {
            PRX_ASSERT(values.size() == dimension);
            for( unsigned i = 0; i < values.size(); ++i )
                ( *addresses[i] ) = values[i];

            foreach(mapping_function_t* func, mappings)
            {
                func->invert();
            }

            PRX_ASSERT(point != NULL);
            {
                CHILD_CHECK(point);
                for( unsigned i = 0; i < values.size(); ++i )
                    point->at(i) = values[i];

                embedded_point_t* point_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(point));
                if( point_em->link == NULL )
                    point_em->link = preimage_space->alloc_point();
                else
                    preimage_space->copy_to_point(point_em->link);
            }
        }

        void embedded_space_t::set_from_vector(const std::vector<double>& values)
        {
            PRX_ASSERT(values.size() == dimension);
            for( unsigned i = 0; i < values.size(); ++i )
                ( *addresses[i] ) = values[i];

            foreach(mapping_function_t* func, mappings)
            {
                func->invert();
            }
        }

        void embedded_space_t::set_from_vector(const std::vector<double>& values) const
        {
            PRX_ASSERT(values.size() == dimension);
            for( unsigned i = 0; i < values.size(); ++i )
                ( *addresses[i] ) = values[i];

            foreach(mapping_function_t* func, mappings)
            {
                func->invert();
            }
        }

        void embedded_space_t::copy_vector_to_point(const std::vector<double>& vector, space_point_t * const point) const
        {
            this->set_from_vector(vector);
            this->copy_to_point(point);
        }

        space_point_t* embedded_space_t::get_preimage(embedded_point_t* image) const
        {
            if( image->link != NULL )
            {
                return image->link;
            }
            else
            {
                copy_from_point(image);
                return image->link;
            }
        }

        void embedded_space_t::update_image(embedded_point_t * const image) const
        {
            preimage_space->copy_from_point(image->link);
            copy_to_point(image);
        }

        void embedded_space_t::uniform_sample(space_point_t * const point) const
        {
            CHILD_CHECK(point)

            space_t::uniform_sample(point);

            for( unsigned i = 0; i < dimension; i++ )
            {
                *addresses[i] = point->at(i);
            }

            foreach(mapping_function_t* func, mappings)
            {
                func->invert();
            }

            embedded_point_t* point_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(point));
            if( point_em->link == NULL )
                point_em->link = preimage_space->alloc_point();
            else
                preimage_space->copy_to_point(point_em->link);

        }

        void embedded_space_t::uniform_sample_near(space_point_t * const point, space_point_t * const near_point, const std::vector<bounds_t*>& near_bounds) const
        {
            CHILD_CHECK(point)

            space_t::uniform_sample_near(point, near_point, near_bounds);

            for( unsigned i = 0; i < dimension; i++ )
            {
                *addresses[i] = point->at(i);
            }

            foreach(mapping_function_t* func, mappings)
            {
                func->invert();
            }

            embedded_point_t* point_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(point));
            if( point_em->link == NULL )
                point_em->link = preimage_space->alloc_point();
            else
                preimage_space->copy_to_point(point_em->link);
        }

        void embedded_space_t::uniform_sample_ball(space_point_t * const point, double radius) const
        {
            CHILD_CHECK(point)

            space_t::uniform_sample_ball(point,radius);

            for( unsigned i = 0; i < dimension; i++ )
            {
                *addresses[i] = point->at(i);
            }

            foreach(mapping_function_t* func, mappings)
            {
                func->invert();
            }

            embedded_point_t* point_em = dynamic_cast<embedded_point_t*>(const_cast<space_point_t*>(point));
            if( point_em->link == NULL )
                point_em->link = preimage_space->alloc_point();
            else
                preimage_space->copy_to_point(point_em->link);
        }

        void embedded_space_t::verify() const
        {
            space_t::verify();

            foreach(mapping_function_t* map, mappings)
            {
                map->verify();
            }
        }

    }
}
