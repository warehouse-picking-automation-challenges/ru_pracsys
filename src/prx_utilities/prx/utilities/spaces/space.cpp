/**
 * @file space.cpp
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

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/random.hpp"

#include <ros/ros.h>
#include <fstream>
#include <sstream>

namespace prx
{
    namespace util
    {
        space_t::space_t(const std::string& name, const std::vector<double*>& addresses)
        {
            //Space information is specified under the "spaces" namespace.  This information
            //is populated by the file: "prx_root/prx_input/prx_templates/space_types.yaml"
            parameter_reader_t reader("prx/spaces");
            dimension = addresses.size();
            PRX_DEBUG_S("SPACES " << name << "    dimension: " << dimension);
            local_offset = 0;

            std::string topo;
            if( reader.has_attribute(name) )
                topo = reader.get_attribute(name);
            else
            {
                //composition spaces (can only be created in code.)
                std::vector<std::string> elems;
                std::stringstream ss(name);
                std::string item;
                while( std::getline(ss, item, '|') )
                {
                    elems.push_back(item);
                }

                foreach(std::string elem, elems)
                {
                    if( reader.has_attribute(elem) )
                        topo += reader.get_attribute(elem);
                    else
                        throw invalid_space_exception("Unknown space type: " + elem);
                }
            }

            space_name = name;
            quaternion = false;

            set_topology_string(topo);

            PRX_ASSERT(topology.size() == addresses.size());

            this->addresses.clear();

            foreach(double* val, addresses)
            {
                this->addresses.push_back(val);
            }


            for( unsigned i = 0; i < dimension; i++ )
            {
                bounds.push_back(new bounds_t());
                scale.push_back(new double);
            }
            set_default_bounds();
            set_default_scale();

            owned_bounds = true;
        }

        space_t::space_t(const std::vector<const space_t*>& spaces, const space_t* owned_space)
        {
            this->addresses.clear();
            local_offset = 0;

            //existing space elements

            //    if (spaces[0] != NULL)
            //    {
            //        foreach(double* val, spaces[0]->addresses)
            //        {
            //            this->addresses.push_back(val);
            //            local_offset++;
            //        }
            //
            //    space_name = spaces[0]->get_space_name();
            //    }
            for( unsigned i = 0; i < spaces.size(); i++ )
            {

                if( spaces[i] != NULL && spaces[i]->get_space_name() != "EMPTY" )
                {

                    foreach(double* val, spaces[i]->addresses)
                    {
                        this->addresses.push_back(val);
                        local_offset++;
                    }
                    if( !space_name.empty() )
                        space_name += "|";
                    space_name += spaces[i]->get_space_name();

                    //            PRX_ERROR_S ("Space_name: " << space_name);
                }
            }
            //    if (!space_name.empty())
            //        space_name.erase(space_name.size()-1);

            foreach(const space_t* space, spaces)
            {

                if( space != NULL && space->get_space_name() != "EMPTY" )
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

            PRX_ASSERT(local_offset == bounds.size());
            PRX_ASSERT(local_offset == topology.size());
            PRX_ASSERT(local_offset == scale.size());

            //new space elements
            dimension = this->addresses.size();

            if( owned_space != NULL )
            {

                foreach(double* val, owned_space->addresses)
                {
                    this->addresses.push_back(val);
                }

                dimension = this->addresses.size();
                for( unsigned i = local_offset; i < dimension; i++ )
                {
                    bounds.push_back(owned_space->bounds[i - local_offset]);
                    scale.push_back(owned_space->scale[i - local_offset]);
                    topology.push_back(owned_space->topology[i - local_offset]);
                }

                space_name += "|";
                space_name += owned_space->get_space_name();
            }
            owned_bounds = false;
        }

        space_t::~space_t()
        {
            if( owned_bounds )
            {
                for( unsigned i = 0; i < dimension; i++ )
                {
                    delete bounds[i];
                    delete scale[i];
                }
            }
        }

        void space_t::init(const parameter_reader_t * const reader)
        {

            // Must have both "min" and "max" attributes or neither.
            // The '||' ensures the reader throws an error if only one is present.
            if( reader->has_attribute("min") || reader->has_attribute("max") )
            {
                const std::vector<double> min = reader->get_attribute_as<std::vector<double> > ("min");
                const std::vector<double> max = reader->get_attribute_as<std::vector<double> > ("max");

                bounds::set_bounds(bounds, min, max);
            }
            else
                set_default_bounds();

            if( reader->has_attribute("scale") )
            {
                set_scales(reader->get_attribute_as<std::vector<double> > ("scale"));
            }
            else
                set_default_scale();
        }

        space_point_t* space_t::alloc_point() const
        {
#ifndef NDEBUG
            space_point_t* point = new space_point_t(this);
#else
            space_point_t* point = new space_point_t();
#endif

            point->memory.resize(dimension);
            copy_to_point(point);

            return point;
        }

        void space_t::free_point(space_point_t* point) const
        {
            delete point;
        }

        void space_t::verify() const
        {
            bounds::verify(bounds);

            if( topology.size() != dimension )
                throw invalid_space_exception("Dimensionality of topology (" + int_to_str(topology.size()) +
                                              ") doesn't match dimensionality of space (" + int_to_str(dimension) + ").");

            if( bounds.size() != dimension )
                throw invalid_space_exception("Dimensionality of bounds (" + int_to_str(bounds.size()) +
                                              ") doesn't match dimensionality of space (" + int_to_str(dimension) + ").");

            if( scale.size() != dimension )
                throw invalid_space_exception("Dimensionality of scale (" + int_to_str(scale.size()) +
                                              ") doesn't match dimensionality of space (" + int_to_str(dimension) + ").");
            if( addresses.size() != dimension )
                throw invalid_space_exception("Dimensionality of addresses (" + int_to_str(addresses.size()) +
                                              ") doesn't match dimensionality of space (" + int_to_str(dimension) + ").");
        }

        double space_t::operator[](unsigned index) const
        {
            return *(addresses[index]);
        }

        void space_t::zero(space_point_t * const point) const
        {
            CHILD_CHECK(point)

            for( unsigned int i = 0; i < get_dimension(); ++i )
            {
                if( topology[i] == QUATERNION )
                {
                    point->memory[i] = 0;
                    point->memory[i + 1] = 0;
                    point->memory[i + 2] = 0;
                    point->memory[i + 3] = 1;
                    i += 3;
                }
                else
                {
                    point->memory[i] = 0;
                }
            }

        }

        void space_t::zero()
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
        }

        std::pair<unsigned, unsigned> space_t::get_subspace(const space_t* space) const
        {
            unsigned begin = dimension + 1;
            unsigned end = dimension + 1;
            unsigned space_iter = 0;

            if( space != NULL && space->get_dimension() > 0 )
            {
                for( unsigned i = 0; i < dimension; i++ )
                {

                    if( space->addresses[space_iter] == this->addresses[i] )
                    {
                        if( begin == end )
                        {
                            begin = i;
                            end = i + 1;
                        }
                        else
                        {
                            end++;
                        }
                        space_iter++;
                        if( space_iter == space->dimension )
                        {
                            break;
                        }
                    }
                    else if( begin != end )
                    {
                        begin = end = dimension + 1;
                        break;
                    }
                }
            }
            return std::make_pair(begin, end);
        }

        unsigned space_t::get_dimension() const
        {
            return dimension;
        }

        std::vector<bounds_t*>& space_t::get_bounds()
        {
            return bounds;
        }

        const std::vector<bounds_t*>& space_t::get_bounds() const
        {
            return bounds;
        }

        const std::vector<space_t::topology_t>& space_t::get_topology() const
        {
            return topology;
        }

        const std::vector<double*>& space_t::get_scales() const
        {
            return scale;
        }

        bool space_t::equal_points(const space_point_t * const point1, const space_point_t * const point2, double epsilon) const
        {
            CHILD_CHECK(point1)
            CHILD_CHECK(point2)

            return distance(point1, point2) <= epsilon;

        }

        double space_t::n_ball_measure(double radius) const
        {
            double dim = 0;

            for( unsigned i = 0; i < dimension; ++i )
            {
                if( scale[i] != 0 )
                {
                    if( topology[i] == EUCLIDEAN || topology[i] == ROTATIONAL )
                        ++dim;
                }
            }

            double ret = (pow(M_PI, dim / 2.0) / tgamma((dim / 2.0) + 1.0)) * pow(radius, dim);

            return ret;
        }

        double space_t::space_size() const
        {
            double mult = 1;
            for( unsigned i = 0; i < dimension; ++i )
            {
                if( scale[i] != 0 )
                {
                    mult *= (bounds[i]->get_upper_bound() - bounds[i]->get_lower_bound());
                }
            }
            return mult;
        }

        std::ostream& operator<<(std::ostream& output, const space_t& v)
        {
            output << "Space name: " << v.space_name << "\n";
            output << v.dimension << "-dimensional space";
            output << "\nBounds: [";
            for( unsigned i = 0; i < v.bounds.size(); i++ )
            {
                output << *v.bounds[i] << ' ';
            }
            output << ']';
            output << "\nScale: [";
            for( unsigned i = 0; i < v.scale.size(); i++ )
            {
                output << *v.scale[i] << ' ';
            }
            output << ']';
            output << "\nTopology: [";
            for( unsigned i = 0; i < v.topology.size(); i++ )
            {
                if( v.topology[i] == space_t::EUCLIDEAN )
                {
                    output << 'E';
                }
                else if( v.topology[i] == space_t::ROTATIONAL )
                {
                    output << 'R';
                }
                else if( v.topology[i] == space_t::QUATERNION )
                {
                    output << 'Q';
                }
                else if( v.topology[i] == space_t::DISCRETE )
                {
                    output << 'D';
                }
            }
            output << ']';
            return output;
        }

        std::string space_t::get_space_name() const
        {
            return this->space_name;
        }

        bool space_t::satisfies_bounds(const space_point_t * const point, bool equality) const
        {
            CHILD_CHECK(point)

            PRX_ASSERT(dimension == get_bounds().size());
            PRX_ASSERT(dimension == topology.size());

            for( unsigned int i = 0; i < dimension; ++i )
            {
                if( topology[i] == QUATERNION )
                {
                    quaternion_t quat;
                    quat.set(point->memory[i], point->memory[i + 1], point->memory[i + 2], point->memory[i + 3]);
                    if( !quat.is_valid() )
                        return false;
                    i += 3;
                }
                else
                {
                    if( equality && (point->memory[i] > bounds[i]->get_upper_bound() || point->memory[i] < bounds[i]->get_lower_bound()) )
                        return false;
                    else if( !equality && (point->memory[i] >= bounds[i]->get_upper_bound() || point->memory[i] <= bounds[i]->get_lower_bound()) )
                        return false;
                }
            }

            return true;
        }

        void space_t::enforce_bounds(space_point_t* point, const std::vector<bounds_t*>* additional_bounds) const
        {
            CHILD_CHECK(point)

            PRX_ASSERT(dimension == topology.size());
            if( additional_bounds == NULL )
            {
                additional_bounds = &bounds;
            }
            else
            {
                PRX_ASSERT(dimension == additional_bounds->size());
            }

            unsigned int size = dimension;
            for( unsigned int i = 0; i < size; ++i )
            {
                if( topology[i] == QUATERNION )
                {
                    quaternion_t q;
                    q.set(point->memory[i], point->memory[i + 1], point->memory[i + 2], point->memory[i + 3]);
                    q.rectify();
                    q.get(point->memory[i], point->memory[i + 1], point->memory[i + 2], point->memory[i + 3]);
                    i += 3;
                }
                else if( topology[i] == ROTATIONAL )
                {
                    //this seems to be a problem that was never addressed before
                    double& p = point->memory[i];
                    while (p > PRX_PI)
                        p -= 2.0 * PRX_PI;
                    while (p < -PRX_PI)
                        p += 2.0 * PRX_PI;
                    // p = fmod(p, 2 * PRX_PI);

                    // //move into -pi to pi range
                    // if( p < -1 * PRX_PI )
                    // {
                    //     p += 2 * PRX_PI;
                    // }
                    // if( p > PRX_PI )
                    // {
                    //     p -= 2 * PRX_PI;
                    // }

                    p = PRX_MAXIMUM(p, (*additional_bounds)[i]->get_lower_bound());
                    p = PRX_MINIMUM(p, (*additional_bounds)[i]->get_upper_bound());
                }
                else
                {
                    double& p = point->memory[i];
                    p = PRX_MAXIMUM(p, (*additional_bounds)[i]->get_lower_bound());
                    p = PRX_MINIMUM(p, (*additional_bounds)[i]->get_upper_bound());
                }
            }
        }

        void space_t::enforce_bounds(const std::vector<bounds_t*>* additional_bounds) const
        {
            PRX_ASSERT(dimension == topology.size());
            if( additional_bounds == NULL )
            {
                additional_bounds = &bounds;
            }
            else
            {
                PRX_ASSERT(dimension == additional_bounds->size());
            }

            unsigned int size = dimension;
            for( unsigned int i = 0; i < size; ++i )
            {
                if( topology[i] == QUATERNION )
                {
                    quaternion_t q;
                    q.set((*addresses[i]), (*addresses[i + 1]), (*addresses[i + 2]), (*addresses[i + 3]));
                    q.rectify();
                    q.get((*addresses[i]), (*addresses[i + 1]), (*addresses[i + 2]), (*addresses[i + 3]));
                    i += 3;
                }
                else if( topology[i] == ROTATIONAL )
                {
                    //this seems to be a problem that was never addressed before
                    double& p = (*addresses[i]);

                    p = fmod(p, 2 * PRX_PI);

                    //move into -pi to pi range
                    if( p < -1 * PRX_PI )
                    {
                        p += 2 * PRX_PI;
                    }
                    if( p > PRX_PI )
                    {
                        p -= 2 * PRX_PI;
                    }

                    p = PRX_MAXIMUM(p, (*additional_bounds)[i]->get_lower_bound());
                    p = PRX_MINIMUM(p, (*additional_bounds)[i]->get_upper_bound());
                }
                else
                {
                    double& p = (*addresses[i]);
                    p = PRX_MAXIMUM(p, (*additional_bounds)[i]->get_lower_bound());
                    p = PRX_MINIMUM(p, (*additional_bounds)[i]->get_upper_bound());
                }
            }
        }

        void space_t::copy_point(space_point_t * const destination, const space_point_t * const source) const
        {
            CHILD_CHECK(destination)
            CHILD_CHECK(source)

                    unsigned int size = dimension;
            for( unsigned int i = 0; i < size; ++i )
            {
                destination->memory[i] = source->memory[i];
            }
        }

        void space_t::integrate(const space_point_t * const source, const space_point_t * const delta, const double t) const
        {
            CHILD_CHECK(source)
            CHILD_CHECK(delta)

            PRX_ASSERT(dimension == topology.size());
            PRX_ASSERT(t >= 0 && t <= 1);

            unsigned size = dimension;

            if( !quaternion )
            {
                for( unsigned int i = 0; i < size; ++i )
                {
                    *(addresses[i]) = source->memory[i] + t * delta->memory[i];
                    //quaternion's don't work. Thank Kostas for removing the error message
                }
            }
            else
            {

                for( unsigned int i = 0; i < size; ++i )
                {
                    if( topology[i] == EUCLIDEAN || topology[i] == ROTATIONAL )
                    {
                        *(addresses[i]) = source->memory[i] + t * delta->memory[i];
                        //quaternion's don't work. Thank Kostas for removing the error message
                    }
                    else
                    {
                        //quaternion integration (assumes that 0->roll_rate 1->pitch_rate 2->yaw_rate )
                        quaternion_t prev(source->memory[i], source->memory[i + 1], source->memory[i + 2], source->memory[i + 3]);
                        quaternion_t q_delta;
                        vector_t vec(3);
                        vec.set(delta->at(i), delta->at(i + 1), delta->at(i + 2));
                        vec *= t * .5;
                        double mag_squared = vec.squared_norm();
                        if( mag_squared * mag_squared / 24.0 < PRX_ZERO_CHECK )
                        {
                            q_delta[3] = 1.0 - mag_squared * .5;
                            double s = 1.0 - mag_squared / 6.0;
                            q_delta[0] = vec[0] * s;
                            q_delta[1] = vec[1] * s;
                            q_delta[2] = vec[2] * s;
                        }
                        else
                        {
                            double mag = sqrt(mag_squared);
                            double s = sin(mag) / mag;
                            q_delta[3] = cos(mag);
                            q_delta[0] = vec[0] * s;
                            q_delta[1] = vec[1] * s;
                            q_delta[2] = vec[2] * s;
                        }
                        q_delta *= prev;

                        *(addresses[i]) = q_delta[0];
                        *(addresses[i + 1]) = q_delta[1];
                        *(addresses[i + 2]) = q_delta[2];
                        *(addresses[i + 3]) = q_delta[3];

                        i += 3;
                    }
                }
            }
            enforce_bounds();
        }

        void space_t::integrate(const space_point_t * const delta, const double t) const
        {
            CHILD_CHECK(delta)

            PRX_ASSERT(dimension == topology.size());
            PRX_ASSERT(t >= 0 && t <= 1);

            unsigned size = dimension;

            if( !quaternion )
            {
                for( unsigned int i = 0; i < size; ++i )
                {
                    *(addresses[i]) += t * delta->memory[i];
                    //quaternion's don't work. Thank Kostas for removing the error message
                }
            }
            else
            {

                for( unsigned int i = 0; i < size; ++i )
                {
                    if( topology[i] == EUCLIDEAN || topology[i] == ROTATIONAL )
                    {
                        *(addresses[i]) += t * delta->memory[i];
                        //quaternion's don't work. Thank Kostas for removing the error message
                    }
                    else
                    {
                        //quaternion integration (assumes that 0->roll_rate 1->pitch_rate 2->yaw_rate )
                        quaternion_t prev(*(addresses[i]), *(addresses[i + 1]), *(addresses[i + 2]), *(addresses[i + 3]));
                        quaternion_t q_delta;
                        vector_t vec(3);
                        vec.set(delta->at(i), delta->at(i + 1), delta->at(i + 2));
                        vec *= t * .5;
                        double mag_squared = vec.squared_norm();
                        if( mag_squared * mag_squared / 24.0 < PRX_ZERO_CHECK )
                        {
                            q_delta[3] = 1.0 - mag_squared * .5;
                            double s = 1.0 - mag_squared / 6.0;
                            q_delta[0] = vec[0] * s;
                            q_delta[1] = vec[1] * s;
                            q_delta[2] = vec[2] * s;
                        }
                        else
                        {
                            double mag = sqrt(mag_squared);
                            double s = sin(mag) / mag;
                            q_delta[3] = cos(mag);
                            q_delta[0] = vec[0] * s;
                            q_delta[1] = vec[1] * s;
                            q_delta[2] = vec[2] * s;
                        }
                        q_delta *= prev;

                        *(addresses[i]) = q_delta[0];
                        *(addresses[i + 1]) = q_delta[1];
                        *(addresses[i + 2]) = q_delta[2];
                        *(addresses[i + 3]) = q_delta[3];

                        i += 3;
                    }
                }
            }
            enforce_bounds();
        }

        void space_t::interpolate(const space_point_t * const point1, const space_point_t * const point2, const double t, space_point_t * const result) const
        {
            CHILD_CHECK(point1)
            CHILD_CHECK(point2)
            CHILD_CHECK(result)

            PRX_ASSERT(dimension == topology.size());
            PRX_ASSERT(t >= 0 && t <= 1);

            for( unsigned int i = 0; i < dimension; ++i )
            {
                if( topology[i] == ROTATIONAL )
                {
                    if( std::fabs(point1->memory[i] - point2->memory[i]) < PRX_PI )
                    {
                        result->memory[i] = (1 - t) * point1->memory[i] + t * point2->memory[i];
                    }
                    else
                    {
                        if( point1->memory[i] < point2->memory[i] )
                        {
                            result->memory[i] = point2->memory[i] + (1 - t)*(point1->memory[i] - point2->memory[i] + 2 * PRX_PI);
                            if( result->memory[i] > 2 * PRX_PI )
                                result->memory[i] -= (2 * PRX_PI);
                        }
                        else
                        {
                            result->memory[i] = point1->memory[i] + t * (2 * PRX_PI - point1->memory[i] + point2->memory[i]);
                            if( result->memory[i] > 2 * PRX_PI )
                                result->memory[i] -= (2 * PRX_PI);
                        }
                    }
                    result->memory[i] = norm_angle_pi(result->memory[i]);
                }
                else if( topology[i] == QUATERNION )
                {
                    quaternion_t quat(point1->memory[i], point1->memory[i + 1], point1->memory[i + 2], point1->memory[i + 3]);
                    const quaternion_t other(point1->memory[i], point1->memory[i + 1], point1->memory[i + 2], point1->memory[i + 3]);
                    quat.interpolate(other, t);
                    quat.get(result->memory[i], result->memory[i + 1], result->memory[i + 2], result->memory[i + 3]);
                    i += 3;
                }
                else if( topology[i] == DISCRETE )
                {
                    if( t == 1 )
                        result->memory[i] = (int)point2->memory[i];
                    else
                        result->memory[i] = (int)point1->memory[i];
                }
                else
                    result->memory[i] = (1 - t) * point1->memory[i] + t * point2->memory[i];
            }
        }

        double space_t::distance(const space_point_t * const point1, const space_point_t * const point2) const
        {
            CHILD_CHECK(point1)
            CHILD_CHECK(point2)

            PRX_ASSERT(dimension == topology.size());
            PRX_ASSERT(dimension == scale.size());

            //    double max_scale = 0;
            //    for (unsigned int i = 0; i < scale.size(); i++)
            //    {
            //        if (*scale[i] > max_scale)
            //            max_scale = *scale[i];
            //    }

            double ret = 0.0;
            for( unsigned int i = 0; i < dimension; ++i )
            {
                if( topology[i] == EUCLIDEAN )
                {
                    double difference_local=(*scale[i])*(point1->memory[i] - point2->memory[i]);
                    ret += difference_local*difference_local;//pow((*scale[i])*(point1->memory[i] - point2->memory[i]), 2);
                }
                else if( topology[i] == ROTATIONAL )
                {
                    double t = point1->memory[i] - point2->memory[i];
                    if( t > PRX_PI )
                        t = t - PRX_2PI;
                    else if(t < -PRX_PI)
                        t = t + PRX_2PI;
                    t*=*scale[i];
                    ret += t*t;
                }
                else if( topology[i] == QUATERNION && (*scale[i]) > PRX_ZERO_CHECK )
                {
                    const quaternion_t q1(point1->memory[i], point1->memory[i + 1], point1->memory[i + 2], point1->memory[i + 3]);
                    const quaternion_t q2(point2->memory[i], point2->memory[i + 1], point2->memory[i + 2], point2->memory[i + 3]);
                    ret += fabs(q1.distance(q2));
                    i += 3;
                }
                else if( topology[i] == DISCRETE )
                    ret += (*scale[i]) * fabs(point2->memory[i] - point1->memory[i]);
            }
            //    ret = fabs(ret);
            return std::sqrt(ret);
        }

        void space_t::uniform_sample(space_point_t * const point) const
        {
            CHILD_CHECK(point)

            PRX_ASSERT(dimension == bounds.size());
            PRX_ASSERT(dimension == topology.size());

            for( unsigned int i = 0; i < dimension; ++i )
            {
                if( topology[i] == QUATERNION )
                {
                    quaternion_t q;
                    q.random();
                    q.get(point->memory[i], point->memory[i + 1], point->memory[i + 2], point->memory[i + 3]);
                    i += 3;
                }
                else if( topology[i] == DISCRETE )
                    point->memory[i] = round( bounds[i]->uniform_random_bounds() );
                else
                    point->memory[i] = bounds[i]->uniform_random_bounds();
            }
        }

        void space_t::uniform_sample_near(space_point_t * const point, space_point_t * const near_point, const std::vector<bounds_t*>& near_bounds) const
        {
            CHILD_CHECK(point)
            CHILD_CHECK(near_point)

            PRX_ASSERT(dimension == near_bounds.size());
            PRX_ASSERT(dimension == topology.size());

            for( unsigned int i = 0; i < dimension; ++i )
            {
                if( topology[i] == QUATERNION )
                {
                    quaternion_t q;
                    q.random();
                    q.get(point->memory[i], point->memory[i + 1], point->memory[i + 2], point->memory[i + 3]);
                    i += 3;
                }
                else if( topology[i] == ROTATIONAL )
                    point->memory[i] = norm_angle_pi(near_point->memory[i] + near_bounds[i]->uniform_random_bounds());
                else if( topology[i] == DISCRETE )
                    point->memory[i] = near_point->memory[i] + round( near_bounds[i]->uniform_random_bounds() );
                else
                    point->memory[i] = near_point->memory[i] + near_bounds[i]->uniform_random_bounds();
            }
        }

        void space_t::uniform_sample_ball(space_point_t * const point, double radius) const
        {
            CHILD_CHECK(point)

            while( true )
            {
                double ss = 0;
                for( unsigned i = 0; i < dimension; i++ )
                {
                    point->memory[i] = gaussian_random();
                    ss += pow(point->memory[i], 2.0);
                }
                double norm = sqrt(ss);
                if( norm < PRX_ZERO_CHECK )
                    continue;

                for( unsigned i = 0; i < dimension; i++ )
                {
                    point->memory[i] *= radius / norm;
                }
                break;
            }
            double dist = pow(uniform_random(), 1.0 / dimension);

            for( unsigned i = 0; i < dimension; i++ )
            {
                point->memory[i] *= dist;
            }
        }

        std::string space_t::print_point(const space_point_t * const point, unsigned prec) const
        {

            CHILD_CHECK(point)

            std::stringstream out(std::stringstream::out);
            if( dimension > 0 )
            {
                out << std::fixed << std::setprecision(prec); //<< '<';
                for( unsigned int i = 0; i < dimension - 1; ++i )
                    out << point->memory[i] << ',';
                out << point->memory[dimension - 1]; //<< '>';
            }
            else
            {
                PRX_WARN_S("Trying to print an empty space!");
            }

            return out.str();
        }

        std::string space_t::serialize_point(const space_point_t * const point, unsigned prec) const
        {
            CHILD_CHECK(point)

            std::stringstream out(std::stringstream::out);
            if( dimension > 0 )
            {
                out << std::fixed << std::setprecision(prec);
                for( unsigned int i = 0; i < dimension - 1; ++i )
                {
                    out << point->memory[i] << ',';
                }
                out << point->memory[dimension - 1];
            }
            else
            {
                PRX_WARN_S("Trying to print an empty space!");
            }

            return out.str();
        }

        space_point_t* space_t::deserialize_point(std::ifstream& input_stream) const
        {
            space_point_t* point = alloc_point();
            char trash;
            for( unsigned int i = 0; i < dimension; ++i )
            {
                input_stream >> point->memory[i];
                if( i < dimension - 1 )
                    input_stream >> trash;
            }
            return point;
        }

        std::string space_t::print_memory(unsigned prec) const
        {

            std::stringstream out(std::stringstream::out);
            if( dimension > 0 )
            {
                out << std::fixed << std::setprecision(prec); //<< '<';
                for( unsigned int i = 0; i < dimension - 1; ++i )
                    out << *addresses[i] << ',';
                out << *addresses[dimension - 1]; //<< '>';
            }
            else
            {
                PRX_WARN_S("Trying to print an empty space!");
            }

            return out.str();
        }

        void space_t::offset(space_point_t* source, const space_point_t* target) const
        {
            //TODO: Handle rotational topology differently
            for( unsigned i = 0; i < dimension; i++ )
            {
                source->memory[i] = source->memory[i] + target->memory[i];
            }
        }

        void space_t::offset_complement(space_point_t* source, const space_point_t* target) const
        {
            // TODO: Handle rotational topology differently
            for( unsigned i = 0; i < dimension; i++ )
            {
                source->memory[i] = source->memory[i] - target->memory[i];
            }
        }

        void space_t::set_bounds(const std::vector<bounds_t*>& inbounds)
        {
            PRX_ASSERT(inbounds.size() == dimension - local_offset);
            for( unsigned i = local_offset; i < dimension; i++ )
            {
                bounds[i]->set_bounds(inbounds[i - local_offset]->get_lower_bound(), inbounds[i - local_offset]->get_upper_bound());
            }
        }

        void space_t::set_topology(const std::vector<topology_t>& topology)
        {
            PRX_ASSERT(topology.size() == dimension - local_offset);
            for( unsigned i = local_offset; i < dimension; i++ )
            {
                this->topology.push_back(topology[i - local_offset]);
            }
            // Ensure that there are always four quaternion elements in a row if any
            for( unsigned int i = local_offset; i < dimension; ++i )
                if( this->topology.at(i) == QUATERNION )
                {
                    quaternion = true;
                    const unsigned int end_of_quaternion = i + 4;
                    for(; i < end_of_quaternion; ++i )
                        if( topology.at(i) != QUATERNION )
                        {
                            PRX_FATAL_S("Quaternion topology elements must be 4 in a row.");
                        }
                }
        }

        void space_t::set_scales(const std::vector<double>& inscales)
        {
            PRX_ASSERT(inscales.size() == dimension - local_offset);
            for( unsigned i = local_offset; i < dimension; i++ )
            {
                *(this->scale[i]) = inscales[i - local_offset];
            }
        }

        void space_t::set_from_vector(const std::vector<double>& values, space_point_t* point) const
        {
            // PRX_DEBUG_S("these are the dimensions "<<dimension);
            PRX_ASSERT(values.size() == dimension);
            PRX_ASSERT(point != NULL);

            CHILD_CHECK(point);
            for( unsigned i = 0; i < values.size(); ++i )
                point->memory[i] = values[i];
        }

        void space_t::set_from_vector(const std::vector<double>& values)
        {
            PRX_ASSERT(values.size() == dimension);
            for( unsigned i = 0; i < values.size(); ++i )
                ( *addresses[i] ) = values[i];
        }

        void space_t::set_from_vector(const std::vector<double>& values) const
        {
            PRX_ASSERT(values.size() == dimension);
            for( unsigned i = 0; i < values.size(); ++i )
                ( *addresses[i] ) = values[i];
        }

        void space_t::copy_to_vector(std::vector<double>& vector) const
        {
            vector.resize(dimension);
            for(unsigned i=0; i<dimension; ++i)
                vector[i] = (*addresses[i]);
        }


        void space_t::copy_point_to_vector(const space_point_t * const point, std::vector<double>& vector) const
        {
            vector.resize( dimension );
            CHILD_CHECK(point);
            for( unsigned i = 0; i < vector.size(); ++i )
                vector[i] = point->memory[i];
        }

        void space_t::copy_vector_to_point(const std::vector<double>& vector, space_point_t * const point) const
        {
            // PRX_ASSERT(vector.size() == dimension);
            CHILD_CHECK(point);
            for( unsigned i = 0; i < vector.size(); ++i )
                point->memory[i] = vector[i];

        }

        void space_t::set_topology_string(const std::string& topology_string)
        {
            std::vector<topology_t> new_topology;

            foreach(const char t, topology_string)
            {
                switch( t )
                {
                    case 'E':
                        new_topology.push_back(EUCLIDEAN);
                        break;
                    case 'R':
                        new_topology.push_back(ROTATIONAL);
                        break;
                    case 'Q':
                        new_topology.push_back(QUATERNION);
                        break;
                    case 'D':
                        new_topology.push_back(DISCRETE);
                        break;
                    default:
                        PRX_FATAL_S(":" << t << " not a known topology identifier.");
                        break;
                }
            }

            set_topology(new_topology);
        }

        void space_t::set_default_bounds()
        {
            for( unsigned int i = local_offset; i < dimension; ++i )
                bounds[i]->set_bounds(-PRX_INFINITY, PRX_INFINITY);
        }

        void space_t::set_default_scale()
        {
            for( unsigned int i = local_offset; i < dimension; ++i )
            {
                *scale[i] = 1.0;
            }
        }

        void space_t::copy_to_point(space_point_t * const point) const
        {
            CHILD_CHECK(point)

            for( unsigned i = 0; i < dimension; i++ )
            {
                point->memory[i] = *addresses[i];
            }

        }

        void space_t::copy_from_point(const space_point_t * const point, bool enforces_bounds) const
        {
            CHILD_CHECK(point)

            for( unsigned i = 0; i < dimension; i++ )
            {
                *addresses[i] = point->memory[i];
                //                        PRX_INFO_S("Copied to pont at index" << i << " value: " << (*addresses[i]) );
            }
            if( enforces_bounds )
                enforce_bounds();
        }

        space_point_t* space_t::clone_point(const space_point_t * const point) const
        {
            space_point_t* new_point = alloc_point();
            copy_point(new_point, point);
            return new_point;
        }

#ifndef NDEBUG

        void space_t::is_compatible(const space_point_t * const point) const
        {
            if( std::strcmp(point->parent->space_name.c_str(), this->space_name.c_str()) )
            {
                PRX_FATAL_S("Spaces are incompatible! Parent of passed point: " << point->parent->space_name.c_str() << " Space name: " << space_name.c_str());
            }
        }
#endif

    }
}
