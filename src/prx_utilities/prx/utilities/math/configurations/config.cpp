/**
 * @file config.cpp
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

#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie

namespace prx
{
    namespace util
    {

        config_t::config_t() : position(3), orientation() { }

        config_t::config_t(const config_t& c) : position(c.position), orientation(c.orientation) {
        }

        config_t::config_t(const vector_t& pos, const quaternion_t& orient) : position(pos), orientation(orient) { }

        void config_t::zero()
        {
            position.zero();
            orientation.zero();
        }

        void config_t::copy(const config_t& source)
        {
            position.copy(source.position);
            orientation.copy(source.orientation);
        }

        void config_t::random(const config_t& min_config, const config_t& max_config)
        {
            position.random(min_config.position, max_config.position);
            orientation.random();
        }

        bool config_t::is_zero() const
        {
            return ( position.is_zero() && orientation.is_zero());
        }

        bool config_t::is_approximate_equal(const config_t& conf) const
        {
            return ( position.is_approximate_equal(conf.position) && orientation.is_approximate_equal(conf.orientation));
        }

        bool config_t::is_approximate_equal(const vector_t& pos, const quaternion_t& orient) const
        {
            return (position.is_approximate_equal(pos) && orientation.is_approximate_equal(orient));
        }

        const vector_t& config_t::get_position() const
        {
            return position;
        }

        void config_t::get_position(double* x, double* y, double* z) const
        {
            position.get(x, y, z);
        }

        void config_t::get_position(double& x, double& y, double& z) const
        {
            position.get(x, y, z);
        }

        const quaternion_t& config_t::get_orientation() const
        {
            return orientation;
        }

        void config_t::get(vector_t& pos, quaternion_t& quat) const
        {
            pos = position;
            quat = orientation;
        }

        void config_t::get_position(std::vector<double>& pos) const
        {
            position.get(pos);
        }

        void config_t::get_position(double* pos) const
        {
            position.get(pos);
        }

        void config_t::get_wxyz_orientation(double* quat) const
        {
            orientation.get_wxyz(quat);
        }

        void config_t::get_xyzw_orientation(double* quat) const
        {
            orientation.get_xyzw(quat);
        }

        void config_t::set_position(double x, double y, double z)
        {
            position[0] = x;
            position[1] = y;
            position[2] = z;
        }

        void config_t::set_position(const vector_t& pos)
        {
            PRX_ASSERT(pos.get_dim() == 3);
            position.copy(pos);
        }

        void config_t::set_position(const double* pos)
        {
            position.set(pos);
        }

        void config_t::set_position_at(int index, double val)
        {
            PRX_ASSERT(index > 0 && index < 3);
            position[ index ] = val;
        }

        void config_t::set_orientation(double x, double y, double z, double w)
        {
            orientation.set(x, y, z, w);
        }

        void config_t::set_orientation(const quaternion_t& quat)
        {
            orientation.copy(quat);
        }

        void config_t::set_wxyz_orientation(const double* quat)
        {
            orientation.set_wxyz(quat);
        }

        void config_t::set_wxyz_orientation(double w, double x, double y, double z)
        {
            orientation.set(x, y, z, w);
        }

        void config_t::set_xyzw_orientation(const double* quat)
        {
            orientation.set_xyzw(quat);
        }

        void config_t::set_xyzw_orientation(double x, double y, double z, double w)
        {
            orientation.set(x, y, z, w);
        }

        void config_t::set_orientation(double roll, double pitch, double yaw)
        {
            orientation.set_from_euler(roll,pitch,yaw);
        }

        void config_t::set(const vector_t& pos, const quaternion_t& quat)
        {
            set_position(pos);
            set_orientation(quat);
        }

        void config_t::set_param(const std::string& param_name, const double value)
        {
            std::string component;
            std::string param;
            boost::tie(component, param) = split_path(param_name);

            if( component == "position" )
            {
                if( param == "x" )
                {
                    position[0] = value;
                }
                else if( param == "y" )
                {
                    position[1] = value;
                }
                else if( param == "z" )
                {
                    position[2] = value;
                }
                else
                {
                    throw std::runtime_error("Unknown position coordinate : \"" + param + "\".");
                }
            }
            else if( component == "orientation" )
            {
                if( param == "x" )
                {
                    orientation[0] = value;
                }
                else if( param == "y" )
                {
                    orientation[1] = value;
                }
                else if( param == "z" )
                {
                    orientation[2] = value;
                }
                else if( param == "w" )
                {
                    orientation[3] = value;
                }
                else
                {
                    throw std::runtime_error("Unknown orientation coordinate : \"" + param + "\".");
                }
            }
            else
            {
                throw std::runtime_error("Unknown component of a configuration : \"" + component + "\".");
            }
        }

        void config_t::normalize_orientation()
        {
            orientation.normalize();
        }

        config_t& config_t::operator=(const config_t& conf)
        {
            if( *this == conf )
                return *this;

            copy(conf);
            return *this;
        }

        bool config_t::operator ==(const config_t& conf) const
        {
            if( position == conf.position && orientation == conf.orientation )
                return true;
            return false;
        }

        bool config_t::operator !=(const config_t& conf) const
        {
            if( position != conf.position || orientation != conf.orientation )
                return true;
            return false;
        }

        config_t config_t::operator+(const config_t& conf) const
        {
            //    config_t newConf;
            //    newConf.position = position + conf.position;
            //    newConf.orientation = orientation * conf.orientation;
            return config_t(position + conf.position, orientation * conf.orientation);
        }

        config_t config_t::operator-(const config_t& conf) const
        {
            //    config_t newConf;
            //    newConf.position = position - conf.position;
            //    newConf.orientation = orientation / conf.orientation;
            return config_t(position - conf.position, orientation / conf.orientation);
        }

        config_t& config_t::operator+=(const config_t& conf)
        {
            position += conf.position;
            orientation *= conf.orientation;
            return *this;
        }

        config_t& config_t::operator-=(const config_t& conf)
        {
            position -= conf.position;
            orientation /= conf.orientation;
            return *this;
        }

        void config_t::interpolate(const config_t& target, double t)
        {
            position.interpolate(target.position, t);
            orientation.interpolate(target.orientation, t);
        }

        void config_t::interpolate(const config_t& start, const config_t& target, double t)
        {
            position.interpolate(start.get_position(), target.get_position(), t);
            orientation.interpolate(start.get_orientation(), target.get_orientation(), t);
        }

        void config_t::init(const parameter_reader_t * const reader,const parameter_reader_t * const template_reader)
        {
            position = parameters::get_attribute_as<vector_t>("position",reader,template_reader);
            orientation = parameters::get_attribute_as<quaternion_t>("orientation",reader,template_reader);
            // position = reader->get_attribute_as<vector_t > ("position");
            // orientation = reader->get_attribute_as<quaternion_t > ("orientation");
        }


        std::ostream& operator<<( std::ostream& out, const config_t& config )
        {
            out << config.position << " " << config.orientation;
            return out;
        }

        void augment_config_list(config_list_t& list, unsigned index)
        {
            PRX_ASSERT(list.size()>=index);
            if( list.size() == index )
            {
                list.push_back(config_list_element_t("", config_t()));
            }

        }

        void config_t::copy_to_vector(std::vector<double>& config_vector)
        {
            config_vector.resize(7);
            position.get(config_vector[0],config_vector[1],config_vector[2]);
            orientation.get(config_vector[3],config_vector[4],config_vector[5],config_vector[6]);
        }

        void config_t::copy_from_vector(std::vector<double>& config_vector)
        {
            PRX_ASSERT(config_vector.size()==7);
            position.set(config_vector[0],config_vector[1],config_vector[2]);
            orientation.set(config_vector[3],config_vector[4],config_vector[5],config_vector[6]);
        }

        std::string config_t::print() const
        {
            std::stringstream out(std::stringstream::out);

            out<<"[ "<<position.at(0)<<","<<position.at(1)<<","<<position.at(2)<<"; "<<orientation.get_x()<<","<<orientation.get_y()<<","<<orientation.get_z()<<","<<orientation.get_w()<<" ]";

            return out.str();
        }


        void config_t::relative_to_global(const config_t& root_config)
        {
            set_position(vector_t(root_config.get_orientation().qv_rotation(position)));
            *this = root_config + *this;
        }

        void config_t::global_to_relative( const config_t& root_config )
        {
            //Have an identity temporary orientaiton
            util::quaternion_t tmp_orient;
            tmp_orient.zero();
            //Then, set our position to be the relative position
            this->set_position( this->get_position() - root_config.get_position() );
            tmp_orient /= root_config.get_orientation();
            this->set_position(vector_t(tmp_orient.qv_rotation(this->get_position())));
            tmp_orient *= this->get_orientation();
            tmp_orient.normalize();
            PRX_ASSERT(tmp_orient.is_valid());
            this->set_orientation(tmp_orient);
        }

    }
}
