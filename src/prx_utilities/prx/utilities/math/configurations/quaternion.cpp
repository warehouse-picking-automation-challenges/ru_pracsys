/**
 * @file quaternion.cpp
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

#include "prx/utilities/math/configurations/quaternion.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/vector.hpp"

namespace prx
{
    namespace util
    {

        /**
         * Constructor for the Quaternion class.  Creates a vector of size
         * 4.
         *
         * @brief Constructor.  Creates a new vector of size 4.
         */
        quaternion_t::quaternion_t()
        {
            q[0] = 0.0;
            q[1] = 0.0;
            q[2] = 0.0;
            q[3] = 1.0;
        }

        /**
         * Constructor for the Quaternion class.  Creates a vector of size
         * 4.
         *
         * @brief Constructor.  Creates a new vector of size 4.
         */
        quaternion_t::quaternion_t(double x, double y, double z, double w)
        {
            q[0] = x;
            q[1] = y;
            q[2] = z;
            q[3] = w;
        }


        /**
         * Constructor for the Quaternion class from euler angles
         * 4.
         *
         * @brief Constructor.  Creates a new vector of size 4.
         */
        quaternion_t::quaternion_t(double roll, double pitch, double yaw)
        {
            roll*=.5;
            pitch*=.5;
            yaw*=.5;
            double sinroll = sin(roll);
            double sinpitch = sin(pitch);
            double sinyaw = sin(yaw);
            double cosroll = cos(roll);
            double cospitch = cos(pitch);
            double cosyaw = cos(yaw);

            q[0] = sinroll*cospitch*cosyaw - cosroll*sinpitch*sinyaw;
            q[1] = cosroll*sinpitch*cosyaw + sinroll*cospitch*sinyaw;
            q[2] = cosroll*cospitch*sinyaw - sinroll*sinpitch*cosyaw;
            q[3] = cosroll*cospitch*cosyaw + sinroll*sinpitch*sinyaw;
        }

        quaternion_t::quaternion_t( const quaternion_t& other)
        {
            PRX_ASSERT(this != NULL);
            PRX_ASSERT(&other != NULL);

            q = other.q;
        }

        quaternion_t::quaternion_t( const vector_t* v)
        {
            PRX_ASSERT(v->get_dim() == 4);
            q[0] = v->at(0);
            q[1] = v->at(1);
            q[2] = v->at(2);
            q[3] = v->at(3);
        }

        quaternion_t::quaternion_t( const vector_t& v1, const vector_t& v2 )
        {
            compute_rotation(v1,v2);
        }

        quaternion_t::quaternion_t(const vector_t& axis, double angle)
        {
            set(axis,angle);
        }

        void quaternion_t::clear()
        {
            q[0]=0.0;
            q[1]=0.0;
            q[2]=0.0;
            q[3]=1.0;
        }

        vector_t quaternion_t::get_the_vector()
        {
            return vector_t(q[0],q[1],q[2]);
        }

        void quaternion_t::set_from_euler( double roll, double pitch, double yaw )
        {
            roll*=.5;
            pitch*=.5;
            yaw*=.5;
            double sinroll = sin(roll);
            double sinpitch = sin(pitch);
            double sinyaw = sin(yaw);
            double cosroll = cos(roll);
            double cospitch = cos(pitch);
            double cosyaw = cos(yaw);

            q[0] = sinroll*cospitch*cosyaw - cosroll*sinpitch*sinyaw;
            q[1] = cosroll*sinpitch*cosyaw + sinroll*cospitch*sinyaw;
            q[2] = cosroll*cospitch*sinyaw - sinroll*sinpitch*cosyaw;
            q[3] = cosroll*cospitch*cosyaw + sinroll*sinpitch*sinyaw;
        }

        void quaternion_t::set( double x, double y, double z, double w )
        {
            q[0] = x;
            q[1] = y;
            q[2] = z;
            q[3] = w;
        }


        void quaternion_t::set_xyzw( const double* quat )
        {
            q[0] = quat[0];
            q[1] = quat[1];
            q[2] = quat[2];
            q[3] = quat[3];
        }


        void quaternion_t::set_wxyz( const double* quat )
        {
            q[0] = quat[1];
            q[1] = quat[2];
            q[2] = quat[3];
            q[3] = quat[0];
        }


        void quaternion_t::set( const vector_t& v)
        {
            PRX_ASSERT(v.get_dim() == 4);
            q[0] = v[0];
            q[1] = v[1];
            q[2] = v[2];
            q[3] = v[3];
        }

        void quaternion_t::set( const vector_t& axis, double angle)
        {
            PRX_ASSERT(axis.get_dim() == 3);
            if (fabs(angle) < PRX_ZERO_CHECK)
            {
                clear();
                return;
            }

            angle *= 0.5;
            double sinAngle = sin(angle);

            q[0] = axis[0] * sinAngle;
            q[1] = axis[1] * sinAngle;
            q[2] = axis[2] * sinAngle;
            q[3] = cos(angle);
        }

        double& quaternion_t::operator[](const unsigned int index)
        {
            PRX_ASSERT(index < 4);
            return q[index];
        }

        double quaternion_t::operator[](const unsigned int index) const
        {
            PRX_ASSERT(index < 4);
            return q[index];
        }

        void quaternion_t::get( double& x, double& y, double& z, double& w ) const
        {
            x = q[0];
            y = q[1];
            z = q[2];
            w = q[3];
        }


        void quaternion_t::get_xyzw( double* quat ) const
        {
            quat[0] = q[0];
            quat[1] = q[1];
            quat[2] = q[2];
            quat[3] = q[3];
        }

        void quaternion_t::get_wxyz( double* quat ) const
        {
            quat[0] = q[3];
            quat[1] = q[0];
            quat[2] = q[1];
            quat[3] = q[2];
        }

        void quaternion_t::get( vector_t& v) const
        {
            PRX_ASSERT(v.get_dim() == 4);
            v[0] = q[0];
            v[1] = q[1];
            v[2] = q[2];
            v[3] = q[3];
        }

        double quaternion_t::get_x() const
        {
            return q[0];
        }

        double quaternion_t::get_y() const
        {
            return q[1];
        }

        double quaternion_t::get_z() const
        {
            return q[2];
        }

        double quaternion_t::get_w() const
        {
            return q[3];
        }

        quaternion_t& quaternion_t::operator= ( const quaternion_t& quat)
        {
            q = quat.q;
            return *this;
        }

        bool quaternion_t::operator== ( const quaternion_t& quat) const
        {
            bool ret = false;
            if( q == quat.q )
                ret = true;
            if( q[0] == -quat.q[0] &&
               q[1] == -quat.q[1] &&
               q[2] == -quat.q[2] &&
               q[3] == -quat.q[3] )
                ret = true;
            return ret;
        }

        bool quaternion_t::operator!= ( const quaternion_t& quat) const
        {
            bool ret = true;
            if( q != quat.q)
                ret = false;
            if( q[0] == -quat.q[0] &&
               q[1] == -quat.q[1] &&
               q[2] == -quat.q[2] &&
               q[3] == -quat.q[3] )
                ret = false;
            return ret;
        }

        quaternion_t quaternion_t::operator+ ( const quaternion_t& quat) const
        {
            return quaternion_t( q[0]+quat.q[0], q[1]+quat.q[1], q[2]+quat.q[2], q[3]+quat.q[3]);
        }

        quaternion_t quaternion_t::operator- ( const quaternion_t& quat) const
        {
            return quaternion_t( q[0]-quat.q[0], q[1]-quat.q[1], q[2]-quat.q[2], q[3]-quat.q[3]);
        }

        quaternion_t quaternion_t::operator-() //Negation
        {
            return quaternion_t(-q[0], -q[1], -q[2], -q[3]);
        }

        quaternion_t quaternion_t::operator* ( const quaternion_t& quat) const
        {
            return quaternion_t(
                                q[3]*quat.q[0] + q[0]*quat.q[3] + q[1]*quat.q[2] - q[2]*quat.q[1],
                                q[3]*quat.q[1] + q[1]*quat.q[3] + q[2]*quat.q[0] - q[0]*quat.q[2],
                                q[3]*quat.q[2] + q[2]*quat.q[3] + q[0]*quat.q[1] - q[1]*quat.q[0],
                                q[3]*quat.q[3] - q[0]*quat.q[0] - q[1]*quat.q[1] - q[2]*quat.q[2]
                                );
        }

        quaternion_t quaternion_t::operator * ( double val) const
        {
            return quaternion_t( q[0]*val, q[1]*val, q[2]*val, q[3]*val);
        }

        quaternion_t quaternion_t::operator/ ( const quaternion_t& quat) const
        {
            return quaternion_t((*this)*(quat.conj()));
        }

        quaternion_t quaternion_t::conj() const
        {
            return quaternion_t(-q[0], -q[1], -q[2], q[3]);
        }

        quaternion_t quaternion_t::operator/ ( double val) const
        {
            val = 1.0/val;
            return quaternion_t( q[0]*val, q[1]*val, q[2]*val, q[3]*val);
        }

        quaternion_t& quaternion_t::operator+= ( const quaternion_t& quat)
        {
            for (unsigned int i = 0; i < 4; ++i)
                q[i] += quat.q[i];

            return *this;
        }

        quaternion_t& quaternion_t::operator-= ( const quaternion_t& quat)
        {
            for (unsigned int i = 0; i < 4; ++i)
                q[i] -= quat.q[i];

            return *this;
        }

        quaternion_t& quaternion_t::operator*= ( const quaternion_t& quat)
        {
            *this = *this * quat;
            return *this;
        }

        quaternion_t& quaternion_t::operator*= ( double val)
        {
            for (unsigned int i = 0; i < 4; ++i)
                q[i] *= val;

            return *this;
        }

        quaternion_t& quaternion_t::operator/= ( const quaternion_t& quat)
        {
            *this = *this / quat;
            return *this;
        }

        quaternion_t& quaternion_t::operator/= ( double val)
        {
            val = 1.0/val;
            for (unsigned int i = 0; i < 4; ++i)
                q[i] *= val;

            return *this;
        }


        void quaternion_t::zero()
        {
            q[0]=0;
            q[1]=0;
            q[2]=0;
            q[3]=1;
        }

        /**
         * Checks whether the parameters of the quaternion are all zero.
         *
         * @brief Returns a boolean whether or not the quaternion values are all zero
         *
         * @return True if all values are zero, false if at least one is non-zero.
         */
        bool quaternion_t::is_zero() const
        {
            return (q[0] == 0 && q[1] == 0 && q[2] == 0 && (q[3] == 1 || q[3] == -1));
        }


        /**
         * Quaternion copy method.  Copies the values of the source quaternion and
         * saves them into this.
         *
         * @brief Copies the source quaternion values into this quaternion.
         *
         * @param source Source quaternion pointer to copy the values from.
         */
        void quaternion_t::copy( const quaternion_t& source )
        {
            for (unsigned int i = 0; i < 4; ++i)
                q[i] = source.q[i];
        }

        /**
         * Randomly sets the values of the quaternion to random values.
         */
        void quaternion_t::random( )
        {
            double s = uniform_random();
            double roll1 = sqrt(1-s);
            double roll2 = sqrt(s);
            double theta1 = 2*PRX_PI*uniform_random();
            double theta2 = 2*PRX_PI*uniform_random();

            q[0] = sin(theta1) * roll1;
            q[1] = cos(theta1) * roll1;
            q[2] = sin(theta2) * roll2;
            q[3] = cos(theta2) * roll2;
        }

        bool quaternion_t::is_approximate_zero() const
        {
            for( unsigned int i=0; i<3; i++ )
                if( fabs( q[i] ) > PRX_ZERO_CHECK )
                    return false;
            if ( fabs(fabs(q[3]) - 1) > PRX_ZERO_CHECK)
                return false;
            return true;
        }

        bool quaternion_t::is_approximate_equal( const quaternion_t& other ) const
        {
            bool ret = true;
            for( unsigned int i=0; i<4; i++ )
                if( fabs( q[i] - other.q[i] ) > PRX_ZERO_CHECK)
                    ret = false;

            if(ret)
                return ret;

            for( unsigned int i=0; i<4; i++ )
                if( fabs(q[i] + other.q[i]) > PRX_ZERO_CHECK )
                    return false;
            return true;

        }

        bool quaternion_t::is_valid() const
        {
            return fabs(norm()-1.000000) < PRX_ZERO_CHECK;

            //    double theta = 2 * acos( q[3] );
            //    double d = sin(theta/2.0);
            //    if( fabs(d) >= PRX_ZERO_CHECK )
            //    {
            //        double ax = q[0]/d;
            //        double ay = q[1]/d;
            //        double az = q[2]/d;
            //        double norm = ax*ax + ay*ay + az*az;
            //        if( fabs( norm - 1 ) < PRX_ZERO_CHECK )
            //            return true;
            //        else
            //            return false;
            //    }
            //    else
            //    {
            //        if( fabs(q[0] - q[1]) <= PRX_ZERO_CHECK && fabs(q[1] - q[2]) <= PRX_ZERO_CHECK && fabs(q[2]) <= PRX_ZERO_CHECK )
            //            return true;
            //        else
            //            return false;
            //    }
        }

        void quaternion_t::rectify()
        {
            quaternion_t temp_q = *this;

            if( fabs( squared_norm() - 1 ) > PRX_ZERO_CHECK )
                normalize();

            if(is_approximate_equal(temp_q))
                *this = temp_q;


            // double theta = 2.0 * acos( q[3] );
            // double d = sin(theta*.5);
            // if( d == 0 )
            // {
            //     q[0] = q[1] = q[2] = 0;
            //     q[3] = 1;
            // }
            // else
            // {
            //     double ax = q[0]/d;
            //     double ay = q[1]/d;
            //     double az = q[2]/d;
            //     double norm = sqrt( ax*ax + ay*ay + az*az );
            //     if( fabs( norm - 1 ) < PRX_ZERO_CHECK )
            //     {
            //         ax /= norm;
            //         ay /= norm;
            //         az /= norm;
            //         q[0] = sin( theta / 2.0 ) * ax;
            //         q[1] = sin( theta / 2.0 ) * ay;
            //         q[2] = sin( theta / 2.0 ) * az;
            //     }
            //     else
            //     {
            //         q[0] = 0.0;
            //         q[1] = 0.0;
            //         q[2] = 0.0;
            //         q[3] = 1.0;
            //     }

            // }

        }

        double quaternion_t::norm() const
        {
            return sqrt( squared_norm() );
        }

        double quaternion_t::squared_norm() const
        {
            double nrm = 0.0;
            for( unsigned int i=0; i<4; i++ )
                nrm += (q[i] * q[i]);
            return nrm;
        }

        double quaternion_t::modulus()
        {
            return sqrt( norm() );
        }

        void quaternion_t::conjugate()
        {
            q[0] = -q[0];
            q[1] = -q[1];
            q[2] = -q[2];
        }

        void quaternion_t::convert_to_euler(vector_t& u) const
        {
            // This is with the y and z axes switched
            double r11, r21, r31, r32, r33, r12, r13;
            double q00, q11, q22, q33;
            double qx = q[0];
            double qy = q[1];
            double qz = q[2];
            double qw = q[3];
            double tmp;

            q00 = qw * qw;
            q11 = qx * qx;
            q22 = qy * qy;
            q33 = qz * qz;

            r11 = q00 + q11 - q22 - q33;
            r21 = 2.0 * (qx*qy + qw*qz);
            r31 = 2.0 * (qx*qz - qw*qy);
            r32 = 2.0 * (qy*qz + qw*qx);
            r33 = q00 - q11 - q22 + q33;

            tmp = fabs(r31);
            if( tmp > 0.9999999)
            {
                r12 = 2.0 * ( qx*qy - qw*qz);
                r13 = 2.0 * ( qx*qz + qw*qy);

                u[0] = 0.0;
                u[1] = (-(PRX_PI/2.0) * (r31/tmp));
                u[2] = atan2(-r12, -r31*r13);

            }

            u[0] = atan2(r32, r33);
            u[1] = asin (-r31);
            u[2] = atan2(r21, r11);


            //    // This is with the y and z axes switched
            //    double r11, r21, r31, r32, r33, r12, r13;
            //    double q00, q11, q22, q33;
            //    double tmp;
            //
            //    q00 = q[3] * q[3];
            //    q11 = q[0] * q[0];
            //    q22 = q[2] * q[2];
            //    q33 = q[1] * q[1];
            //
            //    r11 = q00 + q11 - q22 - q33;
            //    r21 = 2 * (q[0]*q[2] + q[3]*q[1]);
            //    r31 = 2 * (q[0]*q[1] - q[3]*q[2]);
            //    r32 = 2 * (q[2]*q[1] + q[3]*q[0]);
            //    r33 = q00 - q11 - q22 + q33;
            //
            //    tmp = fabs(r31);
            //    if( tmp > 0.9999999)
            //    {
            //        r12 = 2 * ( q[0]*q[2] - q[3]*q[1]);
            //        r13 = 2 * ( q[0]*q[1] + q[0]*q[2]);
            //
            //        u[0] = 0.0;
            //        u[1] = atan2(-r12, -r13*r13);
            //        u[2] = (-(PRX_PI/2) * (r31/tmp));
            //    }
            //
            //    u[0] = atan2(r32, r33);
            //    u[1] = atan2(r21, r11);
            //    u[2] = asin (-r31);
        }

        void quaternion_t::inverse()
        {
            conjugate();
            *this /= norm();
        }

        void quaternion_t::normalize()
        {
            *this = *this / norm();
        }

        double quaternion_t::distance( const quaternion_t& other ) const
        {
            PRX_ASSERT_MSG(is_valid(),
                           "Invalid quaternion: %f %f %f %f", q[0], q[1], q[2], q[3]);
            PRX_ASSERT_MSG(other.is_valid(),
                           "Invalid quaternion: %f %f %f %f",
                           other.q[0], other.q[1], other.q[2], other.q[3]);

            const vector_t v1(q[0],q[1],q[2],q[3]);
            const vector_t v2(other.q[0], other.q[1], other.q[2], other.q[3]);
            const double lambda = v1.dot_product(v2);

            return 1 - fabs(lambda);
        }

        void quaternion_t::interpolate( const quaternion_t& target, double f )
        {
            // TODO: don't use vector_t in this function.
            vector_t qv = vector_t(q[0],q[1],q[2],q[3]);

            vector_t other(target.q[0], target.q[1], target.q[2], target.q[3]);
            double lambda = qv.dot_product( other );
            double r, s;

            // fix the opposite
            if (lambda < 0.0)
            {
                other = -other;
                lambda *= -1.0;
            }
            // Calculate the interpolation factors
            // -> Linear
            if ( fabs(1-lambda) < 0.001)
            {
                r = 1 - f;
                s = f;
            }
            // -> Spherical linear
            else
            {
                double alpha = acos(lambda);
                double gamma = 1.0 / sin(alpha);
                r = sin((1.0-f)*alpha) * gamma;
                s = sin(f*alpha) * gamma;
            }
            // Set interpolated quaternion
            qv *= r;
            qv += other*s;
            qv.normalize();

            q[0] = qv[0];
            q[1] = qv[1];
            q[2] = qv[2];
            q[3] = qv[3];
        }

        void quaternion_t::interpolate( const quaternion_t& source, const quaternion_t& target, double f )
        {
            *this = source;
            interpolate(target,f);
        }

        vector_t quaternion_t::qv_rotation(const vector_t& v) const
        {
            const vector_t& vector = v;
            const quaternion_t& q_this = *this;

            //DEBUG: are we doing this in the wrong order maybe?
            // return ( (q_this.conj())*(vector*q_this) ).get_the_vector();
            return (q_this*(vector*q_this.conj())).get_the_vector();
        }

        void quaternion_t::compute_rotation(const vector_t& v1, const vector_t& v2)
        {
            PRX_ASSERT(v1.get_dim() == v2.get_dim());

            vector_t v(3);
            double w;
            //    v.cross_product(v1,v2);
            //
            //    if ( fabs(v1.norm()-1) <= PRX_ZERO_CHECK && fabs(v2.norm()-1) <= PRX_ZERO_CHECK ) {
            //        w = 1 + v1.dot_product(v2);
            //    } else {
            //        w = sqrt(v1.squared_norm() * v2.squared_norm()) + v1.dot_product(v2);
            //    }
            //
            //    set(v[0],v[1],v[2],w);

            v.cross_product(v1,v2);
            w = sqrt(v1.squared_norm() * v2.squared_norm()) + v1.dot_product(v2);
            set(v[0],v[1],v[2],w);
            //    PRX_DEBUG_S("q: " << *this);
            normalize();
            //    PRX_DEBUG_S("AFTER q: " << *this);
        }

        std::istream& operator>>( std::istream& input, quaternion_t& quat )
        {
            for (unsigned int i = 0; i < 4; ++i)
                input >> quat.q[i];

            return input;
        }

        std::ostream& operator<<( std::ostream& output, const quaternion_t& quat )
        {
            output << quat.q[0] << ' ' << quat.q[1] << ' ' << quat.q[2] << ' ' << quat.q[3];
            return output;
        }

        quaternion_t operator * ( const quaternion_t &q, const vector_t& v)
        {
            return quaternion_t(
                                q.get_w()*v[0]+q.get_y()*v[2]-q.get_z()*v[1],
                                q.get_w()*v[1]+q.get_z()*v[0]-q.get_x()*v[2],
                                q.get_w()*v[2]+q.get_x()*v[1]-q.get_y()*v[0],
                                -q.get_x()*v[0]-q.get_y()*v[1]-q.get_z()*v[2]
                                );
        }

        quaternion_t operator * ( const vector_t& v, const quaternion_t &q)
        {
            return quaternion_t(
                                q.get_w()*v[0]+q.get_z()*v[1]-q.get_y()*v[2],
                                q.get_w()*v[1]+q.get_x()*v[2]-q.get_z()*v[0],
                                q.get_w()*v[2]+q.get_y()*v[0]-q.get_x()*v[1],
                                -q.get_x()*v[0]-q.get_y()*v[1]-q.get_z()*v[2]
                                );
        }

    }
}



