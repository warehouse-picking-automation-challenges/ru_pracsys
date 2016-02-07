/**
 * @file vector.cpp 
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

#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/definitions/random.hpp"

#include <boost/numeric/ublas/io.hpp>
#include <boost/foreach.hpp>
#include <iostream>

namespace prx 
{ 
    namespace util 
    {
        
        /**
         *
         */
        vector_t::vector_t( unsigned int dim )
        {
            _vec.resize( dim );
            for(unsigned int i=0; i<dim; i++)
            {
                _vec[i]=0;
            }
        }
        
        /**
         *
         */
        vector_t::vector_t(double x, double y)
        {
            _vec.resize( 2 );
            _vec[0] = x;
            _vec[1] = y;
        }
        
        /**
         *
         */
        vector_t::vector_t(double x, double y, double z)
        {
            _vec.resize( 3 );
            _vec[0] = x;
            _vec[1] = y;
            _vec[2] = z;
        }
        
        /**
         *
         */
        vector_t::vector_t(double x, double y, double z, double w)
        {
            _vec.resize( 4 );
            _vec[0] = x;
            _vec[1] = y;
            _vec[2] = z;
            _vec[3] = w;
        }
        
        /**
         *
         */
        vector_t::vector_t( unsigned int dim, const double* vals )
        {
            _vec.resize( dim );
            for( unsigned int i=0; i<dim; ++i )
                _vec[i] = vals[i];
        }
        
        /**
         *
         */
        vector_t::vector_t( const vector_t & other )
        {
            _vec.resize( other.get_dim() );
            _vec = other._vec;
        }
        
        void vector_t::clear()
        {
            _vec.clear();
            _vec.resize(0);
        }
        
        /**
         *
         */
        vector_t::~vector_t()
        {
        }
        
        /**
         *
         */
        unsigned int vector_t::get_dim() const
        {
            return _vec.size();
        }
        
        /**
         *
         */
        double vector_t::at(int index) const
        {
            PRX_ASSERT( (int)_vec.size() > index );
            return _vec(index);
        }
        
        /**
         *
         */
        void vector_t::get( double* v ) const
        {
            for( unsigned int i=0; i<_vec.size(); ++i )
                v[i] = _vec[i];
        }
        
        /**
         *
         */
        void vector_t::get( double* x, double* y, double* z) const
        {
            PRX_ASSERT(_vec.size() == 3);
            *x = _vec[0];
            *y = _vec[1];
            *z = _vec[2];
        }
        
        void vector_t::get(double& x, double& y, double& z) const
        {
            PRX_ASSERT(_vec.size() == 3);
            x = _vec[0];
            y = _vec[1];
            z = _vec[2];
        }
        
        void vector_t::get(std::vector<double>& params) const
        {
            for( unsigned int i=0; i<_vec.size(); ++i )
                params.push_back(_vec[i]);
        }
        
        void vector_t::set(const std::vector<double>& params) 
        {
            _vec.resize(params.size());
            for( unsigned int i=0; i<_vec.size(); ++i )
                _vec[i] = params[i];
        }
        
        /**
         *
         */
        void vector_t::set( const double* v )
        {
            for( unsigned int i=0; i<_vec.size(); ++i )
                _vec[i] = v[i];    
        }
        
        /**
         *
         */
        void vector_t::set( const double x, const double y, const double z)
        {
            PRX_ASSERT(_vec.size() == 3);
            _vec[0] = x;
            _vec[1] = y;
            _vec[2] = z;
        }
        
        void vector_t::set_at(int index, double val)
        {
            PRX_ASSERT( (int)_vec.size() > index );
            _vec[index] = val;
        }
        
        
        /**
         *
         */
        vector_t& vector_t::operator= ( const vector_t& vec )
        {
            if( *this == vec )
                return *this;
            
            copy( vec );
            return *this;
        }
        
        /**
         *
         */
        bool vector_t::operator== ( const vector_t & vec ) const
        {
            if( _vec.size() != vec._vec.size() )
                return false;
            for( unsigned int i=0; i<_vec.size(); i++ )
                if( _vec[i] != vec._vec[i] )
                    return false;
            return true;
        }
        
        /**
         *
         */
        bool vector_t::operator!= ( const vector_t & vec ) const
        {
            if( _vec.size() != vec._vec.size() )
                return true;
            bool ret = false;
            for( unsigned int i=0; i<_vec.size() && ret == false; i++ )
                if( _vec[i] != vec._vec[i] )
                    ret = true;
            return ret;
        }
        
        /**
         *
         */
        bool vector_t::operator<= ( const vector_t & vec ) const
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            for( unsigned int i=0; i<_vec.size(); ++i )
                if( _vec[i] > vec._vec[i] )
                    return false;
            return true;
        }
        
        /**
         *
         */
        bool vector_t::operator<  ( const vector_t & vec ) const
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            for( unsigned int i=0; i<_vec.size(); ++i )
                if( _vec[i] >= vec._vec[i] )
                    return false;
            return true;
        }
        
        /**
         *
         */
        bool vector_t::operator>= ( const vector_t & vec ) const
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            for( unsigned int i=0; i<_vec.size(); ++i )
                if( _vec[i] < vec._vec[i] )
                    return false;
            return true;
        }
        
        /**
         *
         */
        bool vector_t::operator>  ( const vector_t & vec ) const
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            for( unsigned int i=0; i<_vec.size(); ++i )
                if( _vec[i] <= vec._vec[i] )
                    return false;
            return true;
        }
        
        ///**
        // *
        // */
        //const double& vector_t::operator[] ( unsigned int index ) const
        //{
        //    PRX_ASSERT( _vec.size() > index );
        //    return _vec(index);
        //}
        //
        ///**
        // *
        // */
        //double& vector_t::operator[] ( unsigned int index )
        //{
        //    PRX_ASSERT( _vec.size() > index );
        //    return _vec(index);
        //}
        
        /**
         *
         */
        vector_t& vector_t::operator+= ( const vector_t & vec )
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            _vec += vec._vec;
            return *this;
        }
        
        /**
         *
         */
        vector_t& vector_t::operator-= ( const vector_t & vec )
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            _vec -= vec._vec;
            return *this;
        }
        
        /**
         *
         */
        vector_t& vector_t::operator*= ( const vector_t & vec )
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            for( unsigned int i=0; i<_vec.size(); i++ )
                _vec[i] *= vec._vec[i];
            return *this;
        }
        
        /**
         *
         */
        vector_t& vector_t::operator*= ( double val )
        {
            _vec *= val;
            return *this;
        }
        
        /**
         *
         */
        vector_t& vector_t::operator/= ( const vector_t& vec )
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            for( unsigned int i=0; i<_vec.size(); i++ )
                _vec[i] /= vec._vec[i];
            return *this;
        }
        
        /**
         *
         */
        vector_t& vector_t::operator/= ( double val )
        {
            _vec /= val;
            return *this;
        }
        
        /**
         *
         */
        vector_t vector_t::operator+ ( const vector_t& vec ) const
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            vector_t ret = *this;
            ret._vec += vec._vec;
            return ret;
        }
        
        /**
         *
         */
        vector_t vector_t::operator- ( const vector_t& vec ) const
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            vector_t ret( _vec.size() );
            ret._vec = _vec - vec._vec;
            return ret;
        }
        
        /**
         *
         */
        vector_t vector_t::operator-()
        {
            vector_t ret( _vec.size() );
            for( unsigned int i=0; i<_vec.size(); ++i )
                ret._vec[i] = -1*_vec[i];
            return ret;
        }
        
        /**
         *
         */
        vector_t vector_t::operator* ( const vector_t & vec ) const
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            vector_t ret( _vec.size() );
            for( unsigned int i=0; i<_vec.size(); ++ i)
                ret._vec[i] = _vec[i] * vec._vec[i];
            return ret;
        }
        
        /**
         *
         */
        vector_t vector_t::operator* ( double val ) const
        {
            vector_t ret( _vec.size() );
            ret._vec = _vec * val;
            return ret;
        }
        
        /**
         *
         */
        vector_t vector_t::operator/ ( const vector_t & vec ) const
        {
            PRX_ASSERT( _vec.size() == vec._vec.size() );
            vector_t ret( _vec.size() );
            for( unsigned int i=0; i<_vec.size(); ++ i)
                ret._vec[i] = _vec[i] / vec._vec[i];
            return ret;
        }
        
        /**
         *
         */
        vector_t vector_t::operator/ ( double val ) const
        {
            vector_t ret( _vec.size() );
            ret._vec = _vec / val;
            return ret;
        }
        
        
        /**
         *
         */
        void vector_t::zero()
        {
            for( unsigned int i=0; i<_vec.size(); i++ )
                _vec[i] = 0;
        }
        
        /**
         *
         */
        bool vector_t::is_zero() const
        {
            for( unsigned int i=0; i<_vec.size(); i++ )
                if( _vec[i] != 0 )
                    return false;
            return true;
        }
        
        /**
         *
         */
        void vector_t::copy(const vector_t& source)
        {
            // It's a common use case to create an object on the stack (uninitialized)
            // then assign to it. For this, we allow assigning vectors or different
            // lengths.
            const size_t& o_size = source._vec.size();
            if( _vec.size() != o_size )
            {
                _vec.resize( o_size );
            }
            _vec = source._vec;
        }
        
        /**
         * Seeds the values of this vector with a random point on the unit circle.
         *
         */
        void vector_t::random()
        {    
            for( unsigned int i=0; i<_vec.size(); i++ )
                _vec[i] = uniform_random();
            
            double magn = norm();
            for( unsigned int i=0; i<_vec.size(); i++ )
                _vec[i] /= magn;
        }
        
        /**
         *
         */
        void vector_t::random( const vector_t& alpha, const vector_t& beta )
        {
            PRX_ASSERT( _vec.size() == alpha._vec.size() && _vec.size() == beta._vec.size() );
            for( unsigned int i=0; i<_vec.size(); i++ )
                _vec[i] = uniform_random( alpha._vec[i], beta._vec[i] );
        }
        
        /**
         *
         */
        void vector_t::resize( unsigned int s )
        {
            _vec.resize( s );
        }
        
        /**
         *
         */
        bool vector_t::is_approximate_zero() const
        {
            for( unsigned int i=0; i<_vec.size(); i++ )
                if( fabs( _vec[i] ) > PRX_ZERO_CHECK ) 
                    return false;
            return true;
        }
        
        /**
         *
         */
        bool vector_t::is_approximate_equal( const vector_t& other ) const
        {
            for( unsigned int i=0; i<_vec.size(); i++ )
                if( fabs( _vec[i] - other._vec[i] ) > PRX_ZERO_CHECK )
                    return false;
            return true;
        }
        
        /**
         *
         */
        void vector_t::normalize()
        {
            double n = norm();
            if( n != 0 )
            {
                for( unsigned int i=0; i<_vec.size(); i++ )
                {
                    _vec[i] = _vec[i]/n;
                }
            }
        }
        
        /**
         *
         */
        double vector_t::dot_product( const vector_t& other ) const
        {
            PRX_ASSERT( _vec.size() == other._vec.size() );
            double dotp = 0.0;
            for( unsigned int i=0; i<_vec.size(); i++ )
                dotp += _vec[i] * other._vec[i];
            return dotp;
        }
        
        /**
         *
         */
        void vector_t::cross_product( const vector_t& alpha, const vector_t& beta )
        {
            PRX_ASSERT( _vec.size() == 3 && alpha._vec.size() == 3 && beta._vec.size() == 3 );
            
            _vec[0] = alpha._vec[1] * beta._vec[2] - alpha._vec[2] * beta._vec[1];
            _vec[1] = alpha._vec[2] * beta._vec[0] - alpha._vec[0] * beta._vec[2];
            _vec[2] = alpha._vec[0] * beta._vec[1] - alpha._vec[1] * beta._vec[0];
        }
        
        /**
         *
         */
        bool vector_t::greater_than(double val)
        {
            for( unsigned int i=0; i<_vec.size(); i++ )
                if(_vec[i] <= val)
                    return false;
            return true;
        }
        
        /**
         *
         */
        bool vector_t::fabs_greater_than(double val)
        {
            for( unsigned int i=0; i<_vec.size(); ++i )
                if(fabs(_vec[i]) <= val)
                    return false;
            return true;
        }
        
        /**
         *
         */
        bool vector_t::less_equal_than(double val)
        {
            for( unsigned int i=0; i<_vec.size(); ++i )
                if(_vec[i] > val)
                    return false;
            return true;
        }
        
        /**
         *
         */
        bool vector_t::fabs_less_equal_than(double val)
        {
            for( unsigned int i=0; i<_vec.size(); ++i )
                if(fabs(_vec[i]) > val)
                    return false;
            return true;
        }
        
        /**
         *
         */
        double vector_t::norm() const
        {
            return sqrt( squared_norm() );
        }
        
        /**
         *
         */
        double vector_t::squared_norm() const
        {
            double nrm = 0.0;
            for( unsigned int i=0; i<_vec.size(); i++ )
                nrm += (_vec[i] * _vec[i]);
            return nrm;
        }
        
        /**
         *
         */
        double vector_t::distance( const vector_t& beta ) const
        {
            return sqrt( squared_distance( beta ) );
        }
        
        /**
         *
         */
        double vector_t::squared_distance( const vector_t& beta ) const
        {
            double dist = 0.0, val;
            for( unsigned int i=0; i<_vec.size(); ++i )
            {
                val = beta._vec[i] - _vec[i];
                dist += val*val;
            }
            return dist;
        }
        
        /**
         *
         */
        void vector_t::interpolate( const vector_t& source, const vector_t& target, double t )
        {
            PRX_ASSERT( _vec.size() == source._vec.size() && _vec.size() == target._vec.size() );
            for( unsigned int i=0; i<_vec.size(); i++ )
                _vec[i] = (1.0 - t) * source._vec[i] + t * target._vec[i];
        }
        
        /**
         *
         */
        void vector_t::interpolate( const vector_t& target, double t )
        {
            PRX_ASSERT( _vec.size() == target._vec.size() );
            for( unsigned int i=0; i<_vec.size(); ++i )
                _vec[i] = (1.0 - t) * _vec[i] + t * target._vec[i];
        }
        
        
        double vector_t::get_angle_between(const vector_t& target ) const
        {
            double dot_prod = this->dot_product(target);
            double m1 = this->norm();
            double m2 = target.norm();
            double product = dot_prod/(m1*m2);
            if (product > 1)
                return 0;
            else if (product < -1)
                return PRX_PI;
            
            return acos(product);
        }
        
        /**
         *
         */
        std::istream& operator>>( std::istream& input, vector_t& v )
        {
            std::vector<double> elements;
            
            while( input )
            {
                double r;
                input >> r;
                if (input)
                    elements.push_back(r);
            }
            
            // Check if we ran out of characters or failed to parse a number.
            input.clear();
            
            v.resize(elements.size());
            for(unsigned int i = 0; i < elements.size(); ++i)
            {
                v[i] = elements[i];
            }
            
            return input;
        }
        
        /**
         *
         */
        std::ostream& operator<<( std::ostream& output, const vector_t& v )
        {
            for( unsigned int i = 0; i < v.get_dim()-1; ++i )
            {
                output << v[i] << ' ';
            }
            output << v[v.get_dim()-1];
            
            return output;
        }
        
    } 
}
