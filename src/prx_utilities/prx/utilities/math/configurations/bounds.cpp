/**
 * @file bounds.cpp
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


#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/definitions/random.hpp"

namespace prx
{
    namespace util
    {

        /**
         *
         */
        bounds_t::bounds_t() : lower(0.0), upper(0.0) { }

        bounds_t::bounds_t(double low, double high) : lower(low), upper(high)
        {
            PRX_ASSERT(low <= high);
        }

        bounds_t::~bounds_t() { }

        void bounds_t::clear()
        {
            lower = 0.0;
            upper = 0.0;
        }

        void bounds_t::set_bounds(double low, double high)
        {
            if( fabs(low-high) < PRX_ZERO_CHECK )
                high = low;
            PRX_ASSERT_MSG(low <= high, "low: %f, high: %f", low, high);

            if( high == 3.14 )
                upper = PRX_PI;
            else if( high == -3.14 )
                upper = -1 * PRX_PI;
            else if( high == 1.57 )
                upper = PRX_PI / 2;
            else if( high == -1.57 )
                upper = -1 * PRX_PI / 2;
            else
                upper = high;

            if( low == 3.14 )
                lower = PRX_PI;
            else if( low == -3.14 )
                lower = -1 * PRX_PI;
            else if( low == 1.57 )
                lower = PRX_PI / 2;
            else if( low == -1.57 )
                lower = -1 * PRX_PI / 2;
            else
                lower = low;

            //    upper = high;
            //    lower = low;
        }

        void bounds_t::set_lower_bound(double val)
        {
            PRX_ASSERT(upper >= val);
            if( val == 3.14 )
                lower = PRX_PI;
            else if( val == -3.14 )
                lower = -1 * PRX_PI;
            else if( val == 1.57 )
                lower = PRX_PI / 2;
            else if( val == -1.57 )
                lower = -1 * PRX_PI / 2;
            else
                lower = val;
        }

        void bounds_t::set_upper_bound(double val)
        {
            PRX_ASSERT(lower <= val);
            if( val == 3.14 )
                upper = PRX_PI;
            else if( val == -3.14 )
                upper = -1 * PRX_PI;
            else if( val == 1.57 )
                upper = PRX_PI / 2;
            else if( val == -1.57 )
                upper = -1 * PRX_PI / 2;
            else
                upper = val;
        }

        /**
         *
         */
        bounds_t& bounds_t::operator=(const bounds_t& other)
        {
            if( this == &other )
                return *this;

            lower = other.lower;
            upper = other.upper;
            return *this;
        }

        bool bounds_t::operator==(const bounds_t& other) const
        {
            if( this == &other )
                return true;

            return lower == other.lower && upper == other.upper;
        }

        bool bounds_t::operator!=(const bounds_t& other) const
        {
            return !(*this == other);
        }

        void bounds_t::get_bounds(double& low, double& high) const
        {
            low = lower;
            high = upper;
        }

        /**
         *
         */
        double bounds_t::get_lower_bound() const
        {
            return lower;
        }

        /**
         *
         */
        double bounds_t::get_upper_bound() const
        {
            return upper;
        }

        double bounds_t::adjust(double value) const
        {
            double l = lower;
            double u = upper;
            if( value < l )
                return l;
            if( value > u )
                return u;
            return value;
        }

        double bounds_t::derivative_adjust(double derivative, double state_val, const bounds_t& state_bounds) const
        {
            derivative = adjust(derivative);

            if( state_val >= state_bounds.upper && derivative > 0 )
                derivative = 0;
            if( state_val <= state_bounds.lower && derivative < 0 )
                derivative = 0;

            return derivative;
        }

        void bounds_t::expand(double value)
        {
            if( value < lower )
                lower = value;
            if( value > upper )
                upper = value;
        }

        bool bounds_t::is_valid(double value) const
        {
            return ( value >= lower && value <= upper);
        }

        void bounds_t::copy(const bounds_t& source)
        {
            lower = source.lower;
            upper = source.upper;
        }

        void bounds_t::intersect(const bounds_t& source)
        {
            lower = PRX_MAXIMUM(lower, source.lower);
            upper = PRX_MINIMUM(upper, source.upper);
        }

        void bounds_t::unite(const bounds_t& source)
        {
            lower = PRX_MINIMUM(lower, source.lower);
            upper = PRX_MAXIMUM(upper, source.upper);
        }

        double bounds_t::uniform_random_bounds() const
        {
            return uniform_random(lower, upper);
        }

        double bounds_t::uniform_random_derivative(double state_val, const bounds_t& state_bounds) const
        {
            double lb = lower;
            double ub = upper;

            if( state_val >= state_bounds.upper && ub > 0 )
                ub = 0.0;
            else if( state_val <= state_bounds.lower && lb < 0 )
                lb = 0.0;

            return uniform_random(lb, ub);
        }

        void bounds_t::verify() const
        {
            if( lower > upper )
                throw std::runtime_error("Lower bound is greater than upper bound.");
        }

        std::ostream& operator<<(std::ostream& output, const bounds_t& b)
        {
            output << " [" << b.lower << ',' << b.upper << ']';
            return output;
        }

        namespace bounds
        {

            void verify(const std::vector<bounds_t*>& b)
            {
                foreach(bounds_t* bound, b)
                {
                    bound->verify();
                }
            }

            void verify(const std::vector<bounds_t>& b)
            {
                foreach(const bounds_t& bound, b)
                {
                    bound.verify();
                }
            }

            void set_bounds(const std::vector<bounds_t*>& b, const std::vector<double>& lower_bounds, const std::vector<double>& upper_bounds)
            {
                PRX_ASSERT(lower_bounds.size() == upper_bounds.size());
                for( unsigned i = 0; i < lower_bounds.size(); i++ )
                {
                    b[i]->set_bounds(lower_bounds[i], upper_bounds[i]);
                }
            }

            void intersect(const std::vector<bounds_t*>& b, const std::vector<bounds_t*>& lower_bounds, const std::vector<bounds_t*>& upper_bounds)
            {
                PRX_ASSERT(lower_bounds.size() == upper_bounds.size());
                for( unsigned i = 0; i < lower_bounds.size(); i++ )
                {
                    (*b[i]) = *lower_bounds[i];
                    b[i]->intersect(*upper_bounds[i]);
                }
            }

            void intersect( std::vector<bounds_t>& b, const std::vector<bounds_t>& lower_bounds, const std::vector<bounds_t>& upper_bounds)
            {
                PRX_ASSERT(lower_bounds.size() == upper_bounds.size());
                for( unsigned i = 0; i < lower_bounds.size(); i++ )
                {
                    b[i] = lower_bounds[i];
                    b[i].intersect(upper_bounds[i]);
                }
            }

        }
    }
}




