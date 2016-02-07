/**
 * @file random.cpp 
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


#include "prx/utilities/definitions/random.hpp"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <numeric>
//#include <boost/random/discrete_distribution.hpp>

namespace prx
{
    namespace util
    {


        /**
         * Global random number generator.
         */
        boost::mt19937 gen;

        void init_random(int seed)
        {
            //    srand(55555);
            srand(seed);
            //    if( seed == -1 )
            //    {
            //	seed= 10;	
            //    }
            //    gen.seed( seed );
        }

        double uniform_random()
        {
            //    const boost::uniform_01<> distribution;
            //    boost::variate_generator<boost::mt19937&, boost::uniform_01<> >
            //	var(gen, distribution);
            //    
            //    return var();

            double val = rand() / RAND_MAX;
            //    PRX_WARN_S("Value: "<<val);
            return val;

        }

        double uniform_random(double min, double max)
        {
            //    const boost::uniform_real<> distribution(min, max);
            //    boost::variate_generator<boost::mt19937&, boost::uniform_real<> >
            //	var(gen, distribution);

            //    return var();
            double val = (((double)rand() / (double)RAND_MAX) * (max - min)) + min;
            //    PRX_WARN_S("Min: "<<min<<" Max: "<<max<<" Value: "<<val);
            return val;
        }

        int uniform_int_random(int min, int max)
        {
            //    const boost::uniform_int<> distribution(min, max);
            //    boost::variate_generator<boost::mt19937&, boost::uniform_int<> >
            //	var(gen, distribution);

            //    return var();
            int val = (rand() % (max + 1 - min)) + min;
            // PRX_WARN_S("Min: " << min << " Max: " << max << " Value: " << val);
            return val;
        }

        int roll_weighted_die(std::vector<double> weights, bool use_boost_random)
        {
            int event_index = -1;
            //   if (use_boost_random)
            //   {
            //       boost::random::discrete_distribution<> dist(weights);
            //       event_index =  dist(gen);
            //   }
            //   else
            //   {

            double sum = 0;
            for( unsigned i = 0; i < weights.size(); i++ )
            {
                sum += weights[i];
            }
            double val = uniform_random(0, 1);
            double running_total = 0;
            for( unsigned i = 0; i < weights.size(); i++ )
            {
                running_total += weights[i] / sum;
                if( val <= running_total )
                {
                    event_index = i;
                    break;
                }
            }
            //   }
            PRX_ASSERT(event_index != -1);
            return event_index;
        }

        double gaussian_random()
        {
            static double t = 0.0;
            double x,v1,v2,r;
            if (t == 0) 
            {
                do 
                {
                  v1 = 2.0 * uniform_random() - 1.0;
                  v2 = 2.0 * uniform_random() - 1.0;
                  r = v1 * v1 + v2 * v2;
                } 
                while (r >= 1.0);
                r = sqrt((-2.0*log(r))/r);
                t = v2*r;
                return (v1*r);  
            }
            else 
            {
                x = t;
                t = 0.0;
                return (x);
            }
        }

    }
}
