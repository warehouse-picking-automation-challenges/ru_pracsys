/**
 * @file shortcut_smoother.cpp 
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

#include "prx/planning/modules/path_smoothers/shortcut_smoother.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::shortcut_smoother_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        shortcut_smoother_t::shortcut_smoother_t(local_planner_t* inlp, validity_checker_t* invc, double inres)
        {
            local_planner = inlp;
            validity_checker = invc;
            resolution = inres;
        }

        void shortcut_smoother_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            path_smoother_t::init(reader, template_reader);

            if( local_planner == NULL )
                PRX_FATAL_S("Shortcut Smoother requires a local planner, but none provided!\n");
            if( validity_checker == NULL )
                PRX_FATAL_S("Shortcut Smoother requires a validity checker, but none provided!\n");

            resolution = parameters::get_attribute_as< double >("resolution", reader, template_reader, 1.0);
        }

        void shortcut_smoother_t::step()
        {
            PRX_DEBUG_COLOR("Begin Step", PRX_TEXT_MAGENTA);

            trajectory_t& traj = input_query->path;
            plan_t& plan = input_query->plan;

            unsigned int size = traj.size();

            if( traj.size() <= 2 )
                return;

            PRX_DEBUG_COLOR("Traj: " << traj.size() << "  Plan: " << plan.size(), PRX_TEXT_CYAN);

            unsigned a = 1;
            unsigned b = 1;
            while( b - a < 2 )
            {
                a = uniform_int_random(0, size);
                b = uniform_int_random(0, size);
                if( b < a )
                {
                    unsigned tmp = a;
                    a = b;
                    b = tmp;
                }

            }

            double dist = state_space->distance(traj[a], traj[b]);
            if( dist == 0 )
            {
                PRX_WARN_S("Disparate states are the same.");
            }
            else
            {
                trajectory_t seg = traj;
                seg.clear();
                plan_t p = plan;
                p.clear();

                local_planner->steer(traj[a], traj[b], p, seg);

                //If this path is valid
                if( validity_checker->is_valid(seg) )
                {
                    PRX_DEBUG_COLOR("Valid path found...", PRX_TEXT_LIGHTGRAY);
                    double dur = p[0].duration;

                    PRX_DEBUG_COLOR("Shortcut Duration : " << dur, PRX_TEXT_CYAN);

                    //Setup the temporary holding variables
                    trajectory_t tt = traj;
                    plan_t tp = plan;

                    tt.clear();
                    tp.clear();

                    PRX_DEBUG_COLOR("About to copy plan/trajectory information", PRX_TEXT_LIGHTGRAY);
                    //Constuct the appropriate trajectory
                    for( unsigned k = 0; k <= a; ++k )
                    {
                        tt.copy_onto_back(traj[k]);
                        //                tp.copy_onto_back( plan[k].control, plan[k].duration );
                    }
                    for( unsigned k = b; k < size; ++k )
                    {
                        tt.copy_onto_back(traj[k]);
                    }

                    //The construct the appropriate path.
                    for( unsigned k = 0; k < a; ++k )
                    {
                        tp.copy_onto_back(plan[k].control, plan[k].duration);
                    }
                    tp.copy_onto_back(p[0].control, dur);
                    for( unsigned k = b; k < size - 1; ++k )
                    {
                        tp.copy_onto_back(plan[k].control, plan[k].duration);
                    }

                    PRX_DEBUG_COLOR("Completed generation!", PRX_TEXT_GREEN);
                    input_query->path = tt;
                    input_query->plan = tp;
                    PRX_DEBUG_COLOR("End Step", PRX_TEXT_MAGENTA);
                }
            }
        }

    }
}



