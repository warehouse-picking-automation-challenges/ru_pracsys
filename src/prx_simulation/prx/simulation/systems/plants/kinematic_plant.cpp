/**
 * @file kinematic_plant.cpp
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

#include "prx/simulation/systems/plants/kinematic_plant.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::kinematic_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        kinematic_plant_t::kinematic_plant_t() : plant_t()
        {
            reset = false;
        }

        void kinematic_plant_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //    PRX_WARN_S("Init for rigid body 3D ");
            plant_t::init(reader, template_reader);
            max_step = parameters::get_attribute_as<double>("max_step", reader, template_reader);

            initial_state = state_space->alloc_point();
            state = state_space->alloc_point();
            prior_state = state_space->alloc_point();

            control = input_control_space->alloc_point();
            prior_control = input_control_space->alloc_point();

            //Ensures all kinematic systems start with a zero control so they don't suddenly jump around
            input_control_space->copy_point( control, state );
            input_control_space->copy_point( prior_control, state );

            dist = 1;
            interpolation_step = 0.1;
        }

        void kinematic_plant_t::propagate(const double simulation_step)
        {
            state_space->copy_to_point(state);
            input_control_space->copy_to_point(control);

            // if( state_space->equal_points(state, prior_state, PRX_DISTANCE_CHECK) )
            //     interpolation_step = 0;

            // if( (!input_control_space->equal_points(control, prior_control, PRX_DISTANCE_CHECK)) || reset )
            // {
            //     input_control_space->copy_point(prior_control, control);

            //     state_space->copy_point(initial_state, state);

                interpolation_step = max_step / state_space->distance(state, control);
                // dist = 0;
                // reset = false;
            // }

            // dist += interpolation_step;

            if( interpolation_step <= 1 )
            {
                // state_space->copy_to_point(prior_state);
                state_space->interpolate(state, control, interpolation_step, state);
                state_space->copy_from_point(state);
            }
            else
            {
                // PRX_ERROR_S("OUTPUT FROM KINEMATIC PROPAGATE: "<<dist);
                state_space->copy_from_point(control);
            }

            //Store previous control and the now current state.
            // input_control_space->copy_point(prior_control, control);
        }

        void kinematic_plant_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
        {
            double time = std::ceil(state_space->distance(start, goal) / max_step);
            time *= simulation::simulation_step;

            //    PRX_INFO_S("Time: "<<time);
            //    PRX_INFO_S("Distance: "<<state_space->distance(start, goal));
            result_plan.copy_onto_back(goal, time);
        }
    }
}

