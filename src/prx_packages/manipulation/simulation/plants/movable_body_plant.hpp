/**
 * @file movable_body_plant.hpp
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

#ifndef PRX_MOVABLE_BODY_PLANT_HPP
#define	PRX_MOVABLE_BODY_PLANT_HPP

#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/plant.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

            /**
             * Represents a rigid body system. Implementation of the integration functions
             * for simulating a rigid body system.\n
             * State: [x, y, theta] \n
             * Control: [x, y, theta].
             *
             * @brief <b> Represents a rigid body system. </b>
             *
             * @author Andrew Dobson
             *
             */
            class movable_body_plant_t : public sim::plant_t
            {

              public:
                movable_body_plant_t();

                virtual ~movable_body_plant_t(){ }

                /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * Copies the given \c state to the system state. This is the only way to
                 * move movable object that they do not have any control to be
                 * controlled
                 *
                 * @brief Copies the given \c state to the system state.
                 *
                 * @param state The state that we want to set the system.
                 */
                virtual void push_state(const sim::state_t * const state);

                /**
                 * Move the plant based on the end effector configuration as well
                 * as a relative configuration to that end effector.
                 */
                void move_object( const util::config_t& end_effector_config, const util::config_t& relative_config );

                /**
                 * Calculate relative configuration of movable body with respect to effector_config and store result in rel_config
                 * @param effector_config Configuration of end effector of manipulator
                 * @param rel_config  Resultant relative configuration of movable body with respect to effector_config
                 */
                virtual void relative_config_with_manipulator(const util::config_t& end_effector_config, util::config_t& relative_config);

                /** @copydoc plant_t::propagate(const double) */
                void propagate(const double simulation_step);

                /** @copoydoc plant_t::update_phys_configs(util::config_list_t&) const */
                virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

                void steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result_plan);

                /**
                 * Retrieves the movable body's configuration based on its
                 * current state.
                 */
                void get_configuration( util::config_t& body_config ) const;
                void set_configuration( const util::config_t& body_config );
                void print_configuration() const;

              protected:
                /**
                 * Internal state memory for the \c x position coordinate.
                 * @brief Internal state memory for the \c x position coordinate.
                 */
                double _x;

                /**
                 * Internal state memory for the \c y position coordinate.
                 * @brief Internal state memory for the \c y position coordinate.
                 */
                double _y;

                /**
                 * Internal state memory for the \c z position coordinate.
                 * @brief Internal state memory for the \c z position coordinate.
                 */
                double _z;

                /**
                 * Internal state memory for the \c qx coordinate of the orientation quaternion.
                 * @brief Internal state memory for the \c qx coordinate of the orientation quaternion.
                 */
                double _qx;

                /**
                 * Internal state memory for the \c qy coordinate of the orientation quaternion.
                 * @brief Internal state memory for the \c qy coordinate of the orientation quaternion.
                 */
                double _qy;

                /**
                 * Internal state memory for the \c qz coordinate of the orientation quaternion.
                 * @brief Internal state memory for the \c qz  coordinate of the orientation quaternion.
                 */
                double _qz;

                /**
                 * Internal state memory for the \c qw coordinate of the orientation quaternion.
                 * @brief Internal state memory for the \c qw coordinate of the orientation quaternion.
                 */
                double _qw;

              private:

            };
        }
    }
}

#endif
