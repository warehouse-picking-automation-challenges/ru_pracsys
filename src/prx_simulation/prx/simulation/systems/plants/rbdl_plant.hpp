/**
 * @file rbdl_plant.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_RBDL_PLANT_HPP
#define	PRX_RBDL_PLANT_HPP

#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"


#ifdef RBDL_FOUND
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#endif
namespace prx
{
    namespace sim
    {

        /**
         * 
         */
        class rbdl_plant_t : public integration_plant_t
        {

          public:
            rbdl_plant_t();

            virtual ~rbdl_plant_t(){ }

            /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copoydoc plant_t::update_phys_configs(util::config_list_t&, unsigned&) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

          protected:

            /** @copoydoc plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

#ifdef RBDL_FOUND
            std::vector<std::string> simple_rigid_body_names;
            util::hash_t<std::string,RigidBodyDynamics::Math::VectorNd> config_offsets;

            mutable RigidBodyDynamics::Model model;
            RigidBodyDynamics::Math::VectorNd Q;
            RigidBodyDynamics::Math::VectorNd QDot;
            RigidBodyDynamics::Math::VectorNd Tau;
            RigidBodyDynamics::Math::VectorNd QDDot;
#endif
        };

    }
}

#endif
