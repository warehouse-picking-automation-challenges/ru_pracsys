/**
 * @file unigripper_motoman.hpp 
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
 
#ifndef PRX_UNIGRIPPER_MOTOMAN_PLANT_HPP
#define PRX_UNIGRIPPER_MOTOMAN_PLANT_HPP

#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"
#include "../../baxter/simulation/plants/manipulator.hpp"

#include "prx/simulation/systems/plants/rbdl_plant.hpp"

namespace KDL
{
    class Tree;
}
namespace prx
{
    namespace packages
    {
        namespace motoman
        {

            /**
             * 
             */
            class unigripper_motoman_plant_t : public baxter::manipulator_plant_t
            {

              public:
                unigripper_motoman_plant_t();

                virtual ~unigripper_motoman_plant_t(){ }

                /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /** @copydoc plant_t::propagate(const double) */
                void propagate(const double simulation_step);

                /** @copoydoc plant_t::update_phys_configs(util::config_list_t&, unsigned&) const */
                virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

                virtual void steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result_plan);

                virtual bool is_grasping() const;

                virtual void get_end_effector_configuration(util::config_t& effector_config);

                virtual bool IK_solver( const util::config_t& effector_config, util::space_point_t* computed_state, bool set_grasping, const util::space_point_t* seed_state = NULL, bool do_min = false );

                virtual bool IK_steering( const util::config_t& start_config, const util::config_t& goal_config, sim::plan_t& result_plan, bool set_grasping = false );

              protected:

                KDL::Tree* my_tree;
                /** @copoydoc plant_t::update_derivative (state_t* const) */
                virtual void update_derivative(sim::state_t * const result);

                std::vector<std::string> simple_rigid_body_names;
                util::hash_t<std::string,RigidBodyDynamics::Math::VectorNd> config_offsets;
                std::vector<double> root_offset;
                std::vector<unsigned int> body_ids;

                double _grasping;
                double _grasping_control;
                std::string hand;

                mutable RigidBodyDynamics::Model model;
                RigidBodyDynamics::Math::VectorNd Q;
                RigidBodyDynamics::Math::VectorNd QDot;
                sim::state_t* prev_st;
                sim::state_t* inter_st;

                /**
                 * A \ref boost::function wrapper around \ref integration_plant_t::update_derivative.
                 */
                const sim::integrator_t::derivative_function_t derivative_function;

                /**
                 * The integrator to use when \ref integration_plant_t::propagate is called.
                 */
                sim::integrator_t* integrator;
                double MAX_IK_STEP;

            };
        }

    }
}

#endif
