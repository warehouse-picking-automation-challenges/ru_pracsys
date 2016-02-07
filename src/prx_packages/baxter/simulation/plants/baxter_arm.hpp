/**
 * @file urdf_plant.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Rahul Shome, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_URDF_PLANT
#define PRX_URDF_PLANT

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"
#include "iostream"
#include "fstream"
#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "../ikfast/ikfastdemo.hpp"
// #include "../../msgs/JointCommand.h"
// #include "../../msgs/JointState.h"
namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace packages
    {
        namespace baxter
        {
            /**
             * A general plant for URDF systems loaded from a file.
             *
             * @brief <b> A general plant for URDF systems.  </b>
             *
             * @author Andrew Dobson
             */
            class baxter_arm_t : public manipulator_plant_t
            {

              public:
                baxter_arm_t();
                virtual ~baxter_arm_t();

                /**
                 * @copydoc plant_t::init()
                 *
                 * Reads in all of the information needed to specify the physical
                 * plant in terms that ODE understands: rigid bodies, joints, and
                 * controls.
                 */
                void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * @copydoc plant_t::update_phys_configs()
                 *
                 * Performs the update according to the actual internal state of ODE.
                 */
                void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

                /**
                 *
                 */
                virtual bool is_grasping() const;

                virtual void get_end_effector_configuration(util::config_t& effector_config);

                virtual bool IK_solver( const util::config_t& effector_config, util::space_point_t* computed_state, bool set_grasping, const util::space_point_t* seed_state = NULL, bool do_min = false );

                virtual bool IK_steering( const util::config_t& start_config, const util::config_t& goal_config, sim::plan_t& result_plan, bool set_grasping = false );

                void append_contingency(sim::plan_t& result_plan, double duration);

                bool is_left_handed();

              protected:

                double* joints_;
                double gripper_;
                double* controls_;
                double gripper_control;

                double wrist_to_end_effector_distance;

                std::vector<std::string> colliding_bodies;

                sim::state_t* prev_st;
                sim::state_t* inter_st;

                double MAX_IK_STEP;

              private:
                kinematic_model_t kinematic_model;

                bool _is_left_handed;
            };
        }
    }
}

#endif

