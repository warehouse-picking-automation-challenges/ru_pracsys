
/**
 * @file manipulator.hpp
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

#ifndef PRX_MANIPULATOR_PLANT_HPP
#define PRX_MANIPULATOR_PLANT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/kinematic_plant.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"

namespace prx
{
    #define GRIPPER_CLOSED 1
    #define GRIPPER_OPEN 0

    namespace packages
    {
        namespace baxter
        {

            class manipulator_plant_t : public sim::kinematic_plant_t
            {

              public:

                manipulator_plant_t() : sim::kinematic_plant_t(){ }

                virtual ~manipulator_plant_t()
                {
                    state_space->free_point(tmp_state);
                }

                void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
                {
                    sim::kinematic_plant_t::init(reader,template_reader);
                    tmp_state = state_space->alloc_point();
                }

                virtual std::string get_end_effector_name() const
                {
                    return effector_name;
                }


                virtual std::vector<std::string> get_ignored_grasping_bodies()
                {
                    return grasped_ignored;
                }

                virtual bool is_grasping() const = 0;

                virtual void get_end_effector_configuration(util::config_t& effector_config) = 0;

                virtual void get_camera_configuration( util::config_t& effector_config )
                {
                    PRX_FATAL_S("Get Camera Configuration function is NOT implemented.");
                    return;
                }

                virtual bool get_utility_IK(const util::config_t& effector_config, util::space_point_t* computed_state, bool set_grasping, const util::space_point_t* seed_state = NULL, bool do_min = false )
                {
                    PRX_FATAL_S("Utility IK unimplemented");
                    return false;
                }

                virtual bool IK_solver( const util::config_t& effector_config, util::space_point_t* computed_state, bool set_grasping, const util::space_point_t* seed_state = NULL, bool do_min = false ) = 0;

                virtual bool IK_steering( const util::config_t& start_config, const util::config_t& goal_config, sim::plan_t& result_plan, bool set_grasping = false )
                {
                    PRX_ERROR_S("IK_steering is not Implemented.");
                    return false;
                }

                virtual bool IK_steering_general( const util::config_t& start_config, const util::config_t& goal_config, sim::plan_t& result_plan, bool set_grasping = false, bool camera_link = false )
                {
                    PRX_ERROR_S("IK_steering_general is not Implemented.");
                    return false;
                }

              protected:
                std::string effector_name;
                std::vector<std::string> grasped_ignored;
                sim::state_t* tmp_state;
            };
        }
    }
}
#endif
