/**
 * @file manipulator_simulator.hpp
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

#ifndef PRACSYS_MANIPULATION_SIMULATOR_HPP
#define PRACSYS_MANIPULATION_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/simulators/null_response_simulator.hpp"
#include "../../baxter/simulation/plants/manipulator.hpp"
#include "simulation/plants/movable_body_plant.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class grasp_sensing_info_t;

            using namespace baxter;

            /**
             * This is a variation of the simulator will response to the manipulation problems.
             *
             * @brief <b> A simulator that responses to the manipulation problems </b>
             *
             * @author Athanasios Krontiris
             *
             */
            class manipulation_simulator_t : public sim::null_response_simulator_t
            {

              public:

                manipulation_simulator_t();
                ~manipulation_simulator_t();

                /** @copydoc simulator_t::init( const util::parameter_reader_t* const) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                virtual void initialize_sensing();

                virtual void push_state(const sim::state_t * const state);

                void compute_relative_config();
                void print_relative_config();

                /**
                 * Will check if there is at least one collision. If the collision is with the
                 * end effector the simulator will ignore this collision.
                 */
                virtual bool in_collision();

                virtual void propagate(const double simulation_step);

                virtual void propagate_and_respond();

                /** @copydoc simulator_t::internal_state_push() */
                virtual bool internal_state_push();

                void state_checking_propagate();

                void update_ground_truth();

                virtual void link_collision_list(sim::collision_list_t* collision_list);

                bool is_grasping_object()
                {
                    bool grasped = false;
                    foreach(movable_body_plant_t* plant, grasped_objects)
                    {
                        grasped = grasped || plant!=NULL;
                    }
                    return grasped;
                }

                bool can_grasp_object();

              protected:

                std::vector<sim::collision_pair_t> collision_pairs;
                sim::vector_collision_list_t* updated_collision_list;

                std::vector<manipulator_plant_t*> manipulators;

                std::vector<std::string> effector_names;
                std::vector<std::vector<std::string> > ignore_during_grasping;

                std::vector<movable_body_plant_t*> grasped_objects;

                std::string grasp_sensor_name;
                grasp_sensing_info_t* grasp_sensing_info;

                util::config_t effector_config;

              private:
                util::config_t tmp_config;
                std::vector< util::config_t > relative_configs;

                bool check_states;
                unsigned check_state_pos;
                std::vector< std::vector<double> > states_to_check;

                bool check_poses;
                unsigned check_poses_pos;
                std::vector< std::vector<double> > poses_to_check;

                std::vector< std::string > grasped_names;                
            };
        }
    }
}

#endif
