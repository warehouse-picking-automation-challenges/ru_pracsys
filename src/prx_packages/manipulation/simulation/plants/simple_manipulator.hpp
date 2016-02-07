/**
 * @file simple_manipulator.hpp
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

// #pragma once

// #ifndef PRX_SIMPLE_MANIPULATOR_PLANT_HPP
// #define PRX_SIMPLE_MANIPULATOR_PLANT_HPP

// #include "prx/utilities/definitions/defs.hpp"
// #include "../../../baxter/simulation/plants/manipulator.hpp"

// namespace prx
// {
//     namespace packages
//     {
//         namespace manipulation
//         {

//             using namespace baxter;
//             class simple_manipulator_plant_t : public manipulator_plant_t
//             {

//               public:

//                 simple_manipulator_plant_t();

//                 virtual ~simple_manipulator_plant_t();

//                 virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

//                 void propagate(const double simulation_step);

//                 virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

//                 virtual bool is_grasping() const;

//                 virtual void get_end_effector_position(std::vector<double>& pos);

//                 virtual void get_end_effector_configuration(util::config_t& effector_config);

//                 virtual bool IK_solver(util::config_t& effector_config, std::vector<double>& state_vec, bool do_min = false);

//                 /** @copydoc plant_t::steering_function(const state_t*, const state_t*, plan_t&)*/
//                 void steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result_plan);

//                 /** @copydoc plant_t::append_contingency(plan_t&, double)*/
//                 void append_contingency(sim::plan_t& result_plan, double duration);

//               protected:

//                 /** Indexer for state variable : X */
//                 const static unsigned VAR_X;

//                 /** Indexer for state variable : Y */
//                 const static unsigned VAR_Y;

//                 /** Indexer for state variable : Z */
//                 const static unsigned VAR_Z;

//                 /** Indexer for state variable : THETA X */
//                 const static unsigned VAR_TX;

//                 /** Indexer for state variable : THETA Y */
//                 const static unsigned VAR_TY;

//                 /** Indexer for state variable : THETA Z */
//                 const static unsigned VAR_TZ;

//                 /** Indexer for grasping control variable G */
//                 const static unsigned VAR_G;

//                 /** Internal state & control memory */
//                 double _x;
//                 double _y;
//                 double _z;
//                 double _tx;
//                 double _ty;
//                 double _tz;
//                 double _g;

//                 double _cx;
//                 double _cy;
//                 double _cz;
//                 double _ctx;
//                 double _cty;
//                 double _ctz;
//                 double _cg;

//                 mutable util::config_t end_effector_config;
//                 util::config_t *end_effector_relative_config;
//                 util::config_t *rside_config;
//                 util::config_t *lside_config;
//                 util::config_t *rfinger_config;
//                 util::config_t *lfinger_config;
//                 std::string right_finger_name;
//                 std::string left_finger_name;
//                 std::string right_side_name;
//                 std::string left_side_name;
//                 double effector_distance;
//                 double side_x;
//                 double side_y;
//                 double finger_y;
//                 double side_grasp_y;
//                 double finger_grasp_y;
//                 double max_grasp;
//                 double end_effector_position;
//                 double prev_g;

//                 util::vector_t tmp_pos;
//                 std::vector<double> tmp_state;
//                 mutable util::quaternion_t tmp_orient;
//                 util::config_t tmp_config;

//                 /**
//                  * The maximum distance for the plant in a simulation step.
//                  * @brief The maximum distance for the plant in a simulation step.
//                  */
//                 double max_step;

//                 /**
//                  * The step for a specific distance.
//                  * @brief The step for a specific distance.
//                  */
//                 double interpolation_step;

//                 /**
//                  * The total cover before change control.
//                  * @brief The total cover before change control.
//                  */
//                 double dist;

//                 /**
//                  * A flag for the next step.
//                  * @brief A flag for the next step.
//                  */
//                 bool reset;

//                 /**
//                  * To copy the initial state before the interpolation.
//                  * @brief To copy the initial state before the interpolation.
//                  */
//                 sim::state_t* initial_state;

//                 /**
//                  * To copy the plant's state into to do interpolation and copying and such.
//                  * @brief To copy the plant's state into to do interpolation and copying and such.
//                  */
//                 sim::state_t* state;

//                 /**
//                  * Used to check if the rigid body is actually making progress.
//                  * @brief Used to check if the rigid body is actually making progress.
//                  */
//                 sim::state_t* hold_state;

//                 sim::state_t* interpolated_state;
//                 sim::state_t* prev_state;

//                 /**
//                  * Holds the state from the last propagation.
//                  * @brief Holds the state from the last propagation.
//                  */
//                 sim::state_t* prior_state;

//                 /**
//                  * The previously used control by the system.
//                  * @brief The previously used control by the system.
//                  */
//                 sim::control_t* prior_control;

//                 /**
//                  * To copy the plant's control into in order to do equivalence checking.
//                  * @brief To copy the plant's control into in order to do equivalence checking.
//                  */
//                 sim::control_t* control;

//             };
//         }
//     }
// }

// #endif
