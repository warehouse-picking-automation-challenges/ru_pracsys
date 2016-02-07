/**
 * @file manual.hpp
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

#ifndef PRX_MANUAL_HPP
#define	PRX_MANUAL_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Its a simple controller for controlling systems manually. The user is 
         * controlling the system with input from the keyboard. 
         * 
         * @brief <b> A simple controller for controlling systems manually. </b>
         * 
         * @author  Athanasios Krontiris, Andrew Kimmel
         * 
         */
        class manual_controller_t : public simple_controller_t
        {

          public:
            manual_controller_t();
            ~manual_controller_t();

            /** 
             * @copydoc controller_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*)
             * 
             * @note Manual controller also needs to initialize the mapping for the input keys to 
             * the different controls. An example how you can set up this input files follows:
             * 
             * @code
             * input_key: UP
             * control_index: 1
             * value: 1.5708
             * action: replace
             * @endcode
             * 
             * The current version of the manual controller supports the keys up, down, left, right, page_up, page_down
             * and the actions add, mul, replace.
             */
            void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** 
             * @copydoc controller_t::compute_control()
             * 
             * @note The computed control will be based on the input from the keyboard.
             */
            void compute_control();

            /** controller_t::verify() const*/
            void verify() const;

          protected:

            /**
             * A struct that maintains all the information that the controller needs
             * in order to compute the controls correct. 
             */
            struct manual_control_info_t
            {

                enum action_t
                {

                    PRX_ADD, PRX_MUL, PRX_REPLACE
                };

                unsigned control_index;
                double value;
                action_t action;

                manual_control_info_t()
                {
                    control_index = -1;
                    value = 0;
                    action = PRX_ADD;
                }

                manual_control_info_t(unsigned index, double val, std::string act)
                {
                    set_values(index, val, act);
                }

                ~manual_control_info_t(){ }

                void set_values(unsigned index, double val, std::string act)
                {
                    control_index = index;
                    value = val;

                    if( act == "mul" || act == "MUL" )
                        action = PRX_MUL;
                    if( act == "replace" || act == "REPLACE" )
                        action = PRX_REPLACE;
                    else
                        action = PRX_ADD;
                }

                double get_new_control(double val)
                {
                    //                      PRX_ERROR_S("Action is : " << action << " val : " << val << "   value:  " << value);
                    switch( action )
                    {
                        case PRX_ADD:
                            return val + value;
                        case PRX_MUL:
                            return val*value;
                        case PRX_REPLACE:
                            return value;
                    }
                    return val;
                }

                void print()
                {
                    PRX_DEBUG_S("control_index:" << control_index << "  value:" << value << "  action:" << action);
                }
            };

            util::hash_t<int, bool> input_keys;
            util::hash_t<int, unsigned> index_map;
            util::hash_t<int, manual_control_info_t> actions;

            int correct_input_key(const std::string& key);
            void handle_keys();


        };

    }
}

#endif

