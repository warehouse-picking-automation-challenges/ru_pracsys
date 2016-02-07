/**
 * @file manual.cpp
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

#include "prx/simulation/systems/controllers/manual/manual_controller.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include <boost/range/adaptor/map.hpp> //adaptors

#include <boost/tuple/tuple.hpp> // boost::tie
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::manual_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        manual_controller_t::manual_controller_t()
        {
            active = false;
        }

        manual_controller_t::~manual_controller_t() { }

        void manual_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            std::vector<const parameter_reader_t*> readers = parameters::get_list("input_keys", reader, template_reader);

            foreach(const parameter_reader_t* r, readers)
            {
                if( r->has_attribute("input_key") )
                {
                    int key = correct_input_key(r->get_attribute("input_key"));
                    input_keys[key] = false;
                    actions[key].set_values(r->get_attribute_as<unsigned>("control_index"), r->get_attribute_as<double>("value"), r->get_attribute("action", "ADD"));
                    std::string key_name = "input_keys/" + int_to_str(key);
                    index_map[key] = actions[key].control_index;
                    if( !ros::param::has("prx/"+key_name) )
                        ros::param::set("prx/"+key_name, false);
                }
            }
            controller_t::init(reader, template_reader);
        }

        void manual_controller_t::compute_control()
        {
            if( active )
            {
                handle_keys();
                //        int current_key = 0;

                foreach(int key, input_keys | boost::adaptors::map_keys)
                {
                    if( input_keys[key] )
                    {
                        // Consume key
                        ros::param::set("prx/input_keys/" + int_to_str(key), false);
                        computed_control->at(index_map[key]) = actions[key].get_new_control(computed_control->at(index_map[key]));
                    }
                }
                output_control_space->enforce_bounds(computed_control);
                output_control_space->copy_from_point(computed_control);
                subsystems.begin()->second->compute_control();
            }
        }

        void manual_controller_t::verify() const
        {
            controller_t::verify();

            if( input_keys.size() != actions.size() )
                throw invalid_system_exception("Manual controller " + pathname + " has different number of action than input keys!");
            if( input_keys.size() == 0 )
                throw invalid_system_exception("Manual controller " + pathname + " Do not have any action key!");
        }

        void manual_controller_t::handle_keys()
        {

            foreach(int key, input_keys | boost::adaptors::map_keys)
            {
                ros::param::getCached("prx/input_keys/" + int_to_str(key), input_keys[key]);
            }
        }

        int manual_controller_t::correct_input_key(const std::string& key)
        {

            PRX_DEBUG_COLOR("new key L:  " << key, PRX_TEXT_CYAN);
            if( key == "UP" || key == "up" )
                return PRX_KEY_UP;
            else if( key == "DOWN" || key == "down" )
                return PRX_KEY_DOWN;
            else if( key == "LEFT" || key == "left" )
                return PRX_KEY_LEFT;
            else if( key == "RIGHT" || key == "right" )
                return PRX_KEY_RIGHT;
            else if( key == "PAGE_UP" || key == "page_up" )
                return PRX_KEY_PAGE_UP;
            else if( key == "PAGE_DOWN" || key == "page_down" )
                return PRX_KEY_PAGE_DOWN;
            else if( key == "LOWER_I" || key == "lower_i" )
                return PRX_KEY_LOWER_I;
            else if( key == "LOWER_J" || key == "lower_j" )
                return PRX_KEY_LOWER_J;
            else if( key == "LOWER_K" || key == "k" )
                return PRX_KEY_LOWER_K;
            else if( key == "LOWER_L" || key == "lower_l" )
                return PRX_KEY_LOWER_L;
            else if( key == "LEFT_BRACKET" || key == "left_bracket" )
                return PRX_KEY_LEFT_BRACKET;
            else if( key == "RIGHT_BRACKET" || key == "right_bracket" )
                return PRX_KEY_RIGHT_BRACKET;
            else if( key == "DELETE" || key == "delete" )
                return PRX_KEY_DELETE;
            else if( key == "HOME" || key == "home" )
                return PRX_KEY_HOME;
            else if( key == "UPPER_U" || key == "upper_u" )
                return PRX_KEY_UPPER_U;
            else if( key == "UPPER_I" || key == "upper_i" )
                return PRX_KEY_UPPER_I;
            else if( key == "UPPER_O" || key == "upper_o" )
                return PRX_KEY_UPPER_O;
            else if( key == "UPPER_P" || key == "upper_p" )
                return PRX_KEY_UPPER_P;
            else if( key == "UPPER_CURLY_LEFT" || key == "upper_curly_left" )
                return PRX_KEY_UPPER_CURLY_LEFT;
            else if( key == "UPPER_CURLY_RIGHT" || key == "upper_curly_right" )
                return PRX_KEY_UPPER_CURLY_RIGHT;
            else if( key == "UPPER_H" || key == "upper_h" )
                return PRX_KEY_UPPER_H;
            else if( key == "UPPER_J" || key == "upper_j" )
                return PRX_KEY_UPPER_J;
            else if( key == "UPPER_K" || key == "upper_k" )
                return PRX_KEY_UPPER_K;
            else if( key == "UPPER_L" || key == "upper_l" )
                return PRX_KEY_UPPER_L;
            else if( key == "UPPER_COLON" || key == "upper_colon" )
                return PRX_KEY_UPPER_COLON;
            else if( key == "UPPER_QUOTES" || key == "upper_quotes" )
                return PRX_KEY_UPPER_DBL_QUT;
            else if( key == "UPPER_N" || key == "upper_n" )
                return PRX_KEY_UPPER_N;
            else if( key == "UPPER_M" || key == "upper_m" )
                return PRX_KEY_UPPER_M;
            else if( key == "UPPER_LESS" || key == "upper_less" )
                return PRX_KEY_UPPER_LESS;
            else if( key == "UPPER_GREATER" || key == "upper_greater" )
                return PRX_KEY_UPPER_GRT;
            else if( key == "UPPER_QU_MARK" || key == "upper_qu_mark" )
                return PRX_KEY_UPPER_QU_MARK;

            return (int)key[0];
        }



    }
}


