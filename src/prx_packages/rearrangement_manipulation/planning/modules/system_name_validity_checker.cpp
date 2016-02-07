/**
 * @file system_name_validity_checker.cpp 
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


#include "planning/modules/system_name_validity_checker.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include "../../../baxter/simulation/plants/manipulator.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::system_name_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace baxter;
        namespace rearrangement_manipulation
        {

            void system_name_validity_checker_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                ignore_list = NULL;
                ignore_list_validity_checker_t::init(reader, template_reader);
            }

            const std::vector< std::string >& system_name_validity_checker_t::what_violates(const state_t* point, bool clear)
            {
                if( clear )
                    violations.clear();

                foreach(collision_pair_t body_pair, world_model->get_colliding_bodies(point)->get_body_pairs())
                {
                    std::pair< std::string, std::string > pair = body_pair;
                    if( body_pair.first != end_effector && body_pair.second != end_effector )
                    {
                        if( (ignore_list == NULL || !(ignore_list->pair_in_list(pair.first, pair.second))) )
                        {
                            pair.first = reverse_split_path(pair.first).first;
                            pair.second = reverse_split_path(pair.second).first;

                            //First, just go ahead and just check if the manipulator is part of this pair.                        
                            if( pair.first == manip_name )
                            {
                                PRX_DEBUG_COLOR("Collsion (first == manip_name) between: " << pair.first << " " << pair.second, PRX_TEXT_RED);
                                if( pair.second != object_name )
                                    add_violation(pair.second);
                                else if( !_manipulator->is_grasping() )
                                    add_violation(pair.second);
                                else
                                    PRX_DEBUG_COLOR("Something bad happened here collsion between: " << pair.first << " " << pair.second, PRX_TEXT_RED);
                            }
                            else if( pair.second == manip_name )
                            {
                                PRX_DEBUG_COLOR("Collsion (second == manip_name) between: " << pair.first << " " << pair.second, PRX_TEXT_RED);
                                if( pair.first != object_name )
                                    add_violation(pair.first);
                                else if( !_manipulator->is_grasping() )
                                    add_violation(pair.first);
                                else
                                    PRX_DEBUG_COLOR("Something bad happened here collsion between: " << pair.first << " " << pair.second, PRX_TEXT_RED);
                            }
                                //Otherwise, we need to check if either of the objects is what we are holding
                            else
                            {

                                if( _manipulator->is_grasping() )
                                {
                                    PRX_DEBUG_COLOR("Collsion (manipulator->is_grasping) between: " << pair.first << " " << pair.second, PRX_TEXT_RED);
                                    if( pair.first == object_name )
                                        add_violation(pair.second);
                                    else if( pair.second == object_name )
                                        add_violation(pair.first);
                                    else
                                        PRX_ASSERT_MSG(false, "This cannot happen!");
                                }
                                else
                                {
                                    PRX_DEBUG_COLOR("Something bad happened here collsion between: " << pair.first << " " << pair.second, PRX_TEXT_RED);
                                    PRX_DEBUG_COLOR("state in collision : " << world_model->get_state_space()->serialize_point(point, 4), PRX_TEXT_RED);
                                }
                            }
                        }
                    }
                }
                return violations;
            }

            const std::vector< std::string >& system_name_validity_checker_t::what_violates(const trajectory_t& input)
            {
                violations.clear();

                foreach(state_t* state, input)
                {
                    what_violates(state, false);
                }
                return violations;
            }

            bool system_name_validity_checker_t::is_valid(const state_t* point)
            {
                collision_list_t* cb = world_model->get_colliding_bodies(point);
                std::string obj_name = object_name + "/body";

                foreach(const collision_pair_t& cp, cb->get_body_pairs())
                {
                    if( cp.first != end_effector && cp.second != end_effector )
                    {
                        if( ignore_list == NULL || !(ignore_list->pair_in_list(cp.first, cp.second)) )
                        {
                            if( cp.first == obj_name || cp.second == obj_name )
                            {
                                PRX_DEBUG_COLOR("Collision : " << cp.first << " - " << cp.second, PRX_TEXT_RED);
                                if(cp.first == "simulator/cup2/body" || cp.second == "simulator/cup2/body")
                                {
                                    PRX_DEBUG_COLOR(world_model->get_full_state_space()->print_memory(5), PRX_TEXT_LIGHTGRAY);
                                }
                                
                            }
                            else
                            {
                                PRX_DEBUG_COLOR("Collision : " << cp.first << " - " << cp.second, PRX_TEXT_MAGENTA);
                            }
                            return false;
                        }
                    }
                }
                return true;
            }

            void system_name_validity_checker_t::setup_checker(manipulator_plant_t* manip, const std::string& object_name)
            {
                _manipulator = manip;
                this->manip_name = manip->get_pathname();
                end_effector = manip->get_end_effector_name();
                this->object_name = object_name;
            }

            void system_name_validity_checker_t::add_violation(const std::string& inname)
            {
                if( std::find(violations.begin(), violations.end(), inname) == violations.end() )
                    violations.push_back(inname);
            }
        }
    }
}