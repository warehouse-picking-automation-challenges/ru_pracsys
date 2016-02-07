/**
 * @file system_name_validity_checker.hpp 
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

#ifndef PRX_SYSTEM_NAME_VALIDITY_CHECKER_HPP
#define PRX_SYSTEM_NAME_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/ignore_list_validity_checker.hpp"

namespace prx
{
    namespace packages
    {
        namespace baxter
        {
            class manipulator_plant_t;
        }
        
        namespace rearrangement_manipulation
        {            

            /**
             * @anchor system_name_validity_checker_t
             *
             * Validity checker which uses the world model to retrieve the names
             * of the systems which cause collisions.
             *
             * @brief <b> Validity checker which retrieves system names. </b>
             *
             * @author Andrew Dobson
             */
            class system_name_validity_checker_t : public plan::ignore_list_validity_checker_t
            {

              public:

                system_name_validity_checker_t() : plan::ignore_list_validity_checker_t(){ }

                virtual ~system_name_validity_checker_t(){ }

                void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /** */
                const std::vector< std::string >& what_violates(const sim::state_t* point, bool clear = true);
                /** */
                const std::vector< std::string >& what_violates(const sim::trajectory_t& input);

                /** @copydoc ignore_list_validity_checker_t::is_valid() */
                virtual bool is_valid(const sim::state_t* point);

                /** */
                void setup_checker(baxter::manipulator_plant_t* manip, const std::string& object_name);

              protected:
                void add_violation(const std::string& inname);

                std::vector< std::string > violations;
                baxter::manipulator_plant_t* _manipulator;
                std::string manip_name;
                std::string object_name;
                std::string end_effector;
            };
        }
    }
}


#endif