/**
 * @file base_apc_planning_application.hpp 
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

#include "prx/planning/applications/planning_application.hpp"
#include "prx/planning/applications/motoman_planning_application.hpp"

#ifndef PRX_GRASP_TEST_APC_APPLICATION_HPP
#define PRX_GRASP_TEST_APC_APPLICATION_HPP

namespace prx
{
    namespace plan
    {
        class grasp_test_planning_application_t : public motoman_planning_application_t
        {
          public:
            grasp_test_planning_application_t();
            virtual ~grasp_test_planning_application_t();

            virtual bool place_object_callback(prx_planning::place_object::Request& req, prx_planning::place_object::Response& res);

            bool test_grasp_for_object( std::string object_to_grasp, sim::state_t*& good_state , util::config_t& ee_config);
            bool test_simple_IK( const util::config_t& ee_config, util::space_point_t* target_state, sim::plan_t& resulting_plan );

          private:
            std::vector< bool > consider_grasp;
            std::vector< unsigned > grasps_to_remove;

        };
    }
}

#endif
