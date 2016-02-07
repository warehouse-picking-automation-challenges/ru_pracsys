/**
 * @file base_apc_task_planner.hpp 
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
#ifndef PRX_BASE_APC_TASK_PLANNER_HPP
#define	PRX_BASE_APC_TASK_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/IK_data_base/IK_data_base.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/task_planners/base_apc_task_planner/base_apc_task_query.hpp"
#include "prx/planning/task_planners/base_apc_task_planner/base_apc_task_specification.hpp"
#include "simulation/plants/manipulator.hpp"

namespace prx
{
    namespace plan
    {

        class base_apc_task_planner_t : public task_planner_t
        {

          public:
                 
            base_apc_task_planner_t();
                       
            /**
             * @copydoc task_planner_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @copydoc task_planner_t::setup()
             */
            virtual void setup();
            void restart_planners();

            /**
             * @copydoc task_planner_t::execute()
             */
            virtual bool execute();

            virtual void special_serialize();

            /**
             * @copydoc task_planner_t::deserialize( )
             */
            virtual bool deserialize( std::string name );
            virtual bool special_deserialize( std::string name );

            /**
             * @copydoc task_planner_t::get_statistics()
             */
            virtual const util::statistics_t* get_statistics();

            /**
             * @copydoc task_planner_t::succeeded()
             */
            virtual bool succeeded() const;
            
            /**
             * @copydoc task_planner_t::link_specification(specification_t*)
             */
            virtual void link_specification(specification_t* in_specification);

            /**
             * @copydoc task_planner_t::link_query(query_t*)
             */
            virtual void link_query(query_t* in_query);
            
            /**
             * Links the two manipulators for the left and right hand.
             */
            virtual void link_manipulators(prx::packages::baxter::manipulator_plant_t* left_manipulator, prx::packages::baxter::manipulator_plant_t* right_manipulator);

            /**
             * @copydoc task_planner_t::resolve_query()
             */
            virtual void resolve_query();

            bool is_valid(sim::state_t* state);

            void change_queried_planner(std::string planner_name)
            {
                selected_planner = planner_name;
            }

            void resolve_query( bool grasping );
            
            bool IK_resolve_query( util::config_t end_config, bool grasping, bool camera_link );
            
            void generate_IK_data_base(IK_data_base_t& left_arm_IK_base, IK_data_base_t& right_arm_IK_base, IK_data_base_t& left_cam_IK_base, IK_data_base_t& right_cam_IK_base);

            void link_IK_data_base(IK_data_base_t* left_arm_base, IK_data_base_t* right_arm_base, IK_data_base_t* left_cam_base, IK_data_base_t* right_cam_base);
          
          protected:

            std::string selected_planner;

            /**
             * @copydoc task_planner_t::update_vis_info()
             */
            virtual void update_vis_info() const;
            
            /**
             * @copydoc planner_t::set_param( const std::string&, const boost::any& )
             */
            virtual void set_param(const std::string& parameter_name, const boost::any& value);

            prx::packages::baxter::manipulator_plant_t* _left_manipulator;
            prx::packages::baxter::manipulator_plant_t* _right_manipulator;

            IK_data_base_t* right_IK_data_base;
            IK_data_base_t* left_IK_data_base;
            IK_data_base_t* right_cam_IK_data_base;
            IK_data_base_t* left_cam_IK_data_base;

            base_apc_task_specification_t* specification;
            base_apc_task_query_t* query;
            
            //Shelf information that will be passed to the motion_planners' modules.
            std::vector<double> shelf_size;
            util::vector_t min_shelf_bounds;
            util::vector_t max_shelf_bounds; 
        };

    }
}

#endif

