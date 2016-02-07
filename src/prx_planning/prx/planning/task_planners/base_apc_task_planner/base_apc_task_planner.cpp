/**
 * @file base_apc_task_planner.cpp
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

#include "prx/planning/task_planners/base_apc_task_planner/base_apc_task_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/motion_planners/apc_planner/apc_planner.hpp"
#include "prx/planning/modules/samplers/apc_sampler.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
#include "prx/planning/motion_planners/sst/sst_statistics.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/planning/applications/single_query_application.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/adaptor/map.hpp>
#include <iostream>
#include <fstream>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::base_apc_task_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        base_apc_task_planner_t::base_apc_task_planner_t()
        {
            left_IK_data_base = NULL;
            right_IK_data_base = NULL;
            left_cam_IK_data_base = NULL;
            right_cam_IK_data_base = NULL;
        }

        void base_apc_task_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            task_planner_t::init(reader, template_reader);


            std::ifstream input;
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/../object_models/");
            if( parameters::has_attribute("shelf_description_file", reader, template_reader) )
            {
                std::string file_name = dir + parameters::get_attribute("shelf_description_file", reader, template_reader);

                FILE * pFile;
                pFile = fopen(file_name.c_str(), "r");
                if( pFile != NULL )
                {
                    shelf_size.resize(3);
                    double trash;
                    int ret = fscanf(pFile, "{'x':%lf, 'y':%lf, 'z':%lf, 'roll':%lf, 'pitch':%lf, 'yaw':%lf, 'width':%lf, 'height':%lf, 'depth':%lf"
                                     , &trash, &trash, &trash, &trash, &trash, &trash
                                     , &shelf_size[1], &shelf_size[2], &shelf_size[0]
                                     );
                    fclose(pFile);
                }
                else
                {
                    PRX_FATAL_S("I could not read the file : " << file_name << "  for the shelf specification in the apc_planner!");
                }
            }
            else
            {
                PRX_FATAL_S("You need to specify shelf_description_file for the base_apc_task_planner_t in the folder: " << dir);
            }
        }

        void base_apc_task_planner_t::restart_planners()
        {
            foreach(std::string& name, planner_names)
                ((apc_planner_t*)(planners[name]))->restart_astar();
        }

        void base_apc_task_planner_t::setup()
        {
            //create specifications for each motion planner

            foreach(std::string& name, planner_names)
            {
                model->use_context(space_names[name]);
                output_specifications[name] = new motion_planning_specification_t();
                motion_planning_specification_t* spec = dynamic_cast<motion_planning_specification_t*>(output_specifications[name]);
                std::string new_name = ros::this_node::getName() + "/" + path + "/" + name;
                PRX_INFO_S("Setup in apc task planner is trying to init a mp_spec for " << new_name);
                parameter_reader_t reader(new_name);
                parameters::initialize(spec, &reader, "", NULL, "");
                spec->link_spaces(model->get_state_space(), model->get_control_space());
                spec->setup(model);

                PRX_PRINT("Local planner in specs: " << spec->local_planner << "  trace:" << reader.trace(), PRX_TEXT_GREEN);
                apc_local_planner_t* apc_local_planner = dynamic_cast<apc_local_planner_t*>(spec->local_planner);
                if( apc_local_planner != NULL )
                {
                    if( name == "left_prm" || name == "left_prm_grasp" )
                        apc_local_planner->link_info(_left_manipulator, min_shelf_bounds, max_shelf_bounds);
                    else
                        apc_local_planner->link_info(_right_manipulator, min_shelf_bounds, max_shelf_bounds);
                }
                else
                {
                    PRX_FATAL_S("APC planner needs an apc_local_planner to run!");
                }

                apc_sampler_t* apc_sampler = dynamic_cast<apc_sampler_t*>(spec->sampler);
                if( apc_sampler != NULL )
                {
                    if( name == "left_prm" || name == "left_prm_grasp" )
                        apc_sampler->link_info(_left_manipulator, min_shelf_bounds, max_shelf_bounds, left_IK_data_base);
                    else
                        apc_sampler->link_info(_right_manipulator, min_shelf_bounds, max_shelf_bounds, right_IK_data_base);
                }
                else
                {
                    PRX_FATAL_S("APC planner needs an apc_sampler to run!");
                }

                planners[name]->link_specification(spec);
            }
            selected_planner = planner_names[0];
        }

        bool base_apc_task_planner_t::execute()
        {
            PRX_PRINT("EXECUTING", PRX_TEXT_MAGENTA);

            foreach(std::string& name, planner_names)
            {
                PRX_PRINT( "Name of space name: " << name, PRX_TEXT_BLUE );
                if( name == "left_prm" || name == "right_prm" )
                {
                    motion_planning_specification_t* spec = dynamic_cast<motion_planning_specification_t*>(output_specifications[name]);
                    spec->get_stopping_criterion()->reset();
                    bool done = false;
                    
                    planners[name]->set_param("", "serialize_flag", true);
                    planners[name]->set_param("", "serialization_file", "roadmaps/" + name + ".roadmap");
                    planners[name]->setup();
                    
                    while( !done )
                    {
                        try
                        {
                            model->use_context(space_names[name]);
                            planners[name]->execute();
                        }
                        catch( stopping_criteria_t::stopping_criteria_satisfied e )
                        {
                            PRX_INFO_S(e.what());
                            done = true;
                        }
                    }
                }
            }

            return true;
        }

        void base_apc_task_planner_t::special_serialize() 
        {
            model->use_context( space_names["left_prm"] );
            ((apc_planner_t*)planners["left_prm"])->validate_precomputation();
            model->use_context( space_names["right_prm"] );
            ((apc_planner_t*)planners["right_prm"])->validate_precomputation();

            ((motion_planner_t*)planners["left_prm"])->serialize();
            ((motion_planner_t*)planners["right_prm"])->serialize();

            PRX_PRINT( "Deserializing LEFT PRM GRASP roadmap from LEFT PRM roadmap", PRX_TEXT_BROWN );
            model->use_context( space_names["left_prm"] );
            special_deserialize("left_prm_grasp" );
            PRX_PRINT( "Deserializing RIGHT PRM GRASP roadmap from RIGHT PRM roadmap", PRX_TEXT_BROWN );
            model->use_context( space_names["right_prm"] );
            special_deserialize("right_prm_grasp" );
        }

        bool base_apc_task_planner_t::deserialize(std::string name)
        {
            PRX_PRINT("Deserializing roadmap from file: roadmaps/" << name << ".roadmap", PRX_TEXT_MAGENTA);

            planners[name]->set_param("", "deserialize_flag", true);
            planners[name]->set_param("", "deserialization_file", "roadmaps/" + name + ".roadmap");
            ((apc_planner_t*)planners[name])->setup_online_mode();
            planners[name]->setup();
            ((motion_planner_t*)planners[name])->deserialize();
            PRX_PRINT("Done Deserialization", PRX_TEXT_MAGENTA);

            return true;
        }

        bool base_apc_task_planner_t::special_deserialize(std::string name )
        {
            std::string new_name;
            if( name == "left_prm_grasp" )
                new_name = "left_prm";
            else
                new_name = "right_prm";

            PRX_PRINT("Deserializing roadmap from file: roadmaps/" << new_name << ".roadmap", PRX_TEXT_MAGENTA);

            planners[name]->set_param("", "deserialize_flag", true);
            planners[name]->set_param("", "deserialization_file", "roadmaps/" + new_name + ".roadmap");
            ((apc_planner_t*)planners[name])->setup_online_mode();
            planners[name]->setup();
            ((motion_planner_t*)planners[name])->deserialize();
            PRX_PRINT("Done Deserialization", PRX_TEXT_MAGENTA);

            ((apc_planner_t*)planners[name])->change_grasping_state();

            return true;
        }

        const statistics_t* base_apc_task_planner_t::get_statistics()
        {
            return planners[selected_planner]->get_statistics();
        }

        bool base_apc_task_planner_t::succeeded() const
        {
            PRX_WARN_S("APC Task Planner succeeded is not implemented yet, returning false");
            return false;
        }

        void base_apc_task_planner_t::link_specification(specification_t* in_specification)
        {
            task_planner_t::link_specification(in_specification);
            specification = dynamic_cast<base_apc_task_specification_t*>(in_specification);
        }

        void base_apc_task_planner_t::link_query(query_t* in_query)
        {
            query = dynamic_cast<base_apc_task_query_t*>(in_query);
            if( query->state_space == NULL )
                query->link_spaces(model->get_state_space(), model->get_control_space());
            // specification->get_stopping_criterion()->link_motion_planner(dynamic_cast<motion_planner_t*>(planners[planner_names[0]]));            
            // specification->get_stopping_criterion()->link_goal(query->get_goal());
            planners[selected_planner]->link_query(query);
        }

        void base_apc_task_planner_t::link_manipulators(prx::packages::baxter::manipulator_plant_t* left_manipulator, prx::packages::baxter::manipulator_plant_t* right_manipulator)
        {
            _left_manipulator = left_manipulator;
            _right_manipulator = right_manipulator;
            foreach(std::string& name, planner_names)
            {
                model->use_context(space_names[name]);
                if( name == "left_prm" || name == "left_prm_grasp" )
                    dynamic_cast<apc_planner_t*>(planners[name])->link_manipulator(left_manipulator, true);
                else
                    dynamic_cast<apc_planner_t*>(planners[name])->link_manipulator(right_manipulator, false);
            } 
        }

        void base_apc_task_planner_t::resolve_query()
        {
            PRX_PRINT( "Using planner:: " << selected_planner, PRX_TEXT_LIGHTGRAY );
            ((apc_planner_t*)planners[selected_planner])->resolve_query();
        }

        void base_apc_task_planner_t::resolve_query( bool grasping )
        {
            //PRX_PRINT( "Selected planner is: " << selected_planner, PRX_TEXT_LIGHTGRAY );
            ((apc_planner_t*)planners[selected_planner])->resolve_query( model->get_active_space(), model, grasping );
        }

        bool base_apc_task_planner_t::IK_resolve_query( config_t end_config, bool grasping, bool camera_link )
        {
            return ((apc_planner_t*)planners[selected_planner])->IK_steer_resolution( model, end_config, grasping, camera_link );
        }

        void base_apc_task_planner_t::generate_IK_data_base(IK_data_base_t& left_arm_IK_base, IK_data_base_t& right_arm_IK_base, IK_data_base_t& left_cam_IK_base, IK_data_base_t& right_cam_IK_base)
        {
            std::string old_context = model->get_current_context();
            foreach(std::string& name, planner_names)
            {
                model->use_context(space_names[name]);
                if( name == "left_prm"  )
                    dynamic_cast<apc_planner_t*>(planners[name])->generate_IK_data_base(left_arm_IK_base, left_cam_IK_base);
                else if( name == "right_prm" )
                    dynamic_cast<apc_planner_t*>(planners[name])->generate_IK_data_base(right_arm_IK_base, right_cam_IK_base);
                PRX_PRINT("Made progress with : " << name, PRX_TEXT_MAGENTA );
            }
            model->use_context(old_context);        
        }

        void base_apc_task_planner_t::link_IK_data_base(IK_data_base_t* left_arm_base, IK_data_base_t* right_arm_base, IK_data_base_t* left_cam_base, IK_data_base_t* right_cam_base)
        {
            left_IK_data_base = left_arm_base;
            right_IK_data_base = right_arm_base;
            left_cam_IK_data_base = left_cam_base;
            right_cam_IK_data_base = right_cam_base;
        }

        void base_apc_task_planner_t::update_vis_info() const {
            //do nothing
        }

        void base_apc_task_planner_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            if( parameter_name == "shelf_position" )
            {
                std::vector<double> shelf_center = boost::any_cast< std::vector<double> >(value);
                min_shelf_bounds.resize(3);
                max_shelf_bounds.resize(3);

                min_shelf_bounds[0] = shelf_center[0] - shelf_size[0] / 2.0;
                min_shelf_bounds[1] = shelf_center[1] - shelf_size[1] / 2.0;
                min_shelf_bounds[2] = shelf_center[2] - shelf_size[2] / 2.0;

                max_shelf_bounds[0] = shelf_center[0] + shelf_size[0] / 2.0;
                max_shelf_bounds[1] = shelf_center[1] + shelf_size[1] / 2.0;
                max_shelf_bounds[2] = shelf_center[2] + shelf_size[2] / 2.0;
            }
        }

        bool base_apc_task_planner_t::is_valid(state_t* state)
        {
            return dynamic_cast<motion_planning_specification_t*>(output_specifications[planner_names[0]])->validity_checker->is_valid(state);
        }

    }
}


