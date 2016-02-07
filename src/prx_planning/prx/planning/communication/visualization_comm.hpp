/**
 * @file prm.cpp
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

#ifndef PRX_PLANNING_VISUALIZATION_COMM_HPP
#define	PRX_PLANNING_VISUALIZATION_COMM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/communication/plan_base_communication.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/simulation/state.hpp"

namespace prx
{
    namespace util
    {
        class tf_broadcaster_t;
    }
    
    namespace plan
    {
        using namespace prx_utilities;        

        namespace comm
        {

            /**
             * @anchor visualization_comm_t
             *
             * This communication class is responsible for interfacing the
             * planning node with the visualization node, from the planning
             * node's side.
             *
             * @brief <b> Planning node communication class for talking to visualization. </b>
             *
             * @author Andrew Kimmel
             */
            class visualization_comm_t : public plan_base_communication_t
            {

              public:
                visualization_comm_t();
                
                typedef boost::function<std::vector<util::config_t> (const sim::state_t*, const std::vector<std::string>&) > state_to_config_t;

                
                void link_application(planning_application_t* parent_app);
                
                /**
                 * Sends all geometries to be visualized to the visualization node.
                 * 
                 * @brief Sends all geometries to be visualized to the visualization node.
                 */
                void send_geometries();

                //            void request_send_solution ( state_to_config_t to_config, const std::string& planner, const sim::trajectory_t& solution, const std::string& name, const std::string& color = "white");
                //            void request_send_structure ( state_to_config_t to_config, const std::string& planner, const std::vector<sim::trajectory_t>& structures, const std::string& name,  const std::string& color);
                //
                //        private:
                //            util::hash_t<std::string, std::vector<util::config_t> > configs_map; // Goes from planner name to the configurations of the planner's structure
                //            util::hash_t<std::string, std::vector<util::geometry_info_t> > geom_map; // Goes from planner name to the geometry infos of the planner's structure
                //            
                //            int numStructures;
                //            void append_position(state_to_config_t to_config, const sim::state_t * const state, const std::string& space_name, const std::string& name, std::vector<double>& info);

                /**
                 * Extract the position part of a configuration.
                 * 
                 * @brief Extract the position part of a configuration.
                 * 
                 * @param config The configuration to convert.
                 */
                std::vector<double> config2params(const util::config_t& config) const;

                /**
                 * @brief Compute configurations from state information.
                 *
                 * Given an input state, this function requests from the planning
                 * application to update a list of configurations.
                 *
                 * @param state The state to convert to configurations.
                 * @param systems_name 
                 * @param params The returned configuration information.
                 */
                void compute_configs(sim::state_t* state, std::vector<std::string> systems_name, util::hash_t<std::string, std::vector<double> >& params) const;

                /**
                 * @brief Send text string to visualize.
                 *
                 * Sends a string to the visualization to display.  The text will be
                 * anchored to a given system, and the font can be customized.
                 *
                 * @param anchored_system The system to which to attach the text string.
                 * @param text_name An identifier for this particular text.
                 * @param text The actual text to display on visualization.
                 * @param relative_position The position of the text relative to the anchored system.
                 * @param color The display color of the font.
                 * @param font_size The size of the font to display.
                 * @param font Which font to use to display the string.
                 */
                bool visualize_scene_text(const std::string& anchored_system, const std::string& text_name, const std::string& text, const std::vector<double>& relative_position, const std::vector<double>& color, double font_size, const std::string& font = "");

                
                virtual bool visualize_ghost_plant(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& plant_configs, double duration = 5, const std::string& destination_node = "visualization");
                virtual bool take_screenshot(unsigned screen_num, int number_of_screenshots, double duration = 5, const std::string& destination_node = "visualization");

                
                /** @brief State to config converter for the visualization comm class. */
                state_to_config_t to_config;
                /** @brief Map of names to geometry info objects. */
                util::hash_t<std::string, std::vector<util::geometry_info_t> > visualization_geom_map;
                /** @brief Map of names to configuration objects. */
                util::hash_t<std::string, std::vector<util::config_t> > visualization_configs_map;

              private:
                /** @brief Publisher which sends out text information. */
                ros::Publisher text_publisher;
            };

            /** @brief The global visualization communication class pointer. */
            extern plan_base_communication_t* vis_comm;

            /** @brief A global pointer for the transform broadcaster. */
            extern util::tf_broadcaster_t* tf_broadcaster;
        }

    }
}

#endif

