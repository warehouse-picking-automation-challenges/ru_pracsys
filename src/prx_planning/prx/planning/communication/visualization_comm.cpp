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

#include <boost/range/adaptor/map.hpp>

#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/applications/planning_application.hpp"

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

PLUGINLIB_EXPORT_CLASS( prx::plan::comm::visualization_comm_t, prx::plan::plan_base_communication_t)

namespace prx
{
    using namespace util;    
    using namespace sim;
    namespace plan
    {
        namespace comm
        {
            plan_base_communication_t* vis_comm;
            tf_broadcaster_t* tf_broadcaster;

            visualization_comm_t::visualization_comm_t()
            {
                text_publisher = node.advertise<prx_utilities::scene_text_msg > ("visualization/scene_text", 10);
                planning_app = NULL;
            }
            
            void visualization_comm_t::link_application(planning_application_t* parent_app)
            {

                plan_base_communication_t::link_application(parent_app);

            }

            void visualization_comm_t::send_geometries()
            {
                //need to transform the hash of name to vectors of geom_info's to a single hash of names to geom_infos.
                hash_t<std::string, geometry_info_t> geoms;

                foreach(std::string name, visualization_geom_map | boost::adaptors::map_keys)
                {
                    unsigned size = visualization_geom_map[name].size();
                    for( unsigned i = 0; i < size; i++ )
                    {
                        //            PRX_WARN_S("Geom name: "<<visualization_geom_map[name][i].full_name);

                        geoms[ visualization_geom_map[name][i].full_name ] = visualization_geom_map[name][i];
                        tf_broadcaster->queue_config(visualization_configs_map[name][i], visualization_geom_map[name][i].full_name);
                    }
                }

                tf_broadcaster->broadcast_configs();
                send_extra_geometries(geoms, 5);

            }

            std::vector<double> visualization_comm_t::config2params(const config_t& config) const
            {
                std::vector<double> params;
                config.get_position(params);
                return params;
            }

            void visualization_comm_t::compute_configs(state_t* state, std::vector<std::string> system_names, hash_t<std::string, std::vector<double> >& params) const
            {
                //    PRX_INFO_S(system_names[0]);
                const std::vector<config_t> configs = to_config(state, system_names);
                for( unsigned int i = 0; i < configs.size(); ++i )
                {
                    params[system_names[i]] = config2params(configs[i]);
                }
            }

            bool visualization_comm_t::visualize_scene_text(const std::string& anchored_system, const std::string& text_name, const std::string& text, const std::vector<double>& relative_position, const std::vector<double>& color, double font_size, const std::string& font)
            {
                prx_utilities::scene_text_msg msg;
                msg.anchored_system = anchored_system;
                msg.text = text;
                msg.text_name = text_name;
                msg.relative_position = relative_position;
                msg.color_rgba = color;
                msg.font = font;
                msg.font_size = font_size;

                text_publisher.publish(msg);
                return true;
            }
            
            bool visualization_comm_t::visualize_ghost_plant(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& plant_configs, double duration, const std::string& destination_node)
            {
                
                if (planning_app->is_visualizing())
                {
                    PRX_INFO_S ("Visualizing ghost geometries");
                    std::vector<std::string> simulation_plant_paths;
                    
                    // Map the planning plant paths to the simulation plant paths
                    for (unsigned i = 0; i < plant_paths.size(); i++)
                    {
                        simulation_plant_paths.push_back(planning_app->planning_to_simulator_sys_mapping[plant_paths[i]]);
                    }
                    return communication_t::visualize_ghost_plant(simulation_plant_paths, plant_configs, duration, destination_node);
                }
                else
                {
                    PRX_WARN_S ("Planning is not visualizing!");
                    return false;
                }
                
            }
            
            bool visualization_comm_t::take_screenshot(unsigned screen_num, int number_of_screenshots, double duration, const std::string& destination_node)
            {
                if (planning_app->is_visualizing())
                {
                    return communication_t::take_screenshot(screen_num, number_of_screenshots, duration, destination_node);
                }
                else
                {
                    PRX_WARN_S ("Simulation is not visualizing!");
                    return false;
                }
            }
        }
    }
}
