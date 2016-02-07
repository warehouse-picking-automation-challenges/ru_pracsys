/**
 * @file osg_visualization.cpp 
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

#include "prx/visualization/PLUGINS/OSG/osg_visualization.hpp"

#include <osg/PositionAttitudeTransform>

#include <cstdlib> // For getenv

#include <osgDB/FileUtils>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pluginlib/class_list_macros.h>

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/visualization/visualization_comm.hpp"


PLUGINLIB_EXPORT_CLASS( prx::vis::osg_visualization_t, prx::vis::visualization_t)

namespace prx
{
    using namespace util;
    namespace vis
    {        

        osg_visualization_t::osg_visualization_t()
        {
            PRX_DEBUG_S("Constructing OSG visualization.");
            initialized = false;

            const std::string model_path = std::getenv("PRACSYS_MODELS_PATH");
            osgDB::setDataFilePathList(model_path);
        }

        osg_visualization_t::~osg_visualization_t()
        {
            delete listener;
            delete comm;
        }

        void osg_visualization_t::init(const parameter_reader_t* reader)
        {
            PRX_DEBUG_S("Initializing OSG visualization.");

            reader->initialize(&viewer, "viewer");

            PRX_DEBUG_S("Initialized OSG visualization.");
            initialized = true;

            std::string plugin_type_name;
            plugin_type_name = reader->get_attribute("vis_comm", "visualization_comm");
            comm = visualization_comm_t::get_loader().createUnmanagedInstance("prx_visualization/" + plugin_type_name);
            comm->link_visualization(this);
            comm->link_scene(this->get_scene());
            listener = new tf_listener_t();
        }

        void update_configurations(const tf_listener_t& listener, osg_scene_t& scene)
        {
            // Working memory.
            config_t config;

            typedef boost::unordered_map< std::string, osg::ref_ptr<osg::PositionAttitudeTransform>, string_hash>::iterator iter_t;

            foreach(std::string geom_name, scene.info_geoms_to_update)
            {
                listener.lookup(geom_name, config);
                scene.move_info_geometry(geom_name, config);
            }
            scene.info_geoms_to_update.clear();
            for( iter_t iter = scene.rigid_bodies.begin(); iter != scene.rigid_bodies.end(); ++iter )
            {
                const std::string& name = iter->first;
                listener.lookup(name, config);
                //        PRX_WARN_S("Listener name: " << name);
                //        config.print();
                scene.move_geometry(name, config);
            }

        }

        bool osg_visualization_t::run()
        {
            PRX_ASSERT(initialized);

            //    bool init = false;
            sys_clock_t timer;
            // Listens for updates to rigid body configurations.
            osg_scene = dynamic_cast<osg_scene_t*>(get_scene());

            while( ros::ok() && !viewer.done() )
            {
                //        ros::getGlobalCallbackQueue()->callAvailable();
                ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
                update_configurations(*listener, *osg_scene);

                for (unsigned i = 0; i <window_screenshot_queue.size(); i++)
                {
                    viewer.take_screenshot(window_screenshot_queue[i].first, window_screenshot_queue[i].second );
                }
                viewer.frame();
                
                window_screenshot_queue.clear();

                //        if(!init || timer.measure() > 5)
                //        {
                //            comm->poll_topics();
                //            timer.reset();
                //            init = true;
                //        }
            }
            return true;
        }
        
        void osg_visualization_t::update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf)
        {
            config_t temp_conf;
            for(unsigned i = 0; i < geom_configs.size(); i++)
            {
                if (poll_tf)
                {
                    listener->lookup(geom_names[i], temp_conf);
                }
                else
                {
                    temp_conf = geom_configs[i];
                }
//                PRX_ERROR_S ("Moving info geom: " << geom_names[i]);
                osg_scene->move_info_geometry(geom_names[i], temp_conf);
            }
            for(unsigned i = 0; i < geom_colors.size(); i++)
            {
//                PRX_ERROR_S ("Changing info geom color: " << geom_names[i]);
                osg_scene->change_info_geom_color(geom_names[i], geom_colors[i]);
            }
        }
        
        scene_t* osg_visualization_t::get_scene()
        {
            PRX_ASSERT(initialized);
            return &(viewer.get_scene());
        }

        viewer_t* osg_visualization_t::get_viewer()
        {
            PRX_ASSERT(initialized);
            return &viewer;
        }

        void osg_visualization_t::take_screenshot(unsigned screen_num, int num_screenshots)
        {
            window_screenshot_queue.push_back(std::make_pair<unsigned,int>(screen_num, num_screenshots));
        }



    }
}
