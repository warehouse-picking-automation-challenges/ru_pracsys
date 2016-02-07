/**
 * @file visualization_comm.cpp 
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


#include "prx/visualization/visualization_comm.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/visualization/visualization.hpp"
#include "PLUGINS/OSG/osg_viewer.hpp"

#include <boost/bind.hpp>

#include <ros/topic_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <osgDB/ReadFile>

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

PLUGINLIB_EXPORT_CLASS( prx::vis::visualization_comm_t, prx::vis::visualization_comm_t)

namespace prx
{
    using namespace util;
    namespace vis
    {        

        // TODO: Remove?
        //class update_camera_sensor_callback
        //{
        //private:
        //    const std::string name;
        //    osg::ref_ptr<osgViewer::Viewer> viewer;
        //    osg::ref_ptr<osg::Texture2D> img_texture;
        //    osg::ref_ptr<osg::Group> root;
        //    bool init;
        //
        //public:
        //    update_camera_sensor_callback(const std::string& _name) : name(_name),
        //                                                     init(false){}
        //
        //    void operator()(const sensor_msgs::ImageConstPtr& msg)
        //    {
        //        if(!init)
        //        {
        //            viewer = new osgViewer::Viewer();
        //
        //            osg::ref_ptr<osg::GraphicsContext::Traits> traits =
        //                                             new osg::GraphicsContext::Traits();
        //
        //            traits->x = 10; // how far in x the window appears?
        //            traits->y = 50; // how far in y the window appears?
        //            traits->windowDecoration = true; // not sure
        //
        //            traits->width  = msg->width;
        //            traits->height = msg->height;
        //
        //            traits->doubleBuffer = false;
        //            traits->pbuffer      = false;
        //
        //            osg::ref_ptr<osg::GraphicsContext> graphics_context =
        //                            osg::GraphicsContext::createGraphicsContext(traits);
        //            PRX_ASSERT(graphics_context.valid());
        //
        //            osg::ref_ptr<osg::Camera> camera = new osg::Camera(*viewer->getCamera());
        //            camera->setGraphicsContext(graphics_context);
        //            camera->setViewport(new osg::Viewport(0,0,msg->width, msg->height));
        //            camera->setDrawBuffer(GL_FRONT);
        //            camera->setReadBuffer(GL_FRONT);
        //            camera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        //            camera->setClearColor(osg::Vec4(255,0,255,0));
        //
        ////            camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
        ////            camera->setProjectionMatrixAsPerspective( 75.0, traits->width/traits->height, 0.1, 10000 );
        //
        //            viewer->setCamera(camera);
        //
        ////        osg::Vec3Array* HUDnormals = new osg::Vec3Array;
        ////        HUDnormals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
        ////        HUDBackgroundGeometry->setNormalArray(HUDnormals);
        ////        HUDBackgroundGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
        //
        //            root = new osg::Group();
        //            viewer->setSceneData(root);
        //            viewer->realize();
        //            init = true;
        //        }
        //
        //        root->removeChildren(0, 1);
        //        osg::ref_ptr<osg::Geode> quad_geode = new osg::Geode();
        //        osg::ref_ptr<osg::Geometry> quad_geometry = new osg::Geometry();
        //
        //        osg::Projection* HUDProjectionMatrix = new osg::Projection;
        //        HUDProjectionMatrix->setMatrix(osg::Matrix::ortho2D(0,1000,0,1000));
        //
        //        osg::MatrixTransform* HUDModelViewMatrix = new osg::MatrixTransform;
        //        HUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        //        HUDModelViewMatrix->setMatrix(osg::Matrix::identity());
        //
        //        root->addChild(HUDProjectionMatrix);
        //        HUDProjectionMatrix->addChild(HUDModelViewMatrix);
        //        HUDModelViewMatrix->addChild( quad_geode );
        //
        //        osg::ref_ptr<osg::Vec3Array> quad_coords = new osg::Vec3Array; // vertex coords
        //        // counter-clockwise
        //        quad_coords->push_back(osg::Vec3d(0, 0, .8));
        //        quad_coords->push_back(osg::Vec3d(msg->width, 0, .8));
        //        quad_coords->push_back(osg::Vec3d(msg->width, msg->height, .8));
        //        quad_coords->push_back(osg::Vec3d(0, msg->height, .8));
        //
        //        osg::DrawElementsUInt* quad_da =
        //          new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
        //        quad_da->push_back(0);
        //        quad_da->push_back(1);
        //        quad_da->push_back(2);
        //        quad_da->push_back(3);
        //
        //        osg::ref_ptr<osg::Vec2Array> quad_tcoords = new osg::Vec2Array; // texture coords
        //        quad_tcoords->push_back(osg::Vec2(0.0f, 0.0f));
        //        quad_tcoords->push_back(osg::Vec2(1.0f, 0.0f));
        //        quad_tcoords->push_back(osg::Vec2(1.0f, 1.0f));
        //        quad_tcoords->push_back(osg::Vec2(0.0f, 1.0f));
        //
        ////            osg::ref_ptr<osg::DrawArrays> quad_da = new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4);
        //
        //
        ////            img_texture->setDataVariance(osg::Object::DYNAMIC);
        ////            img_texture->setImage(osgDB::readImageFile("Ilias_test_camera_60.jpg"));
        //
        //        quad_geode->addDrawable(quad_geometry);
        //        quad_geometry->addPrimitiveSet(quad_da);
        //
        //        quad_geometry->setVertexArray(quad_coords.get());
        //        quad_geometry->setTexCoordArray(0, quad_tcoords.get());
        //        
        //        img_texture = new osg::Texture2D;
        //        osg::ref_ptr<osg::Image> image = new osg::Image;
        ////        osg::ref_ptr<osg::Image> image = osgDB::readImageFile("Ilias_test_camera_60.jpg");
        //        unsigned int enc = msg->encoding == sensor_msgs::image_encodings::RGB8 ?
        //                                                               GL_RGB : GL_RGBA;
        //        std::string data(msg->data.begin(), msg->data.end());
        //
        //        image->setImage((int)msg->width, (int)msg->height, 1, (GLint)enc, (GLenum)enc,
        //                    GL_UNSIGNED_BYTE, (unsigned char*)data.c_str(), osg::Image::NO_DELETE);
        //
        //        // Assign the texture to the image we read from file:
        //        img_texture->setImage(image);
        //
        //        osg::StateSet *state_set = quad_geode->getDrawable(0)->getOrCreateStateSet();
        //
        //        state_set->setRenderBinDetails( 11, "DepthSortedBin");
        //        state_set->setMode(GL_BLEND,osg::StateAttribute::ON);
        //        state_set->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
        //        state_set->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
        //        state_set->setTextureAttributeAndModes(0,img_texture,osg::StateAttribute::ON);
        //        quad_geode->setStateSet(state_set);
        ////        window->get_wrapped_view()->getCamera()->getGraphicsContext()->clear();
        ////        glDrawPixels(msg->width, msg->height, GL_RGB, GL_UNSIGNED_BYTE, data.c_str());
        ////        window->get_wrapped_view()->getCamera()->getGraphicsContext()->swapBuffers();
        ////        glutSwapBuffers();
        ////        bufferWriter->unbindBuffer(state->getContextID());
        //        viewer->updateTraversal();
        //        viewer->renderingTraversals();
        //        viewer->frame();
        //
        //        PRX_LOG_WARNING("Received message from %s", name.c_str());
        //        PRX_LOG_WARNING("Width %d Height %d", msg->width, msg->height);
        //    }
        //};

        pluginlib::ClassLoader<visualization_comm_t> visualization_comm_t::loader("prx_visualization", "prx::vis::visualization_comm_t");

        void visualization_comm_t::link_visualization(visualization_t* in_vis)
        {
            vis = in_vis;
        }

        pluginlib::ClassLoader<visualization_comm_t>& visualization_comm_t::get_loader()
        {
            return loader;
        }

        /**
         *
         */
        bool visualization_comm_t::add_hud_callback(create_HUD_srv::Request& request, create_HUD_srv::Response& response)
        {

            PRX_DEBUG_S("Adding new HUD");
            hud_t* new_hud = new hud_t(request.hud_name, request.area, request.color, request.texture);

            PRX_DEBUG_S("Calling scene->create hud with name "<< new_hud->name.c_str());
            PRX_ASSERT(vis != NULL);
            vis->get_scene()->create_hud(new_hud);
            //hud_t* potential_HUD = vis->get_scene()->hud_map[request.hud_name];

            return true;

        }

        /**
         *
         */
        bool visualization_comm_t::update_hud_callback(add_HUD_element_srv::Request& request, add_HUD_element_srv::Response& response)
        {
            PRX_DEBUG_S("Updating HUD callback");
            PRX_ASSERT(vis != NULL);
            hud_t* potential_HUD = vis->get_scene()->hud_map[request.parent_name];
            // First check if the parent HUD exist
            if( !potential_HUD )
            {
                PRX_WARN_S("HUD parent with name "<< request.parent_name.c_str() <<" does not exist");
            }
            else
            {
                PRX_DEBUG_S("Creating hud element");
                // set up hud_element
                hud_element_t new_element;
                new_element.character_size = request.character_size;
                PRX_DEBUG_S("Received color: "<<request.color[0]<<", "<<request.color[1]<<", "<<request.color[2]<<","<<request.color[3]);
                new_element.color.resize(4);
                new_element.color[0] = request.color[0];
                new_element.color[1] = request.color[1];
                new_element.color[2] = request.color[2];
                new_element.color[3] = request.color[3];
                new_element.font = request.font;
                new_element.our_name = request.our_name;
                new_element.parent_name = request.parent_name;
                PRX_DEBUG_S("Received position: "<<request.position[0]<<", "<<request.position[1]<<", "<<request.position[2]);
                new_element.position.resize(3);
                new_element.position[0] = request.position[0];
                new_element.position[1] = request.position[1];
                new_element.position[2] = request.position[2];
                new_element.text = request.text;

                PRX_DEBUG_S("Created new element with parent: "<<new_element.parent_name<<", name: "<<new_element.our_name<<", and text: " << new_element.text);

                // create/update HUD
                vis->get_scene()->update_hud(new_element);

            }

            return true;
        }

        ///**
        // *
        // */
        //bool visualization_comm_t::add_twoD_text_callback(add_twoD_text_srv::Request& request, add_twoD_text_srv::Response& response)
        //{
        //
        //    PRX_LOG_DEBUG ("Adding new 2D text");
        //    PRX_ASSERT(vis != NULL);
        //    scene_text_t* new_text = new scene_text_t(request.our_name, request.associate_system, request.text, request.font, request.relative_position, request.color, request.text_size);
        //
        //   // PRX_LOG_DEBUG ("Calling scene->create text with name %s", new_text->name.c_str());
        //    vis->get_scene()->(new_text);
        //
        //    return true;
        //
        //}

        ///**
        // *
        // */
        //bool visualization_comm_t::update_twoD_text_callback(update_twoD_text_srv::Request& request, update_twoD_text_srv::Response& r)
        //{
        //    PRX_LOG_DEBUG ("Updating text callback");
        //    PRX_ASSERT(vis != NULL);
        //    scene_text_t* potential_text = vis->get_scene()->text_map[request.our_name];
        //    // First check if the text exists
        //    if (!potential_text)
        //    {
        ////        PRX_LOG_WARNING("HUD parent with name %s does not exist", request.parent_name.c_str());
        //    }
        //
        //    return true;
        //
        //}

        /**
         *
         */
        visualization_comm_t::visualization_comm_t()
        {
            decribe_geometries_service = node.advertiseService<visualization_comm_t,
                    describe_geometries_srv::Request, describe_geometries_srv::Response >
                    ("visualization/geometries", &visualization_comm_t::geometry_callback, this);
            remove_geometries_service = node.advertiseService<visualization_comm_t,
                    describe_geometries_srv::Request, describe_geometries_srv::Response >
                    ("visualization/remove_geometries", &visualization_comm_t::geometry_callback, this);

            send_plants_service = node.advertiseService<visualization_comm_t,
                    send_plants_srv::Request, send_plants_srv::Response >
                    ("visualization/plants", &visualization_comm_t::plants_callback, this);

            remove_plant_service = node.advertiseService<visualization_comm_t,
                    remove_plant_srv::Request, remove_plant_srv::Response >
                    ("visualization/remove_plant", &visualization_comm_t::remove_plant_callback, this);

            visualize_plant_service = node.advertiseService<visualization_comm_t,
                    visualize_plant_srv::Request, visualize_plant_srv::Response >
                    ("visualization/visualize_plant", &visualization_comm_t::visualize_plant_callback, this);
            
            visualize_obstacles_service = node.advertiseService<visualization_comm_t,
                    visualize_obstacles_srv::Request, visualize_obstacles_srv::Response >
                    ("visualization/visualize_obstacles", &visualization_comm_t::visualize_obstacles_callback, this);
            
            update_info_geoms_service = node.advertiseService<visualization_comm_t,
                    update_info_geoms_srv::Request, update_info_geoms_srv::Response >
                    ("visualization/update_info_geoms", &visualization_comm_t::update_info_geoms_callback, this);

            add_HUD = node.advertiseService("visualization/add_HUD",
                                            &visualization_comm_t::add_hud_callback, this);
            update_HUD = node.advertiseService("visualization/update_HUD",
                                               &visualization_comm_t::update_hud_callback, this);
            
            visualize_ghost_plants_service = node.advertiseService<visualization_comm_t,
                    visualize_ghost_plants_srv::Request, visualize_ghost_plants_srv::Response >
                    ("visualization/visualize_ghost_plants", &visualization_comm_t::visualize_ghost_plants_callback, this);
            
            take_screenshot_service = node.advertiseService<visualization_comm_t,
                    take_screenshot_srv::Request, take_screenshot_srv::Response >
                    ("visualization/take_screenshot", &visualization_comm_t::take_screenshot_callback, this);
            
        }
        
        bool visualization_comm_t::update_info_geoms_callback(update_info_geoms_srv::Request& request,
                                                  update_info_geoms_srv::Response& response)
        {
            std::vector<config_t> info_confs;
            if (!request.info_configs.empty())
            {
                foreach(geometry_msgs::Pose pose, request.info_configs)
                {
                    info_confs.push_back(pose_to_config_t(&pose));
                }
            }
            std::vector<vector_t> info_colors;
            vector_t color_vec(4);
            if (!request.info_colors.empty())
            {
                foreach(Vec4_msg color, request.info_colors)
                {
                    color_vec[0] = color.r;
                    color_vec[1] = color.g;
                    color_vec[2] = color.b;
                    color_vec[3] = color.a;
                    info_colors.push_back(color_vec);
                }
            }
            this->vis->update_info_geoms(request.info_names, info_confs, info_colors, request.poll_tf);
            
            return true;
//            config_t temp_conf;
//            for(unsigned i = 0; i < geom_names.size(); i++)
//            {
//                if (poll_tf)
//                {
//                    listener.lookup(geom_names[i], temp_conf);
//                }
//                else
//                {
//                    temp_conf = geom_configs[i];
//                }
//                
//                scene->move_info_geometry(geom_names[i], temp_conf);
//            }
//            return true;
        }
        
        
        bool visualization_comm_t::take_screenshot_callback(take_screenshot_srv::Request& request,
                                                  take_screenshot_srv::Response& response)
        {
            vis->take_screenshot(request.screen_num, request.number_of_screenshots);
            return true;
        }

        //void visualization_comm_t::poll_topics(void)
        //{
        //    ros::master::V_TopicInfo topics;
        //    ros::master::getTopics(topics);
        //
        //    foreach(ros::master::TopicInfo topic, topics)
        //    {
        //        if(!topic.name.compare(0, 22, "/sensing/topic/camera/") &&
        //           camera_sensors.find(topic.name) == camera_sensors.end())
        //        {
        //            camera_sensors[topic.name] = true;
        //            update_camera_sensor_callback callback(topic.name);
        //            subscriptions[topic.name] = node.subscribe<sensor_msgs::Image>
        //                                                      (topic.name, 1, callback);
        //            PRX_LOG_WARNING("Added topic %s", topic.name.c_str());
        //        }
        //    }
        //}

    }
}
