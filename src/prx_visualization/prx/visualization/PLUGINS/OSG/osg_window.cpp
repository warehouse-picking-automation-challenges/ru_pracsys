/**
 * @file osg_window.cpp 
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

#include "prx/visualization/PLUGINS/OSG/osg_window.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_handler_wrapper.hpp"
#include "prx/visualization/visualization.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <algorithm>
#include <osg/ref_ptr>
#include <osg/GraphicsContext>

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

osg_window_t::osg_window_t(osg_viewer_t* osg_viewer) : osg_viewer(osg_viewer)
{
    view = new osgViewer::View();
}

osg_window_t::~osg_window_t() 
{
    //    PRX_LOG_WARNING("Destructor of osg_window");
}

osg::ref_ptr<osgViewer::View> osg_window_t::get_wrapped_view() const
{
    return view;
}

//osg::ref_ptr<osg_camera_t> osg_window_t::get_active_camera()
//{
//    return NULL; //cameras[active_camera];
//}

void osg_window_t::frame(osg::Group* pickedGroup)
{
    cameras[active_camera].set_view(view, pickedGroup, aspect_ratio);
    // TODO: What to do with result?
}

void osg_window_t::set_camera(const unsigned int num)
{
    PRX_ASSERT(num < cameras.size());

    //if (cameras[active_camera].is_follower())
    //cameras[active_camera].toggle_follow();

    active_camera = num;
}

bool osg_window_t::handle(const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action)
{
    PRX_ASSERT(osg_viewer != NULL);
    if( event.getEventType() == osgGA::GUIEventAdapter::KEYDOWN )
    {
        ros::param::set("prx/input_keys/" + int_to_str(event.getKey()), true);
        //        PRX_WARN_S( "Key press is: " << event.getKey());
        switch( event.getKey() )
        {
                //printf ("Event: key press\n");
            // FOWARD
            case 'w': case 'W':
                if( !cameras[active_camera].is_ortho() )
                    cameras[active_camera].set_move_pos_in(true);
                else if( cameras[active_camera].get_type() != 2 )
                {
                    cameras[active_camera].move_down();
                }

                cameras[active_camera].direction[4] = true;
                break;

            // BACK
            case 's': case 'S':
                if( !cameras[active_camera].is_ortho() )
                    cameras[active_camera].set_move_pos_in(false);
                else if( cameras[active_camera].get_type() != 2 )
                {
                    cameras[active_camera].move_up();
                }

                cameras[active_camera].direction[1] = true;
                break;

            // LEFT
            case 'a': case 'A':
                if( !cameras[active_camera].is_ortho() )
                    cameras[active_camera].set_move_pos_side(false);
                else if( cameras[active_camera].get_type() != 2 )
                    cameras[active_camera].set_move_pos_side(true);
                
                cameras[active_camera].direction[0] = true;
                break;

            // LEFT AND UP
            case 'q': case 'Q':

                if( cameras[active_camera].get_type() != 2 )
                {
                    if( !cameras[active_camera].is_ortho() )
                    {
                        cameras[active_camera].set_move_pos_side(false);
                        cameras[active_camera].move_up();
                    }
                    else
                    {
                        cameras[active_camera].set_move_pos_side(true);
                        cameras[active_camera].move_down();
                    }
                }
                else
                {
//                    cameras[active_camera].horizontal_rotation(75);
                    cameras[active_camera].direction[3] = true;
                }
                break;

            // UP
            case '2':
                if( cameras[active_camera].get_type() != 2 )
                {
                    if( !cameras[active_camera].is_ortho() )
                        cameras[active_camera].move_up();
                    else
                        cameras[active_camera].set_move_pos_in(true);
                }
                else
                {
                    cameras[active_camera].direction[6] = true;
                }
                break;

            // UP AND RIGHT
            case 'e': case 'E':

                if( cameras[active_camera].get_type() != 2 )
                {
                    if( !cameras[active_camera].is_ortho() )
                    {
                        cameras[active_camera].set_move_pos_side(true);
                        cameras[active_camera].move_up();
                    }
                    else
                    {
                        cameras[active_camera].move_down();
                        cameras[active_camera].set_move_pos_side(false);
                    }
                }
                else
                {
//                    cameras[active_camera].horizontal_rotation(-75);
                    cameras[active_camera].direction[5] = true;
                }
                break;

            // RIGHT
            case 'd': case 'D':
                if( !cameras[active_camera].is_ortho() )
                    cameras[active_camera].set_move_pos_side(true);
                else if( cameras[active_camera].get_type() != 2 )
                    cameras[active_camera].set_move_pos_side(false);
                
                cameras[active_camera].direction[2] = true;
                break;

            // RIGHT AND DOWN
            case 'c':
                if( !cameras[active_camera].is_ortho() )
                {
                    cameras[active_camera].set_move_pos_side(true);
                    cameras[active_camera].move_down();
                }
                else if( cameras[active_camera].get_type() != 2 )
                {
                    cameras[active_camera].set_move_pos_side(false);
                    cameras[active_camera].move_up();
                }
                break;

            // DOWN
            case 'x':
                if( cameras[active_camera].get_type() != 2 )
                {
                    if( !cameras[active_camera].is_ortho() )
                        cameras[active_camera].move_down();
                    else
                    {
                        cameras[active_camera].set_move_pos_in(false);
                    }
                }
                else
                {
                    cameras[active_camera].direction[7] = true;
                }
                break;

            // DOWN AND LEFT
            case 'z':
                if( !cameras[active_camera].is_ortho() )
                {
                    cameras[active_camera].move_down();
                    cameras[active_camera].set_move_pos_side(false);
                }
                else if( cameras[active_camera].get_type() != 2 )
                {
                    cameras[active_camera].move_up();
                    cameras[active_camera].set_move_pos_side(true);
                }
                break;

            // Resets to initial view (or default view)
            case ' ':
//                if( cameras[active_camera].get_type() != 2 )
                {
                    if( cameras[active_camera].is_initialized() )
                        cameras[active_camera].reset();
                    else
                        cameras[active_camera].reset_view();
                }
                break;

                // Slows down camera
            case '-':
                cameras[active_camera].speed_down();
                break;

                // Speeds up camera
            case '+':
                cameras[active_camera].speed_up();
                break;

                // Toggle camera to follow selected target
            case 'f':
                if( !cameras[active_camera].is_ortho() )
                {
                    cameras[active_camera].toggle_follow();
                }
                break;

                // Toggle ghost rendering
            case 'g':
                osg_viewer->toggle_ghost();
                break;

                // Toggle rendering of tree
            case 'b':
                osg_viewer->toggle_bounding_box();
                break;
            case 'l':

                if( osg_viewer->get_scene().root->containsNode(osg_viewer->get_scene().selection_plane) )
                {
                    osg_viewer->get_scene().root->removeChild(osg_viewer->get_scene().selection_plane);
                }
                else
                {
                    osg_viewer->get_scene().root->addChild(osg_viewer->get_scene().selection_plane);
                }
                break;
                // Used to switch between different cameras
            case osgGA::GUIEventAdapter::KEY_F1:
                set_camera(0);
                break;
            case osgGA::GUIEventAdapter::KEY_F2:
                set_camera(1);
                break;
            case osgGA::GUIEventAdapter::KEY_F3:
                set_camera(2);
                break;
            case osgGA::GUIEventAdapter::KEY_F4:
                set_camera(3);
                break;
            case osgGA::GUIEventAdapter::KEY_F5:
                set_camera(4);
                break;
            case osgGA::GUIEventAdapter::KEY_F6:
                set_camera(5);
                break;
            case osgGA::GUIEventAdapter::KEY_Page_Up:
                if( osg_viewer->get_scene().selection_plane )
                {
                    osg::Vec3d position = osg_viewer->get_scene().selection_plane->getPosition();
                    position[1] += 0.5;
                    osg_viewer->get_scene().selection_plane->setPosition(position);
                }
                break;
            case osgGA::GUIEventAdapter::KEY_Page_Down:
                if( osg_viewer->get_scene().selection_plane )
                {
                    osg::Vec3d position = osg_viewer->get_scene().selection_plane->getPosition();
                    position[1] -= 0.5;
                    osg_viewer->get_scene().selection_plane->setPosition(position);
                }
                break;
        }
    }
    else if( event.getEventType() == osgGA::GUIEventAdapter::PUSH )
    {
        //printf ("Event: mouse push\n");
        if( event.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
        {
            left_button = true;
        }
        mouse_x = event.getX();
        mouse_y = event.getY();
//        return false;
    }
    else if( event.getEventType() == osgGA::GUIEventAdapter::DRAG )
    {
        //printf ("Event: mouse drag\n");
        if( left_button && !cameras[active_camera].is_ortho())
        {
            // Calculate the absolute difference
            double temp_x, temp_y, diff_x, diff_y;
            temp_x = event.getX();
            temp_y = event.getY();
            diff_x = fabs(mouse_x - temp_x);
            diff_y = fabs(mouse_y - temp_y);

            // Horizontal
            if( mouse_x < temp_x )
                cameras[active_camera].horizontal_rotation(-diff_x);
            else
                cameras[active_camera].horizontal_rotation(diff_x);

            // Vertical
            if( mouse_y < temp_y )
                cameras[active_camera].vertical_rotation(-diff_y);
            else
                cameras[active_camera].vertical_rotation(diff_y);

            mouse_x = temp_x;
            mouse_y = temp_y;

        }

    }
    else if( event.getEventType() == osgGA::GUIEventAdapter::KEYUP )
    {
        ros::param::set("prx/input_keys/" + int_to_str(event.getKey()), false);
        //        PRX_WARN_S("Key released : " << event.getKey());

        switch( event.getKey() )
        {
            case 'a': case 'A':
                cameras[active_camera].direction[0] = false;
                break;
            case 's': case 'S':
                cameras[active_camera].direction[1] = false;
                break;
            case 'd': case 'D':
                cameras[active_camera].direction[2] = false;
                break;
            case 'q': case 'Q':
                cameras[active_camera].direction[3] = false;
                break;
            case 'w': case 'W':
                cameras[active_camera].direction[4] = false;
                break;
            case 'e': case 'E':
                cameras[active_camera].direction[5] = false;
                break;
            case '2':
                cameras[active_camera].direction[6] = false;
                break;
            case 'x': case 'X':
                cameras[active_camera].direction[7] = false;
                break;
        }
    }
    return true;

//    return false;
}

void osg_window_t::update_custom_mouse()
{
    if( different_cursor )
    {
        vector_t vec;
        cameras[active_camera].get_camera_vector(PRX_CAMERA_CENTER, vec);
        custom_mouse->setPosition(osg::Vec3d(vec[0], vec[1], vec[2] - 1.0));
    }

}

osg_camera_t& osg_window_t::get_current_camera()
{
    return cameras[active_camera];
}

void osg_window_t::init(const parameter_reader_t* reader, int num, const vector_t& back_color,
                        osg::GraphicsContext* shared_context, const osg_scene_t* sceneData)
{
    mouse_visible = true;
    different_cursor = false;

    view->setSceneData(sceneData->get_wrapped_root());

    const int x = reader->get_attribute_as<int>("xpos");
    const int y = reader->get_attribute_as<int>("ypos");
    const int width = reader->get_attribute_as<int>("width");
    const int height = reader->get_attribute_as<int>("height");
    if( reader->has_attribute("use_cursor") )
    {
        mouse_visible = reader->get_attribute_as<bool>("use_cursor");
    }
    if( reader->has_element("custom_cursor") )
    {
        custom_mouse = new osg::PositionAttitudeTransform();
        custom_mouse->addChild(osg_geode_t::setup_mesh(reader->get_attribute("custom_cursor/file")));
        sceneData->root->addChild(custom_mouse);
        different_cursor = true;
    }

    // TODO: Check if fewer than six cameras have been specified.
    // If not, add defaults.

    foreach(parameter_reader_t::reader_map_t::value_type key_value, reader->get_map("cameras"))
    {
        const parameter_reader_t* cam_reader = key_value.second;
        osg_camera_t new_camera;
        new_camera.init(cam_reader);
        cameras.push_back(new_camera);
    }
    while( cameras.size() < 6 )
    {
        cameras.push_back(osg_camera_t());
    }

    // Create a graphics context from the shared context with specified traits.
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = x; // how far in x the window appears?
    traits->y = y; // how far in y the window appears?
    traits->width = width; // width of new window
    traits->height = height; // height of new window
    traits->windowDecoration = true; // not sure
    traits->doubleBuffer = true; // we always want this! well, unless we want a better framerate possibly
    traits->sharedContext = shared_context; // if the viewer shares the same window as another, else this is null
    traits->useMultiThreadedOpenGLEngine = true; // used for threading
    //traits->screenNum = num; // name of screen
    //traits->supportsResize = true;
    // traits->vsync = true;


    if( mouse_visible )
        PRX_DEBUG_S("Mouse cursor is enabled\n");
    else
        PRX_DEBUG_S("Mouse cursor is disabled\n");
    traits->useCursor = mouse_visible;

    osg::ref_ptr<osg::GraphicsContext> context = osg::GraphicsContext::createGraphicsContext(traits);

    PRX_ASSERT(context.valid());
    PRX_DEBUG_S("GraphicsWindow has been created successfully");

    osg::Camera* camera = view->getCamera();
    camera->setViewport(new osg::Viewport(0, 0, width, height));
    camera->setGraphicsContext(context);
    camera->setProjectionResizePolicy(osg::Camera::HORIZONTAL);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    camera->setProjectionMatrixAsPerspective(75.0, width / height, 0.1, 10000);
    camera->setClearColor(toVec4(back_color));

    camera->setDrawBuffer(GL_BACK);
    camera->setReadBuffer(GL_BACK);
    camera->setAllowEventFocus(true);
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Set initial camera.
    active_camera = 0;
    set_camera(0);
    aspect_ratio = width / height;

    view->init();
    //TODO: Circular referrence to view
    view->addEventHandler(this);

    PRX_DEBUG_S("Created osg_window_t");
}

    }
 }
