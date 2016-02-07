/**
 * @file osg_viewer.cpp 
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

#include "prx/visualization/PLUGINS/OSG/osg_viewer.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_handler_wrapper.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_ghost_switch.hpp"

#include <osg/Version>

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

osg_viewer_t::osg_viewer_t()
{
}

osg_viewer_t::~osg_viewer_t()
{
    delete osg_scene;
}

// TODO: Needed?
void osg_viewer_t::set_view_fullscreen()
{
}

/**
 * Sets the view into window mode given the position of the left upper window position
 * and the width/height of the screen.
 *
 * @brief Sets to window mode.
 *
 * @remark pos_x The x position fo the left upper window position.
 * @remark pos_y The y position fo the left upper window position.
 * @remark width The width of the window.
 * @remark height The height of the window.
 */
// TODO: Needed?
void osg_viewer_t::set_view_window(  )
{
}

/**
 * Sets the clear color of the view.
 *
 * @brief Sets the clear color.
 *
 * @param r The red value of the color.
 * @param g The green value of the color.
 * @param b The blue value of the color.
 * @param a The alpha(transparency) value of the color.
 */
// TODO: Needed?
void osg_viewer_t::set_clear_color(  )
{
}


bool osg_viewer_t::done()
{
    master_viewer.stopThreading();
    return master_viewer.done();
}


void osg_viewer_t::frame()
{
    // Updates all cameras
    foreach (osg::ref_ptr<osg_window_t> window, windows)
    {
        //PRX_LOG_DEBUG ("Stepping through window frames");
        window->frame(selectedGroup);
        window->get_current_camera().camera_frame();
        window->update_custom_mouse();
    }
    // moves the skydome
    vector_t eye_position;
    //PRX_LOG_DEBUG("Setting skydome position");
    windows[0]->get_current_camera().get_camera_vector(0,eye_position);
//    eye_position[2] += 10;
    skydome->setPosition( toVec3(eye_position) );

    // draw ghosts
    //PRX_LOG_DEBUG ("Rendering ghosts");
    //osg_scene.render_ghosts(draw_ghosts);
    master_viewer.frame();
}


void osg_viewer_t::run()
{
    // Empty Implementation
}


void osg_viewer_t::resize( )
{
}


void osg_viewer_t::init_sky()
{

    PRX_DEBUG_S ("Initializing skydome");
    //read in skydome from database
    osg::ref_ptr< osg::Geode > skydome_geom = (osg::Geode*)osgDB::readNodeFile("skydomes/skydome.osg");

    if( skydome_geom == NULL)
    {
      PRX_ERROR_S("Can't load skydome.osg. No skydome will be visible.");

      // Use a null (dummy) node if there's a problem with skydome.osg.
      skydome_geom = new osg::Geode();
    }    

    // set up movable skydome PAT
    skydome = new osg::PositionAttitudeTransform;
    skydome->addChild(skydome_geom);
    skydome->setName("skydome");
//    skydome->setPosition( osg::Vec3d(0.0, 0.0, 2000.0) );
//    skydome->setAttitude( osg::Quat(0.0, 0.0, 0.0, 1.0) );
    skydome->setScale( osg::Vec3d(150.0, 150.0, 150.0) );
//    
    osg_scene->get_wrapped_root()->asGroup()->addChild( skydome );

    // initialize the rotation of the skydome
    sky_orientation  = 0;
}



osg::ref_ptr<osgViewer::ScreenCaptureHandler> osg_viewer_t::init_handlers(int num)
{   
    char* w = std::getenv("PRACSYS_PATH");
    std::string dir(w);
    dir += ("/prx_output/");
    boost::filesystem::path output_dir (dir);
    if (!boost::filesystem::exists(output_dir))
    {
        PRX_ERROR_S("Output folder does not exist, creating directory");
        // If not create it
        boost::filesystem::create_directory( output_dir );

    }
    dir+="images/";
    PRX_DEBUG_S ("Dir: " << dir);
//    for (size_t i=0; i<p.we_wordc;i++ ) 
//        std::cout << w[i] << std::endl;

    // Check if images directory exists in current directory
    boost::filesystem::path images_dir (dir);
    //PRX_ERROR_S("Path: " << images_dir.system_complete(images_dir).string());
//    boost::filesystem::create_directory( images_dir );
    if (!boost::filesystem::exists(images_dir))
    {
        PRX_ERROR_S("Image folder does not exist, creating directory");
        // If not create it
        boost::filesystem::create_directory( images_dir );

    }
        // handles taking screen shots
    osg::ref_ptr<osgViewer::ScreenCaptureHandler> screenShooter = new osgViewer::ScreenCaptureHandler( new osgViewer::ScreenCaptureHandler::WriteToFile( dir, "jpg", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER ) );
        // allows for continous screen shots
    screenShooter->setKeyEventTakeScreenShot('r');

//#if OPENSCENEGRAPH_MAJOR_VERSION >= 2 && OPENSCENEGRAPH_MINOR_VERSION > 8
    screenShooter->setKeyEventToggleContinuousCapture('v');
//#endif
    
    screenShooter->setName("screen capturer");
    master_viewer.getView(num)->addEventHandler( screenShooter );
        // sets up statistics handler
    osgViewer::StatsHandler* stats = new osgViewer::StatsHandler();
    stats->setKeyEventTogglesOnScreenStats('~');
    stats->setName("prx statistics");
    master_viewer.getView(num)->addEventHandler(stats);

    // Add a handler that shows the rough, fine, or both geometries for objects
    // that support it.
    osgGA::GUIEventHandler* ghost_switch = new osg_ghost_switch_t(osg_scene);
    master_viewer.getView(num)->addEventHandler(ghost_switch);
    
    return screenShooter;
}

void osg_viewer_t::take_screenshot(unsigned screen_num, int num_screenshots)
{
    if (screen_num < screenshot_handlers.size())
    {
        screenshot_handlers[screen_num]->setFramesToCapture(num_screenshots);
        screenshot_handlers[screen_num]->startCapture();
    }
}


void osg_viewer_t::change_bounding_box( osg::Group* selected_group )
{
    
    selectedGroup = selected_group;
    if (osg_bbox.valid())
    {
        osg_bbox->getParent(0)->removeChild(osg_bbox);
    }
    if (selected_group && render_bbox)
    {
        osg_bbox = new osg::Geode();

        osg_bbox->setName("extents_geode");

        osg::StateSet* new_state;
        new_state = new osg::StateSet();
        osg_bbox->setStateSet(new_state);

        osg::PolygonMode* polyModeObj = new osg::PolygonMode();
        polyModeObj->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
        osg_bbox->getStateSet()->setAttribute(polyModeObj);

        osg::LineWidth* lw = new osg::LineWidth();
        lw->setWidth(2.0);
        osg_bbox->getStateSet()->setAttribute(lw);

        // create a new bounding box
        osg::BoundingBox bbox;

        bbox.init();
        bbox.expandBy( selected_group->getBound());

        double lx = bbox._max.x() - bbox._min.x();
        double ly = bbox._max.y() - bbox._min.y();
        double lz = bbox._max.z() - bbox._min.z();

        // make the box visible

        osg::ShapeDrawable* shape = new osg::ShapeDrawable( new osg::Box(osg::Vec3(0.0,0.0,0.0), (1.1*lx), (1.1*ly), (1.1*lz)));
        shape->setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0)); // red bounding box
        dynamic_cast<osg::Geode*>(osg_bbox.get())->addDrawable(shape);

        // Put the bbox under the system's PAT node
        selected_group->asTransform()->asPositionAttitudeTransform()->addChild( osg_bbox );
    }

}

void osg_viewer_t::set_handler(handler_t* hand)
{
    viewer_t::set_handler(hand);

    PRX_ASSERT(handler != NULL);
    osg_handler_wrapper_t* userHandler = new osg_handler_wrapper_t(this, handler);
    master_viewer.getView(0)->addEventHandler(userHandler);
}

// TODO: Needed?
void osg_viewer_t::toggle_ghost()
{
    draw_ghosts = !draw_ghosts;
}

void osg_viewer_t::toggle_bounding_box()
{
    render_bbox = !render_bbox;
    // if bounding box has been toggled off
    if (!render_bbox)
    {
        // if there exists a bounding box
        if (osg_bbox.valid())
        {
            // remove it from the scene
            osg_bbox->getParent(0)->removeChild(osg_bbox);
        }
    }
    // if bounding box has been toggled on
    else
    {
        // if there exists a bounding box
        if (osg_bbox.valid())
        {
            // remove it from the scene
            osg_bbox->getParent(0)->addChild(osg_bbox);
        }
    }
}

osg_scene_t& osg_viewer_t::get_scene()
{
    return *osg_scene;
}

std::vector< osg::ref_ptr< osg_window_t> >* osg_viewer_t::get_windows()
{
    return &windows;
}

void osg_viewer_t::init(const parameter_reader_t* reader)
{
    selectedGroup = NULL;
    // SET BOOLEAN RENDER GHOSTS. FIX SCENE TO REMOVE GHOSTS ON FALSE, AND ADD IT ON TRUE
    draw_ghosts = false;
    // default render bonding box true;
    render_bbox = true;

    viewer_t::init(reader);

    // Null shared_context is no shared context;
    osg::GraphicsContext* const shared_context = NULL;

    osg_scene = reader->initialize_new<osg_scene_t>("scene");

    unsigned int counter = 0;
    foreach (parameter_reader_t::reader_map_t::value_type key_value, reader->get_map("windows"))
    {
	const parameter_reader_t* window_reader = key_value.second;
        const int n = windows.size();
        osg::ref_ptr<osg_window_t> window = new osg_window_t(this);
        window->init(window_reader, n, back_color, shared_context, osg_scene);
        windows.push_back(window);
        PRX_ASSERT(windows.back()->get_wrapped_view() != NULL);
        master_viewer.addView(windows.back()->get_wrapped_view());
        screenshot_handlers.push_back(init_handlers(counter));
        screenshot_handlers[counter]->setFramesToCapture(1);
        ++counter;
        delete window_reader;
    }

    init_sky();

    // initialize threading
    /*
    master_viewer.setThreadingModel(master_viewer.suggestBestThreadingModel());
    master_viewer.setUpThreading();
    master_viewer.startThreading();
    */
    master_viewer.realize();
    PRX_DEBUG_S("Created osg_viewer_t.");
}

    }
 }
