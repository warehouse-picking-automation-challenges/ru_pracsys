/**
 * @file osg_visualization.hpp 
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

#ifndef PRX_OSG_VISUALIZATION_HPP
#define PRX_OSG_VISUALIZATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/visualization/visualization.hpp"
#include "prx/visualization/visualization_comm.hpp"
#include "prx/visualization/handler.hpp"
#include "prx/visualization/scene.hpp"
#include "prx/visualization/viewer.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_viewer.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"

#include <vector>


namespace prx 
{ 
    namespace vis 
    {

/**
 * The OSG plugin for the visualization class. Top level class
 * that executes the frame loop. Deals with communicating with tf,
 * and thus updating configurations for geometries (by calling the
 * appropriate scene functions).
 * 
 * @brief <b> The OSG plugin for the visualization class. </b>
 *
 * @authors Andrew Kimmel
 */

class osg_visualization_t : public visualization_t
{
protected:
    /** @brief The viewer*/
    osg_viewer_t viewer;
    /** @brief Pointer to the visualization communication class*/
    visualization_comm_t *comm;
    /** @brief Flag for checking initialization of the class */
    bool initialized;
    osg_scene_t* osg_scene;
    
    std::vector< std::pair<unsigned,int> > window_screenshot_queue;

public:

    osg_visualization_t();
    ~osg_visualization_t();

    /**
     * @copydoc visualization_t::get_scene()
     */
    scene_t* get_scene();
    
    /**
     * @copydoc visualization_t::get_viewer()
     */
    viewer_t* get_viewer();

    /**
     * @copydoc visualization_t::init
     * @note Also initializes a visualization_comm_t class
     */
    void init(const util::parameter_reader_t* reader);
    
    /**
     * @copydoc visualization_t::run()
     * @note Updates configurations of plants, and any updates to info geoms
     */
    bool run();
    
    /**
     * @copydoc visualization_t::run()
     * @note Updates configurations of plants, and any updates to info geoms
     */
    void update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf);

    void take_screenshot(unsigned screen_num, int num_screenshots);

};

    }
 }
#endif 
