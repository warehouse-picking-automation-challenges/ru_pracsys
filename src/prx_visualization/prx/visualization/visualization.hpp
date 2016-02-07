/**
 * @file visualization.hpp 
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

#ifndef PRACSYS_VISUALIZATION_HPP
#define PRACSYS_VISUALIZATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/communication/tf_listener.hpp"
#include "prx/visualization/scene.hpp"
#include "prx/visualization/viewer.hpp"

#include <boost/program_options.hpp>

namespace prx 
{ 
    namespace sim
    {
        class config_t;
    }
    
    namespace vis 
    {

/**
 * The abstract visualization class. Manages memory for and gives access to the 
 * visualization scene and viewer.  
 * @brief <b> Abstract visualization class </b>
 *
 * @authors Andrew Kimmel
 */
class visualization_t
{

public:
    virtual ~visualization_t() {};

    /**
     * Retrieves the pointer to the visualization's scene
     * 
     * @brief Retrieves the pointer to the visualization's scene
     * @return A pointer to the scene
     */
    virtual scene_t* get_scene() = 0;
    
    /**
     * Retrieves the pointer to the visualization's viewer
     * 
     * @brief Retrieves the pointer to the visualization's viewer
     * @return A pointer to the viewer
     */
    virtual viewer_t* get_viewer() = 0;

    /**
     * Handles initialization for the visualization class along
     * with the class members.
     * 
     * @brief Handles initialization for the visualization class along
     * with the class members.
     * 
     * @param reader The parameter reader
     */
    virtual void init(const util::parameter_reader_t* reader) = 0;
    
    /**
     * @brief Handles updating a set of info geoms
     * @param geom_names The names of the info geoms to update
     * @param geom_configs The configurations to update the geoms with
     * @param poll_tf True: polls tf to obtain configs. False: Uses the configurations passed in as parameter.
     */
    virtual void update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf) = 0;
    
    virtual void take_screenshot(unsigned screen_num, int num_screenshots);
    
    /**
     * 
     * @brief Runs the visualization loop
     * @return 
     */
    virtual bool run() = 0;

    /** @brief A string describing the type of this plugin. "visualization" */
    static const char* const plugin_type_name;

    /**
     * Implementations may override this if they need any arguments from the
     * command line.
     * 
     * @brief 
     * @param options Option map that has been parsed from the command-line.
     */
    virtual void notify_program_options(boost::program_options::variables_map options) {};
    
protected:
    util::tf_listener_t* listener;

};

    }
 }
#endif
