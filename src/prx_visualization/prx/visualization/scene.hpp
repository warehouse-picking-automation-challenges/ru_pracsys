/**
 * @file scene.hpp 
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

#ifndef PRACSYS_SCENE_HPP
#define	PRACSYS_SCENE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"
#include "prx/visualization/hud.hpp"
#include "prx/visualization/scene_text.hpp"

namespace prx 
{ 
    namespace vis 
    {


/**
 * The abstract scene class. This class contains interface for visualizing
 * objects in the scene.
 * 
 * @brief <b> Abstract scene class </b>
 *
 * @authors Andrew Kimmel
 */
class scene_t
{

  public:

    virtual ~scene_t(){ };

    /**
     * Visualizes a batch of geometries in the scene
     * 
     * @brief Visualizes a batch of geometries in the scene
     * @param geoms The batch of geometries to visualize
     */
    virtual void add_geometry_batch(const std::vector<util::geometry_info_t>& geoms) = 0;

    /**
     * Used to initialize a set of plants.  The pathnames of the plants are passed,
     * as well as any template paths (used for parameter reading).
     * 
     * @brief Initializes a set of plants
     * @param paths The paths of the plants to be initialized
     * @param template_paths The template paths, if any, for the plants
     */
    virtual void initialize_plants(const std::string& source_node, const std::vector<std::string>& paths, const std::vector<std::string>& template_paths) = 0;
    
    /**
     * Removes a single plant from the scene and deallocates its memory.
     * 
     * @brief Removes a single plant from the scene
     * @param path The path of the plant to remove
     */
    virtual void remove_plant(const std::string& path) = 0;
    
    /**
     * Allows for plants to be toggled on/off in the scene.
     * 
     * The memory for the plants stays allocated in either case.
     * 
     * @brief Toggles visualization of a single plant
     * @param path The pathname of the plant to be visualized
     * @param flag True to visualize plant, false to not visualize plant
     */
    virtual void visualize_plant(const std::string& path, const int flag) = 0;
    
    /**
     * Visualizes obstacles
     */
    virtual void visualize_obstacles(const std::string& path) = 0;

    /**
     * Visualizes a ghost of the plant at a specific config.
     * 
     * 
     * @brief Ghosts.
     * @param path The pathname of the plant to be visualized
     * @param flag True to visualize plant, false to not visualize plant
     */
    virtual void visualize_ghost_plants(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& ghost_configs) = 0;

    // Create HUD
    
    /**
     * Creates a new HUD area in the scene. For example,
     * this reserves a certain amount of space on the screen,
     * loads up the texture for it (if any), and positions it.
     * 
     * @brief Creates a new HUD area in the scene
     * @param new_hud Used to create the HUD area
     */
    virtual void create_hud(hud_t* new_hud) = 0;
    
    /**
     * Adds an element to an existing HUD area. For example,
     * this could be used to add GUI buttons, progress bars, etc.
     * 
     * @brief Adds an element to an existing HUD area
     * @param element The element to add to the HUD area
     */
    virtual void update_hud(hud_element_t element) = 0;

    // Create two dimensional scene text
    
    /**
     * Visualizes text in the scene
     * 
     * @brief Visualizes text in the scene
     * @param text_info Contains necessary information to visualize text in the scene
     */
    virtual void draw_text(const scene_text_t& text_info) = 0;

    /** @brief Used by the scene to keep track of its HUD */
    util::hash_t<std::string, hud_t*, util::string_hash> hud_map; 
    /** @brief Used by the scene to keep track of its scene text */
    util::hash_t<std::string, scene_text_t*, util::string_hash> text_map; 

    /**
     * Initializes the scene through the parameter reader
     * 
     * @brief Initializes the scene through the parameter reader
     * @param reader Used to initialize the scene
     */
    virtual void init(const util::parameter_reader_t* reader) = 0;

};

    }
 }
#endif

