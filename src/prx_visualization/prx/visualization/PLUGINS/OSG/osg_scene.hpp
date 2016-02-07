/**
 * @file osg_scene.hpp 
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

#ifndef PRACSYS_OSG_SCENE_HPP
#define PRACSYS_OSG_SCENE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/visualization/scene.hpp"
#include "osg_ghost_switch.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_light.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_material.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_texture.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_hud.hpp"

#include <osgText/Font>
#include <osgText/Text>


namespace prx 
{ 
    namespace vis 
    {

/**
 * OSG implementation of abstract scene class
 * 
 * @brief <b> OSG implementation of abstract scene class  </b>
 *
 * @authors Andrew Kimmel
 */
class osg_scene_t : public scene_t
{
private:

    // TODO: Needed?
    osg_ghost_switch_t scene_switch;
        
    // TODO: Needed?
    util::hash_t< std::string, osg::ref_ptr<osg::Node>, util::string_hash> mesh_lookup;

public:
    osg_scene_t();
    ~osg_scene_t();

    /**
     * @copydoc scene_t::add_geometry_batch()
     * @note Creates a new OSG node for each geometry, and performs optimizations at the end
     */
    void add_geometry_batch(const std::vector<util::geometry_info_t>& geoms);
    
    /**
     * Updates geometries in the scene with new configurations
     * 
     * @brief Moves the specified geometry to a new configuration
     * @param name The name of the geometry to move
     * @param config The configuration to move the geometry to
     */
    void move_geometry(const std::string& name, const util::config_t& config);
    
    /**
     * Updates geometries in the scene with new configurations
     * 
     * @brief Moves the specified geometry to a new configuration
     * @param name The name of the geometry to move
     * @param config The configuration to move the geometry to
     */
    void move_info_geometry(const std::string& name, const util::config_t& config);
    
    /**
     * @brief Changes the specified geometry's color according to RGBA value
     * @param name The name of the geometry to move
     * @param color The new color in RGBA
     */
    void change_info_geom_color(const std::string& name, const util::vector_t& color);
    
    /**
     * Add Light in the Scene. Light is the PRACSYS OSG Light.
     *
     * @brief Adds a PRACSYS light to the scene
     *
     * @param in_light The osg_light pointer.
     */
    void add_light( osg_light_t& light );
    
    /**
     * Adds an OSG light to the scene
     * 
     * @brief Adds an OSG light to the scene
     * @param light An OSG light to add to the scene
     */
    void add_light( osg::Light* light );

    /**
     * Stores the current scene in an OSG format file
     * 
     * @brief Stores the current scene in an OSG format file
     * @param filename The name of the file to save the scene to
     */
    void save_scene( const std::string& filename );

    // TODO: Needed?
    void render_ghosts(bool draw_ghosts);

    /**
     * Retrieves the pointer to the root node of the scene
     * 
     * @brief Retrieves the pointer to the root node of the scene
     * @return A pointer to the root node of the scene 
     */
    osg::ref_ptr<osg::Node> get_wrapped_root() const;

    // Create geometries in the scene manually (without outside influence)
    // TODO: Needed?
    void create_line();
    void create_sphere();

    /**
     * @copydoc scene_t::draw_text()
     */
    void draw_text(const scene_text_t& text_info);

    /**
     * @copydoc scene_t:: update_hud()
     */
    void update_hud(hud_element_t new_element);
    
    /**
     * @copydoc scene_t::create_hud()
     */
    void create_hud(hud_t* new_hud);

    /**
     * @copydoc scene_t::init()
     */
    void init(const util::parameter_reader_t* reader);
    
    /**
     * This function is used to create OSG nodes for geometries.  These nodes
     * can be attached to either the root OSG node or group nodes. Each of these
     * nodes has a parent pointer (typically an OSG PositionAttitudeTransform node)
     * 
     * @brief Initializes an OSG node for the geometry and attaches it to the scene 
     * @param system The pathname of the geometry
     * @param body The rigid body name of the geometry
     * @param reader Used to initialize the geometry
     * @param parent The parent node (cannot be NULL)
     * @param group The group node (can be NULL)
     */
    void init_geometries( const std::string& system, const std::string& body, const util::parameter_reader_t* reader, osg::PositionAttitudeTransform* parent, osg::Group* group = NULL);

    /**
     * Switches the scene
     * 
     * @brief Switches the scene
     * @param mode Which mode to switch to
     */
    // TODO: Needed?
    void switch_scene(osg_ghost_switch_t::scene_mode_t mode);
    
    /**
     * Plants are handled differently from a standard geometry, and hence
     * use this function for intialization (as oppposed to the standard
     * init_geometries function). This is due to their more "physical" nature,
     * and the fact that plants can be removed from the scene without deallocating
     * their node in OSG.
     * 
     * @brief Initializes the plants of the scene
     * @param source_node The name of the node these plants are coming from
     * @param paths The pathnames of the plants
     * @param template_paths The template plant paths (if any)
     */
    void initialize_plants(const std::string& source_node, const std::vector<std::string>& paths, const std::vector<std::string>& template_paths );
    
    /**
     * @copydoc scene_t::remove_plant()
     */
    void remove_plant(const std::string& path);
    
    void add_ghost_plant(const std::string& path, util::config_t rel_conf);
    
    /**
     * @copydoc scene_t::visualize_plant()
     */
    void visualize_plant(const std::string& path, const int flag);
    
    /**
     * @copydoc scene_t::visualize_obstacles()
     */
    void visualize_obstacles(const std::string& obstacles_path);
    
    /**
     * @copydoc scene_t::visualize_ghost_plants()
     */
    void visualize_ghost_plants(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& ghost_configs);
        
    // Public maps
    
    /** @brief Maps names of template materials (such as red, blue, gren, etc.) to a wrapper class for osg::Materials */
    util::hash_t< std::string, osg_material_t* , util::string_hash> template_material_mapping;
    
    /** @brief Maps materials associated to a specific geometry */
    util::hash_t< std::string, osg_material_t* , util::string_hash> geometry_material_mapping;

    /** @brief Maps geometry names to visualization geometry PAT nodes */
    boost::unordered_map< std::string, osg::ref_ptr<osg::PositionAttitudeTransform> , util::string_hash> rigid_bodies;
    
    /** @brief Maps geometry names to info geometry PAT nodes */
    boost::unordered_map< std::string, osg::ref_ptr<osg::PositionAttitudeTransform> , util::string_hash> info_geoms;
    
    /** @brief Maps scene text names to text pointers */
    boost::unordered_map< std::string, osg::ref_ptr<osgText::Text> > scene_text_map;
    
    /** @brief Keeps track of which info geoms need configuration updates (for optimization) */
    std::vector<std::string> info_geoms_to_update;
    
    /** @brief Maps obstacle names to obstacles PAT nodes */
    boost::unordered_map< std::string, osg::ref_ptr<osg::PositionAttitudeTransform> , util::string_hash> obstacles;
    
    /** @brief Maps system pathnames to their respective osg Node pointers */
    boost::unordered_map< std::string, osg::ref_ptr<osg::Node> , util::string_hash> systems;
    
    boost::unordered_map< std::string, osg::ref_ptr<osg::Node> , util::string_hash> template_systems;
    
    /** @brief Maps HUD names to osg_hud_t pointers */
    util::hash_t< std::string, osg_hud_t* , util::string_hash> osg_hud_map;



    /** @brief The root node of the scene */
    osg::ref_ptr<osg::Group> root;
    
    /** @brief Selection plane in the scene */
    osg::ref_ptr<osg::PositionAttitudeTransform> selection_plane;

private:
    /**
     * Constructs OSG nodes given a geometry. Sets up the node with the properties
     * specified, such as transparency, color, thickness, etc.
     * 
     * @brief Construct an OSG node for a geometry
     * @param name The name of the geometry
     * @param geom A pointer to the geometry
     * @param transparent Whether or not the geometry should be transparent
     * @param use_color Whether or not the geometry should use a color 
     * @return A pointer to the OSG node containing the geometry
     */
    osg::ref_ptr<osg::Node> make_geometry(const std::string& name, const util::geometry_t* geom, osg_material_t* material, bool transparent, bool use_color = false);
    
    /** @brief Used in make geometry to determine line thickness of line geometric primitives */
    double line_thickness;

};

// Derive a class from NodeVisitor to find a node with a
//   specific name.
class FindAllGeodes : public osg::NodeVisitor
{
public:
    FindAllGeodes( std::vector<osg::ref_ptr< osg::Geode > >* find_geodes )
      : osg::NodeVisitor( // Traverse all children.
                osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
        {found_geodes = find_geodes; }

    // This method gets called for every node in the scene
    //   graph. Check each node to see if its name matches
    //   out target. If so, save the node's address.
    virtual void apply( osg::Node& node )
    {
        if (node.asGeode() != NULL)
        {
            found_geodes->push_back(node.asGeode());
        }

        // Keep traversing the rest of the scene graph.
        traverse( node );
    }


protected:
    std::vector<osg::ref_ptr< osg::Geode > >* found_geodes;
};


    }
 }
#endif
