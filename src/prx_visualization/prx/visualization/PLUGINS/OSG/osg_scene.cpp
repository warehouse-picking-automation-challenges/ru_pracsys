
/**
 * @file osg_scene.cpp 
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

#include "prx/visualization/PLUGINS/OSG/osg_scene.hpp"

#include "prx/visualization/visualization.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_geode.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_ghost_switch.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <osgUtil/Optimizer>
#include <queue>

#include <osg/BlendFunc>
#include <osg/LightModel>
namespace prx
{
    using namespace util;

    namespace vis
    {

        /**
         * Constructor for the OSG Scene class.
         *
         * @brief Constructor
         */
        osg_scene_t::osg_scene_t() : scene_switch(this) { }

        /**
         * Destructor for the OSG Scene class.
         *
         * @brief Destructor
         */
        osg_scene_t::~osg_scene_t()
        {
            PRX_DEBUG_S("Destructor of osg_scene");
            for( hash_t< std::string, osg_material_t*, string_hash>::iterator it = template_material_mapping.begin(); it != template_material_mapping.end(); ++it )
            {
                delete it->second;
            }
            for( hash_t< std::string, osg_material_t*, string_hash>::iterator it = geometry_material_mapping.begin(); it != geometry_material_mapping.end(); ++it )
            {
                delete it->second;
            }
            for( hash_t< std::string, osg_hud_t*, string_hash>::iterator it = osg_hud_map.begin(); it != osg_hud_map.end(); ++it )
            {
                delete it->second;
            }
        }

        void osg_scene_t::add_light(osg_light_t& light)
        {
            osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
            ls->setLight(light.get_wrapped_light());

            root->addChild(ls);
        }

        /**
         * Add Light in the Scene. Light is the PRACSYS OSG Light.
         *
         * @brief Add Light.
         *
         * @param in_light The osg_light pointer.
         */
        void osg_scene_t::add_light(osg::Light* light)
        {
            osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
            ls->setLight(light);

            root->addChild(ls);
        }

        /**
         * Save the Scene as a model of OSG format.
         *
         * @brief Save the Scene as a model.
         *
         * @param filename The full path of the model name.
         */
        void osg_scene_t::save_scene(const std::string& filename)
        {
            osgDB::writeNodeFile(*root.get(), filename);
        }

        /**
         * Return the Scene.
         * 
         * @remarks This funciton is called by the viewer class.
         *
         * @return OSG root node of the Scene.
         */
        osg::ref_ptr<osg::Node> osg_scene_t::get_wrapped_root() const
        {
            return root;
        }

        bool flag = true;

        osg::ref_ptr<osg::Node> osg_scene_t::make_geometry(const std::string& material_name, const geometry_t* geom, osg_material_t* material, bool transparent, bool use_color)
        {

            // The material settings from XML
            //    PRX_DEBUG_S("Looking for material: " << material_name.c_str() << "   for the type : " << geom->get_type());

            PRX_ASSERT(template_material_mapping.find(material_name) != template_material_mapping.end());
            material = new osg_material_t(*(template_material_mapping[material_name]));

            PRX_ASSERT(geom != NULL);
            // TODO: Eventually line_thickness could represent other attributes as well (i.e. include transparency and line)
            osg::ref_ptr<osg::Node> node = osg_geode_t::setup_geometry(geom, transparent, line_thickness);


            // Use diffuse/specular/shininess
            if( !transparent && !use_color )
                material->setColorMode(osg::Material::OFF);
            osg::StateSet* stateSet = node->getOrCreateStateSet();


            if( transparent )
            {

                stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

                // Enable depth test so that an opaque polygon will occlude a transparent one behind it.
                stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
                stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

                // Conversely, disable writing to depth buffer so that
                // a transparent polygon will allow polygons behind it to shine thru.
                // OSG renders transparent polygons after opaque ones.
                osg::Depth* depth = new osg::Depth;
                depth->setWriteMask(false);
                stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

                // Disable conflicting modes.
                stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

                PRX_DEBUG_S("setting transparency");
                material->setTransparency(osg::Material::FRONT_AND_BACK, 0.1f);
            }

            // Apply the material to the node.
            if( !use_color )
                stateSet->setAttribute(material, osg::StateAttribute::ON);

            if( !use_color )
                node->setStateSet(stateSet);

            return node;
        }

        void osg_scene_t::add_geometry_batch(const std::vector<geometry_info_t>& geoms)
        {

            //    PRX_ERROR_S(" \n\n\n\n**********\n ADD EXTRA BATCH \n************\n\n\n");

            foreach(geometry_info_t body, geoms)
            {
                //        PRX_INFO_S("I am in");
                osg::ref_ptr<osg::Group> parent = NULL;
                osg::ref_ptr<osg::PositionAttitudeTransform> child = NULL;

                if( info_geoms.find(body.full_name) != info_geoms.end() )
                {
                    //            PRX_LOG_DEBUG("Delete the geometry : %s", body.full_name.c_str());
                    // remove from the scene
                    parent = info_geoms[body.full_name]->getParent(0)->asGroup(); //asTransform()->asPositionAttitudeTransform();
                    parent->removeChild(info_geoms[body.full_name]);
                    if( geometry_material_mapping[body.full_name] != NULL )
                        delete geometry_material_mapping[body.full_name];
                }

                child = new osg::PositionAttitudeTransform();
                child->setName(body.full_name);

                PRX_INFO_S("Going to make the new geometry for the system : " << body.full_name << "    and type : " << body.type);
                //        PRX_DEBUG_S(" With color "); body.geometry->get_color().print();
                //        geometry_t new_geometry(body.type,&body.params);   
                //        PRX_DEBUG_S("From add geometry make geometry with material name " << body.material_name);
                osg::ref_ptr<osg::Node> geode = make_geometry(body.material_name, body.geometry, geometry_material_mapping[body.full_name], false, body.uses_geom_color);

                child->addChild(geode);
                info_geoms[body.full_name] = child;
                info_geoms_to_update.push_back(body.full_name);

                if( parent )
                    parent->addChild(child);
                else
                    root->addChild(child);
            }


        }

        void osg_scene_t::move_info_geometry(const std::string& name, const config_t& config)
        {

            PRX_ASSERT(info_geoms.find(name) != info_geoms.end());
            osg::PositionAttitudeTransform* current_node =
                    info_geoms[name]->asTransform()->asPositionAttitudeTransform();

            current_node->setAttitude(toOSGQuat(config.get_orientation()));
            current_node->setPosition(toVec3(config.get_position()));
        }

        void osg_scene_t::change_info_geom_color(const std::string& name, const util::vector_t& color)
        {
            PRX_INFO_S("Change info geom color for: " << name);

            PRX_ASSERT(info_geoms.find(name) != info_geoms.end());
            osg::Geode* geode = info_geoms[name]->getChild(0)->asGeode();
            PRX_ASSERT(geode != NULL);
            unsigned int numGeoms = geode->getNumDrawables();
            for( unsigned int geodeIdx = 0; geodeIdx < numGeoms; geodeIdx++ )
            {
                PRX_DEBUG_COLOR("Change geom nr: " << geodeIdx, PRX_TEXT_BLUE);
                osg::ShapeDrawable *curGeom = dynamic_cast<osg::ShapeDrawable*>(geode->getDrawable(geodeIdx));
                if( curGeom )
                {
                    curGeom->setColor(osg::Vec4(color[0], color[1], color[2], color[3]));
                    PRX_DEBUG_COLOR("Current geom valid", PRX_TEXT_LIGHTGRAY);
                }
                else
                {
                    PRX_DEBUG_COLOR("Curr geom NULL ", PRX_TEXT_GREEN);
                }

            }
            // If the node was using a material as its color, we need to disable it
            // and use the vertex colors instead.
            osg::StateSet *stateSet = geode->getOrCreateStateSet();
            //            geode->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::MATERIAL);
            osg::Material *material = dynamic_cast<osg::Material *>(stateSet->
                                                                    getAttribute(osg::StateAttribute::MATERIAL));
            if( material )
                material->setColorMode(osg::Material::DIFFUSE);
            //            stateSet->setAttributeAndModes(material, osg::StateAttribute::OFF);
        }

        void osg_scene_t::move_geometry(const std::string& name, const config_t& config)
        {

            PRX_ASSERT(rigid_bodies.find(name) != rigid_bodies.end());
            osg::PositionAttitudeTransform* current_node =
                    rigid_bodies[name]->asTransform()->asPositionAttitudeTransform();

            double quat[4];
            config.get_xyzw_orientation(quat);
            current_node->setAttitude(toOSGQuat(quat));
            current_node->setPosition(toVec3(config.get_position()));
        }

        void osg_scene_t::draw_text(const scene_text_t& text_info)
        {
            PRX_INFO_S("Rendering text on screen");
            std::string path = text_info.anchored_system + "/" + text_info.text_name;
            if( scene_text_map.find(path) == scene_text_map.end() )
            {
                PRX_WARN_S("Scene text name: " << path << " not found. Creating new text");
                osg::ref_ptr<osgText::Text> text_to_visualize = new osgText::Text();

                // Text related: font size, font, text

                text_to_visualize->setCharacterSize(text_info.font_size);
                if( !text_info.font.empty() )
                {
                    text_to_visualize->setFont(text_info.font);
                }
                else
                    text_to_visualize->setFont("fonts/fire_text/Blazed.ttf");
                text_to_visualize->setText(text_info.text);

                // Visualization Related:: 
                text_to_visualize->setDrawMode(osgText::Text::TEXT | osgText::Text::ALIGNMENT);
                text_to_visualize->setAlignment(osgText::Text::CENTER_TOP);
                text_to_visualize->setBackdropType(osgText::Text::OUTLINE);

                text_to_visualize->setPosition(toVec3(text_info.relative_position));
                //        text_to_visualize->setRotation( toVec4(config.get_orientation()));
                text_to_visualize->setColor(toVec4(text_info.color));

                osg::Geode* text_geode = new osg::Geode();
                text_geode->addDrawable(text_to_visualize);
                osg::ref_ptr<osg::PositionAttitudeTransform> linked_node;

                foreach(std::string name, info_geoms | boost::adaptors::map_keys)
                {
                    PRX_DEBUG_S("Info geom name: " << name);
                }

                if( obstacles.find(text_info.anchored_system) != obstacles.end() )
                {
                    linked_node = obstacles[text_info.anchored_system];
                }
                else if( info_geoms.find(text_info.anchored_system) != info_geoms.end() )
                {
                    linked_node = info_geoms[text_info.anchored_system];
                }
                else if( rigid_bodies.find(text_info.anchored_system) != rigid_bodies.end() )
                {
                    linked_node = rigid_bodies[text_info.anchored_system];
                }
                else
                {
                    linked_node = NULL;
                }
                if( linked_node != NULL )
                {
                    PRX_INFO_S("Anchoring text to: " << text_info.anchored_system);
                    text_to_visualize->setAxisAlignment(osgText::Text::SCREEN);
                    linked_node->addChild(text_geode);
                }
                else
                {
                    PRX_WARN_S("Could not anchor text. Visualizing to world coordinate");
                    root->addChild(text_geode);
                }
                scene_text_map[path] = text_to_visualize;
            }
            else
            {
                PRX_INFO_S("Updating text");
                scene_text_map[path]->setColor(toVec4(text_info.color));
                scene_text_map[path]->setText(text_info.text);
                //        scene_text_map[path]->setFont(text_info.font);
            }

        }

        void osg_scene_t::update_hud(hud_element_t element)
        {
            PRX_DEBUG_S("Updating hud element");
            // if we have the text previously
            osg_hud_t* current_hud = osg_hud_map[element.parent_name];
            osgText::Text* new_label;
            if( current_hud->hud_element_map.find(element.our_name) != current_hud->hud_element_map.end() )
            {
                new_label = current_hud->hud_element_map[element.our_name];
            }
            else
            {
                new_label = new osgText::Text();
                current_hud->hud_element_map[element.our_name] = new_label;

            }

            current_hud->HUDGeode->addDrawable(new_label);

            new_label->setCharacterSize(element.character_size);
            new_label->setFont(element.font);
            new_label->setText(element.text);
            new_label->setAxisAlignment(osgText::Text::SCREEN);
            new_label->setPosition(toVec3(element.position));
            new_label->setColor(toVec4(element.color));
        }

        void osg_scene_t::create_hud(hud_t* new_hud)
        {
            // HUD exists, update color
            if( hud_map.find(new_hud->name) != hud_map.end() )
            {
                PRX_INFO_S("HUD exists, update color");
                // update color
                osg::Vec4Array* HUDcolors = new osg::Vec4Array;
                HUDcolors->push_back(osg::Vec4(new_hud->color[0], new_hud->color[1], new_hud->color[2], new_hud->color[3]));
                osg_hud_map[new_hud->name]->HUDBackgroundGeometry->setColorArray(HUDcolors);

            }
            else
            {
                PRX_WARN_S("HUD does not exist, creating a new HUD");
                osg::Geode* HUDGeode = new osg::Geode();
                osg::Projection* HUDProjectionMatrix = new osg::Projection;

                HUDProjectionMatrix->setMatrix(osg::Matrix::ortho2D(0, 1000, 0, 1000));

                osg::MatrixTransform* HUDModelViewMatrix = new osg::MatrixTransform;
                HUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
                HUDModelViewMatrix->setMatrix(osg::Matrix::identity());

                root->addChild(HUDProjectionMatrix);
                HUDProjectionMatrix->addChild(HUDModelViewMatrix);
                HUDModelViewMatrix->addChild(HUDGeode);

                osg::Geometry* HUDBackgroundGeometry = new osg::Geometry();

                double x = new_hud->area[0], y = new_hud->area[1];
                double max_x = new_hud->area[2], max_y = new_hud->area[3];

                osg::Vec3Array* HUDBackgroundVertices = new osg::Vec3Array;
                HUDBackgroundVertices->push_back(osg::Vec3(x, y, .8));
                HUDBackgroundVertices->push_back(osg::Vec3(max_x, y, .8));
                HUDBackgroundVertices->push_back(osg::Vec3(max_x, max_y, .8));
                HUDBackgroundVertices->push_back(osg::Vec3(x, max_y, .8));

                osg::DrawElementsUInt* HUDBackgroundIndices =
                        new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
                HUDBackgroundIndices->push_back(0);
                HUDBackgroundIndices->push_back(1);
                HUDBackgroundIndices->push_back(2);
                HUDBackgroundIndices->push_back(3);

                osg::Vec4Array* HUDcolors = new osg::Vec4Array;
                HUDcolors->push_back(osg::Vec4(new_hud->color[0], new_hud->color[1], new_hud->color[2], new_hud->color[3]));

                osg::Vec2Array* texcoords = new osg::Vec2Array(4);
                (*texcoords)[0].set(0.0f, 0.0f);
                (*texcoords)[1].set(1.0f, 0.0f);
                (*texcoords)[2].set(1.0f, 1.0f);
                (*texcoords)[3].set(0.0f, 1.0f);
                HUDBackgroundGeometry->setTexCoordArray(0, texcoords);

                // set up the texture state.
                osg::Texture2D* HUDTexture = new osg::Texture2D;
                // protect from being optimized away as static state.
                HUDTexture->setDataVariance(osg::Object::DYNAMIC);
                HUDTexture->setImage(osgDB::readImageFile(new_hud->texture));

                osg::Vec3Array* HUDnormals = new osg::Vec3Array;
                HUDnormals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
                HUDBackgroundGeometry->setNormalArray(HUDnormals);
                HUDBackgroundGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

                HUDGeode->addDrawable(HUDBackgroundGeometry);
                HUDBackgroundGeometry->addPrimitiveSet(HUDBackgroundIndices);
                HUDBackgroundGeometry->setVertexArray(HUDBackgroundVertices);
                HUDBackgroundGeometry->setColorArray(HUDcolors);
                HUDBackgroundGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

                osg::StateSet* HUDStateSet = new osg::StateSet();
                HUDGeode->setStateSet(HUDStateSet);
                HUDStateSet->setRenderBinDetails(11, "DepthSortedBin");
                HUDStateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
                HUDStateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
                HUDStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                HUDStateSet->setTextureAttributeAndModes(0, HUDTexture, osg::StateAttribute::ON);
                //osgUtil::RenderBin* HUDBin =
                //  new osgUtil::RenderBin(osgUtil::RenderBin::SORT_BACK_TO_FRONT);

                hud_map[new_hud->name] = new_hud;

                osg_hud_map[new_hud->name] = new osg_hud_t(HUDGeode, HUDProjectionMatrix,
                                                           HUDModelViewMatrix, HUDBackgroundGeometry, HUDTexture);

            }
        }

        void osg_scene_t::init(const parameter_reader_t* reader)
        {
            root = new osg::Group;
            line_thickness = 2.0;

            // Temporarily store materials as they are created.
            // Later, they will be pushed into a geometry-name -> material map.

            foreach(const parameter_reader_t::reader_map_t::value_type key_value, reader->get_map("materials"))
            {
                const std::string& name = key_value.first;
                const parameter_reader_t* m_reader = key_value.second;
                osg_material_t* material = new osg_material_t();
                material->init(m_reader);
                template_material_mapping[name] = material;
                delete m_reader;
            }

            if( reader->has_attribute("line_thickness") )
            {
                line_thickness = reader->get_attribute_as<double>("line_thickness");
            }

            // const parameter_reader_t root_reader("");

            if( global_reader->has_element("") )
            {
                PRX_INFO_S("--Checking for systems--");

                foreach(const parameter_reader_t::reader_map_t::value_type system_key_value, global_reader->get_map("systems"))
                {

                    std::vector<const parameter_reader_t*> readers = system_key_value.second->get_list("geometries");
                    std::string geom_name;

                    foreach(const parameter_reader_t* r, readers)
                    {
                        geom_name = r->get_attribute_as< std::string > ("name");
                        PRX_DEBUG_S("The geom name is : " << geom_name);
                        std::string name = "system/" + system_key_value.first + "/" + geom_name;
                        osg::ref_ptr<osg::PositionAttitudeTransform> parent = new osg::PositionAttitudeTransform();
                        init_geometries("system/" + system_key_value.first, geom_name, r, parent);
                        rigid_bodies[name] = parent;
                    }
                    //            foreach(const parameter_reader_t::reader_map_t::value_type& body_key_value, system_key_value.second->get_map("rigid_bodies")) 
                    //                init_geometries(system_key_value.first + "/" + body_key_value.first, body_key_value.second);
                }
            }

            foreach(const parameter_reader_t* t_reader, reader->get_list("texture"))
            {
                PRX_DEBUG_S("Adding texture.");
                osg_texture_t texture;
                texture.init(t_reader);
                if( texture.is_selection_plane )
                {
                    selection_plane = new osg::PositionAttitudeTransform();
                    selection_plane->addChild(texture.get_terrain());
                    root->addChild(selection_plane);
                }
                else
                    root->addChild(texture.get_terrain());
                delete t_reader;
            }

            unsigned int lightNum = 1;

            foreach(const parameter_reader_t::reader_map_t::value_type light_key, reader->get_map("lights"))
            {
                PRX_DEBUG_S("Read in light!");
                osg_light_t light(lightNum);
                light.init(light_key.second);
                add_light(light);
                ++lightNum;
            }
            PRX_INFO_S("Created scene.");
        }

        void osg_scene_t::init_geometries(const std::string& system, const std::string& body, const parameter_reader_t* body_reader, osg::PositionAttitudeTransform* parent, osg::Group* group)
        {
            std::string name = system + "/" + body;

            parent->setName(name);
            geometry_t collision_geometry, visualization_geometry;

            std::string collision_material_name = body_reader->get_attribute("collision_geometry/material");
            PRX_DEBUG_COLOR("Collision material name: " << collision_material_name, PRX_TEXT_MAGENTA);
            body_reader->initialize(&collision_geometry, "collision_geometry");
            osg::ref_ptr<osg::Node> collision_geode = make_geometry(collision_material_name, &collision_geometry, geometry_material_mapping[name], false);


            if( body_reader->has_element("visualization_geometry") )
            {
                std::string visualization_material_name;
                osg::ref_ptr<osg::Node> visualization_geode;

                osg::ref_ptr<osg::Switch> switch_node = new osg::Switch();
                osg::ref_ptr<osg::Group> combined = new osg::Group();
                osg::ref_ptr<osg::PositionAttitudeTransform> visualization_transform = new osg::PositionAttitudeTransform();

                /////////////////////////
                if( body_reader->get_attribute("visualization_geometry/type") == "mesh" )
                {
                    std::string osg_visualization_geom_name = body_reader->get_attribute("visualization_geometry/filename");
                    osg::ref_ptr<osg::Node> osg_visualization_geode = osgDB::readNodeFile(osg_visualization_geom_name);

                    PRX_DEBUG_S("Found filename: " << osg_visualization_geom_name);
                    if( osg_visualization_geode != NULL )
                    {
                        visualization_geode = osg_visualization_geode;
                    }
                    else
                    {
                        visualization_material_name = body_reader->get_attribute("visualization_geometry/material");
                        body_reader->initialize(&visualization_geometry, "visualization_geometry");
                        visualization_geode = make_geometry(visualization_material_name, &visualization_geometry, geometry_material_mapping[name + "_vis"], false);
                    }
                }
                else
                {
                    visualization_material_name = body_reader->get_attribute("visualization_geometry/material");
                    body_reader->initialize(&visualization_geometry, "visualization_geometry");
                    visualization_geode = make_geometry(visualization_material_name, &visualization_geometry, geometry_material_mapping[name + "_vis"], false);
                }

                if( body_reader->has_attribute("visualization_geometry/local_transform") )
                {
                    config_t local_transform = ((*body_reader->initialize_new<config_t > (std::string("visualization_geometry/local_transform")))); //)body_reader->get_attribute_as<config_t>("visualization_geometry/local_transform");
                    visualization_transform->setPosition(toVec3(local_transform.get_position()));
                    visualization_transform->setAttitude(toVec4(local_transform.get_orientation()));
                }
                else
                {
                    visualization_transform->setPosition(osg::Vec3(0.0, 0.0, 0.0));
                    visualization_transform->setAttitude(osg::Vec4(0.0, 0.0, 0.0, 1.0));
                }
                visualization_transform->addChild(visualization_geode);

                combined->addChild(visualization_transform);
                combined->addChild(make_geometry(collision_material_name, &collision_geometry, geometry_material_mapping[name + "_duo"], true));

                switch_node->addChild(visualization_transform);
                switch_node->addChild(collision_geode);
                switch_node->addChild(combined);

                switch_node->setAllChildrenOff();
                switch_node->setChildValue(visualization_transform, true);

                parent->addChild(switch_node);
            }
            else
            {
                parent->addChild(collision_geode);

            }
            if( group != NULL )
                group->addChild(parent);
            else
                root->addChild(parent);
        }

        void osg_scene_t::initialize_plants(const std::string& source_node, const std::vector<std::string>& paths, const std::vector<std::string>& template_paths)
        {
            PRX_INFO_S("Initializing plants");

            for( unsigned int i = 0; i < paths.size(); i++ )
            {
                std::string name, subpath, current_path;
                //        PRX_LOG_WARNING ("Got path: %s", paths[i].c_str());

                if( template_paths.empty() )
                {
                    boost::tie(name, subpath) = split_path(paths[i]);
                    while( !subpath.empty() )
                    {
                        if( name.empty() )
                        {
                            PRX_ERROR_S("Name is empty");
                        }
                        current_path += name + "/subsystems/";
                        PRX_DEBUG_S("Current path: " << current_path.c_str());
                        boost::tie(name, subpath) = split_path(subpath);
                    }
                    current_path += name;
                }
                else
                {
                    current_path = template_paths[i];
                }

                std::string final_path;

                final_path = source_node + "/" + current_path;

                PRX_DEBUG_S("Final path: " << final_path.c_str());
                const parameter_reader_t root_reader(final_path);

                const parameter_reader_t* template_reader = NULL;
                if( root_reader.has_attribute("template") )
                {
                    std::string template_namespace = root_reader.get_attribute("template");
                    template_reader = new parameter_reader_t(source_node + "/" + template_namespace);
                }

                if( root_reader.has_attribute("geometries") )
                {
                    std::vector<const parameter_reader_t*> readers = root_reader.get_list("geometries");
                    std::string geom_name;
                    osg::ref_ptr< osg::Group > group = new osg::Group();
                    systems[paths[i]] = group;

                    foreach(const parameter_reader_t* r, readers)
                    {
                        geom_name = r->get_attribute_as< std::string > ("name");
                        PRX_DEBUG_S("The geom name is : " << paths[i] << "/" << geom_name);
                        std::string name = paths[i] + "/" + geom_name;
                        osg::ref_ptr<osg::PositionAttitudeTransform> parent = new osg::PositionAttitudeTransform();
                        init_geometries(paths[i], geom_name, r, parent, group);
                        rigid_bodies[name] = parent;
                    }

                    root->addChild(group);
                    template_systems[paths[i]] = group; //new osg::Group(*group, osg::CopyOp::DEEP_COPY_ALL);
                }
                else if( template_reader != NULL )
                {
                    std::vector<const parameter_reader_t*> readers = template_reader->get_list("geometries");
                    std::string geom_name;
                    osg::ref_ptr< osg::Group > group = new osg::Group();
                    systems[paths[i]] = group;

                    foreach(const parameter_reader_t* r, readers)
                    {
                        geom_name = r->get_attribute_as< std::string > ("name");
                        PRX_DEBUG_S("The geom name is : " << paths[i] << "/" << geom_name);
                        std::string name = paths[i] + "/" + geom_name;
                        osg::ref_ptr<osg::PositionAttitudeTransform> parent = new osg::PositionAttitudeTransform();
                        init_geometries(paths[i], geom_name, r, parent, group);
                        rigid_bodies[name] = parent;
                    }

                    root->addChild(group);

                    template_systems[paths[i]] = group; //new osg::Group(*group, osg::CopyOp::DEEP_COPY_ALL);
                }
                else
                {
                    PRX_FATAL_S("No geometries specified for plant!");
                }
            }
            PRX_DEBUG_S("Finished initializing plants");
            //            osgUtil::Optimizer t;
            //            t.optimize(root, t.ALL_OPTIMIZATIONS);
        }

        void osg_scene_t::add_ghost_plant(const std::string& path, util::config_t rel_conf)
        {
            if( template_systems.find(path) != template_systems.end() )
            {
                //                PRX_DEBUG_COLOR ("Found plant path: " << path, PRX_TEXT_BLUE);
                osg::ref_ptr< osg::Group > root_plant = template_systems[path]->asGroup();
                if( root_plant != NULL )
                {
                    osg::ref_ptr< osg::Group > copy_plant = new osg::Group(*root_plant, osg::CopyOp::DEEP_COPY_ALL);
                    PRX_DEBUG_S(fromVec3(copy_plant->getChild(0)->asTransform()->asPositionAttitudeTransform()->getPosition()));
                    osg::ref_ptr< osg::PositionAttitudeTransform > ghost_transform = new osg::PositionAttitudeTransform();
                    ghost_transform->setName(path + "_ghost");
                    ghost_transform->setAttitude(toOSGQuat(rel_conf.get_orientation()));
                    ghost_transform->setPosition(toVec3(rel_conf.get_position()));
                    if( rigid_bodies.find(path) != rigid_bodies.end() )
                    {
                        PRX_INFO_S("Found ghost template in rigid bodies map");
                    }
                    //                    ghost_transform->addChild(copy_plant);
                    //                    std::vector<osg::ref_ptr< osg::Geode > > get_geodes;
                    //                    FindAllGeodes vis(&get_geodes);
                    //                    copy_plant->accept(vis);
                    //                    for (unsigned i = 0; i < get_geodes.size(); i++)
                    //                    {
                    //                        PRX_PRINT ("FOUND GEOEDE!", PRX_TEXT_CYAN);
                    //                        osg::Material* material = (osg::Material *) get_geodes[i]->getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
                    //                        get_geodes[i]->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
                    //                        if (material == NULL)
                    //                        {
                    //                            material = new osg::Material();
                    //                        }
                    //  
                    //                        material->setAlpha(osg::Material::FRONT_AND_BACK, 0.1);
                    //                        material->setTransparency(osg::Material::FRONT, 0.6);
                    //                        get_geodes[i]->getStateSet()->setAttributeAndModes(material, osg::StateAttribute::OVERRIDE);
                    //                        get_geodes[i]->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 
                    //
                    //                    }

                    osg::StateSet *state = ghost_transform->getOrCreateStateSet();
                    state->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
                    state->setAttribute(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
                    osg::Material* material = new osg::Material;
                    material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.2f, 0.2f, 0.2f, 0.3f));
                    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8f, 0.8f, 0.8f, 0.3f));
                    state->setAttribute(material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
                    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                    osg::LightModel *lm = new osg::LightModel();
                    lm->setTwoSided(true);
                    state->setAttribute(lm, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
                    (ghost_transform.get())->addChild(copy_plant.get());

                    root->addChild(ghost_transform);
                }
            }
            else
            {
                PRX_ERROR_S("COULD NOT FIND plant path: " << path);
            }
        }

        void osg_scene_t::remove_plant(const std::string& path)
        {
            // TODO: Figure out how to remove material mappings for plants
            if( systems.find(path) != systems.end() )
            {
                if( root->containsNode(systems[path]) )
                {
                    root->removeChild(systems[path]);
                    systems.erase(path);
                }
                else
                {
                    PRX_FATAL_S("Visualization root node does not contain " << path);
                }
            }
            else
            {
                PRX_WARN_S("Attempted to remove path that has not been sent to visualization: " << path);
            }

        }

        void osg_scene_t::visualize_plant(const std::string& path, const int flag)
        {
            if( systems.find(path) != systems.end() )
            {
                switch( flag )
                {
                    case PRX_VIS_TEMPORARY_REMOVE:
                    {
                        //                PRX_ERROR_S("Temporarily removing: " << path);
                        if( root->containsNode(systems[path]) )
                        {
                            if( systems[path]->asSwitch() )
                            {
                                systems[path]->asSwitch()->setAllChildrenOff();
                            }
                        }
                        break;
                    }
                    case PRX_VIS_TEMPORARY_SHOW:
                    {
                        //                PRX_ERROR_S("Temporarily showing: " << path);
                        if( root->containsNode(systems[path]) )
                        {
                            if( systems[path]->asSwitch() )
                            {
                                systems[path]->asSwitch()->setAllChildrenOn();
                            }
                        }
                        break;
                    }
                    case PRX_VIS_REMOVE:
                    {
                        //                PRX_ERROR_S ("Removing " << path );
                        if( root->containsNode(systems[path]) )
                        {
                            root->removeChild(systems[path]);
                        }
                        else
                        {
                            PRX_WARN_S("Removing plant which is already removed");
                        }
                        break;
                    }
                    case PRX_VIS_SHOW:
                    {
                        //                PRX_ERROR_S ("Showing " << path );
                        if( !root->containsNode(systems[path]) )
                        {
                            root->addChild(systems[path]);
                        }
                        else
                        {
                            PRX_WARN_S("Removing plant which is already removed");
                        }
                        break;
                    }
                    default:
                        break;
                }

            }
            else
            {
                PRX_WARN_S("Attempted to visualize plant that has not been sent to visualization: " << path);
            }
        }
        
        void osg_scene_t::visualize_obstacles(const std::string& obstacles_path)
        {

            // TODO: If a / is in the beginning of the pathname, this does not work.
            PRX_DEBUG_S ("Looking for obstacles at: " << obstacles_path << "/obstacles");
            if( global_reader->has_element(obstacles_path + "/obstacles") )
            {

                PRX_DEBUG_S("Found obstacles in " << obstacles_path << "/obstacles");

                foreach(const parameter_reader_t::reader_map_t::value_type& body_key_value, global_reader->get_map(obstacles_path + "/obstacles"))
                {
                    std::vector<const parameter_reader_t*> readers = body_key_value.second->get_list("geometries");
                    std::string geom_name;

                    foreach(const parameter_reader_t* r, readers)
                    {
                        geom_name = r->get_attribute_as< std::string > ("name");
                        PRX_DEBUG_S("The geom name is : " << geom_name);
                        std::string name = obstacles_path + "/obstacles/" + body_key_value.first + "/" + geom_name;
                        osg::ref_ptr<osg::PositionAttitudeTransform> parent = new osg::PositionAttitudeTransform();
                        init_geometries(obstacles_path + "/obstacles/" + body_key_value.first, geom_name, r, parent);
                        config_t conf;
                        conf.init(r->get_child("config").get());
                        parent->setAttitude(toOSGQuat(conf.get_orientation()));
                        parent->setPosition(toVec3(conf.get_position()));
                        obstacles[name] = parent;
                    }

                }
            }
            else
            {
                PRX_ERROR_S ("No obstacles found at: " << global_reader->trace() + obstacles_path + "/obstacles");
            }
        }

        void osg_scene_t::visualize_ghost_plants(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& ghost_configs)
        {
            for( unsigned i = 0; i < plant_paths.size(); i++ )
            {
                add_ghost_plant(plant_paths[i], ghost_configs[i]);
            }
        }

        void osg_scene_t::switch_scene(osg_ghost_switch_t::scene_mode_t mode)
        {
            scene_switch.switch_mode(mode);
        }

    }
}
