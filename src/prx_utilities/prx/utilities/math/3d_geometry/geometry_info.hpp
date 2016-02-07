/**
 * @file geometry_info.hpp 
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

#ifndef PRX_GEOMETRY_INFO_HPP
#define	PRX_GEOMETRY_INFO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/utilities/math/configurations/vector.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        /**
         * A container class which holds information of geometries to be visualized with.
         * 
         * This class is typically used to aid the creation of geometry messages, which
         * in turn are used to visualize geometries in the visualization node.
         * 
         * @brief <b> A container class which holds information of geometries to be visualized with.  </b>
         *
         * @authors Andrew Kimmel
         */
        
        struct geometry_info_t
        {
            
            /** @brief The system's name, for example "car" */
            std::string system_name;
            
            /** @brief The rigid body's name, for example "chassis" */
            std::string rigid_body_name;
            
            /** @brief A unique identifier for this geometry. Heirarchical trees of groups and leaves may be specified with slashes, For example "car/chassis". */
            std::string full_name;
            
            /** @brief The type of the geometry for the system. */
            geometry_type type;
            
            /** @brief The parameters that the specific geometry needs */
            std::vector<double> params;
            
            /** @brief Optional information about the shape of the geometry. If this is null then there is none available.*/
            geometry_t* geometry; 
            
            /** @brief Optional name for a plugin-specific model file. If the string is empty then a file name is not available. */
            std::string mesh_filename;
            
            /** @brief The name of geometry's material, such as "red" */
            std::string material_name;
            
            /** @brief The numerical value of the geometry's color */
            vector_t color;
            
            /** @brief Specifies whether the geometry color should be used rather than the material */
            bool uses_geom_color;
            
            geometry_info_t() : geometry(NULL){}
            
            geometry_info_t(const std::string& sys_name, std::string rb_name, geometry_type gtype, const std::vector<double>& info, const std::string& material): geometry(NULL)
            {
                system_name = sys_name;
                rigid_body_name = rb_name;
                full_name = sys_name + "/" + rigid_body_name;
                type = gtype;
                params = info;
                material_name = material;
                uses_geom_color = false;
                //        if (uses_geom_color)
                //            PRX_LOG_WARNING ("Will use geometry color");
                //        else
                //            PRX_LOG_WARNING ("Will not use geometry color");
            }
            
            geometry_info_t(const std::string& sys_name, std::string rb_name, geometry_type gtype, const std::vector<double>& info, const vector_t& other_color): geometry(NULL)
            {
                system_name = sys_name;
                rigid_body_name = rb_name;
                full_name = sys_name + "/" + rigid_body_name;
                type = gtype;
                params = info;
                material_name = "white";
                uses_geom_color = true;
                color = other_color;
                if (color.get_dim() == 3)
                {
                    color.resize(4);
                    color[3] = 1.0;
                }
                //        if (uses_geom_color)
                //            PRX_LOG_WARNING ("Will use geometry color");
                //        else
                //            PRX_LOG_WARNING ("Will not use geometry color");
            }

            std::string print( )
            {
                std::stringstream out(std::stringstream::out);
                out << "= Geometry Print =\nFull name: " << full_name << "\n";
                out << "Material name: " << material_name << "\nType: " << type << "\n";
                out << "Variable params: ";
                foreach(double d, params)
                {
                    out << d << ", ";
                }
                out << "\n";
                return out.str();
            }

        };
        
        
        /**
         * A structure that maps a unique rigid body name to its geometry info.
         *
         * A rigid body name should be its system's name with a rigid body name.
         * For example: \c car1/chassis
         */    
        typedef hash_t<std::string, geometry_info_t> geom_info_map_t;
        
    } 
}

#endif	// PRX_GEOMETRY_INFO_HPP

