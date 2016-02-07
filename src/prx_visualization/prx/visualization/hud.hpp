#pragma once
/**
 * @file hud.hpp 
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


#ifndef PRX_HUD_HPP
#define	PRX_HUD_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx 
{ 
    namespace vis 
    {

/**
 * Container class for visualizing an element of a Heads Up Display (HUD).  A HUD consists
 * of 1 or more elements.
 * 
 * @brief <b> Container class for visualizing a HUD element </b>
 *
 * @authors Andrew Kimmel
 */
struct hud_element_t
{
public:
    /** @brief Position of the HUD*/
    util::vector_t position;
    /** @brief Clear color of the HUD */
    util::vector_t color;
    /** @brief Font size */
    double character_size;
    /** @brief Filename of font*/
    std::string font;
    /** @brief Text to visualize for HUD*/
    std::string text;
    /** @brief Name of Parent HUD */
    std::string parent_name;
    /** @brief Name of current HUD */
    std::string our_name;
};

/**
 * Container class for visualizing a HUD area
 * 
 * @brief <b> Container class for visualizing a HUD area </b>
 *
 * @authors Andrew Kimmel
 */
class hud_t
{
public:

    hud_t(std::string new_name, std::vector<double> new_area, std::vector<double> new_color, std::string new_texture)
    : name(new_name), area(new_area), color(new_color), texture(new_texture) {}
    ~hud_t();

    /** @brief The name of the HUD */
    std::string name;
    /** @brief The area of the screen occupied by the HUD */
    std::vector<double> area; // x, y, width, height
    /** @brief Clear color of the HUD */
    std::vector<double> color; // r, g, b , a
    /** @brief Filename of the texture to load into the HUD */
    std::string texture;

};

    }
 }

#endif

