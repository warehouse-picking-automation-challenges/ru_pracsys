/**
 * @file scene_text.hpp 
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

#ifndef PRX_SCENE_TEXT_HPP
#define	PRX_SCENE_TEXT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx 
{ 
    namespace vis 
    {
        
/**
 * A container class to hold the necessary information to visualize
 * text inside the scene.
 * 
 * @brief <b> Container class for scene text </b>
 *
 * @authors Andrew Kimmel
 */
struct scene_text_t
{
    scene_text_t(std::string new_text_name, std::string system_name, std::string new_text, std::string font_location, std::vector<double> new_position, std::vector<double> new_color, double size)
    : text_name(new_text_name), anchored_system(system_name), text(new_text), font(font_location), relative_position(new_position), color(new_color), font_size(size) {}

    /** @brief The name to associate with the text (for mapping purposes) */
    std::string text_name;
    /** @brief The pathname of the system to anchor the text to*/
    std::string anchored_system;
    /** @brief The actualy text to visualize*/
    std::string text;
    /** @brief The filename of the font to use */
    std::string font;
    /** @brief Relative position of the text to the anchored system (absolute position if there is no anchor)*/
    std::vector<double> relative_position;
    /** @brief Color of the text */
    std::vector<double> color;
    /** @brief Font size of the text */
    double font_size;
};

    }
 }


#endif	/* PRX_SCENE_TEXT_HPP */

