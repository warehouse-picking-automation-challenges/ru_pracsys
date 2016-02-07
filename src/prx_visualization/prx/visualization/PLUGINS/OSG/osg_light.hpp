/**
 * @file osg_light.hpp 
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

#ifndef PRACSYS_OSG_LIGHT_HPP
#define PRACSYS_OSG_LIGHT_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace vis
    {

        /**
         * Wraps the OSG light class.
         * 
         * @brief <b> Wrapper class for OSG light </b>
         *
         * @authors Andrew Kimmel
         */
        class osg_light_t
        {

          public:
            /**
             * Constructor which sets the light's identifier
             * 
             * @brief Constructor
             * @param num The integer identifier for the light
             */
            osg_light_t(unsigned int num);

            /**
             * Uses a vector to set the position of the light
             * 
             * @brief Sets the position of the light
             * @param pos The position of the light
             */
            void set_pos(const util::vector_t& pos);

            /**
             * Sets where the light will be pointing
             * 
             * @brief Sets the direction of the light
             * @param dir The direction of the light
             */
            void set_dir(const util::vector_t& dir);

            /**
             * Set the ambient value of the light
             * 
             * @brief Set the ambient value of the light
             * @param rgba The ambient value of the light
             */
            void set_ambient(const util::vector_t& rgba);

            /**
             * Set the diffuse value of the light
             * 
             * @brief Set the diffuse value of the light
             * @param rgba The diffuse value of the light
             */
            void set_diffuse(const util::vector_t& rgba);

            /**
             * Set the specular value of the light
             * 
             * @brief Set the specular value of the light
             * @param rgba The specular value of the light
             */
            void set_specular(const util::vector_t& rgba);

            /**
             * Sets the cutoff distance of the Spotlight. 
             *
             * @brief Sets the cutoff distance of the Spotlight. 
             * @param value The cutoff value for the Spotlight.
             */
            void set_spot_cutoff(double cutoff);

            /**
             * Return the Light.
             * 
             * @brief Return the Light.
             *
             * @remarks This funciton is called by the Scene class.
             *
             * @return OSG node of the Light.
             */
            osg::ref_ptr<osg::Light> get_wrapped_light();

            /**
             * Initializes the light with direction, position, ambient,
             * diffuse, and specular values from input.
             * 
             * @brief Initializes the light
             * @param reader Parameter reader
             */
            void init(const util::parameter_reader_t* reader);

          private:
            /** @brief The OSG Light */
            osg::ref_ptr<osg::Light> light;

        };

    }
}

#endif
