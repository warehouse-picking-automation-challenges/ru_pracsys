/**
 * @file camera.hpp 
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


#ifndef PRACSYS_CAMERA_HPP
#define PRACSYS_CAMERA_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"

#define PRX_CAMERA_EYE     0
#define PRX_CAMERA_CENTER  1
#define PRX_CAMERA_UP      2

#define INIT_CAM_SPEED 3.0
#define INIT_CAM_ROTATE .005

namespace prx 
{ 
    namespace vis 
    {

/**
 * This class represents a camera within a virtual scene on an abstract level.
 * @brief <b> Camera representation abstract class  </b>
 *
 * @authors Andrew Kimmel
 */
class camera_t
{
protected:
    /** @brief The location of the 'eye' (camera)*/
    util::vector_t eye;         
    /** @brief The point the camera is looking at */
    util::vector_t center;
    /** @brief Represents which direction is 'up' for the camera */
    util::vector_t up;          
    /** @brief Represents the direction the camera is looking at*/
    util::vector_t in;          
    /** @brief Represents the sideways direction for the camera */
    util::vector_t cross;       
    /** @brief The translational moving speed of the camera*/
    double camera_speed;
    /** @brief The rotational moving speed of the camera */
    double camera_rotate;           
    /** @brief Whether or not the camera is following a target */
    bool follower;                  
    /** @brief Whether or not the camera is orthographic */
    bool ortho;                     
    /** @brief Used for resetting the camera view to initial positions*/
    bool initialized;               
    /** @brief Distinguishes the type of camera */
    int camera_type;             

    /** @brief Used for orthographic camera creation */
    double left, right, bottom, top, zNear, zFar; // respectively, min x, max x, min y, max y, z near, and z far for orthographic camera
    /** @brief Original camera speed and rotation*/
    double init_cam_speed, init_cam_rotate;
    /** @brief Original camera vectors */
    util::vector_t init_eye, init_center, init_up;

public:
    camera_t() : eye(3), center(3), up(3), in(3), cross(3), init_eye(3), init_center(3), init_up(3) {}
    virtual ~camera_t() {}

    /**
     * Retrieves either the eye, center, or up depending on the index.
     * 
     * The vector is returned by reference in \c vec.
     * 
     * @brief Retrieves either the eye, center, or up depending on the index
     * @param index Determines which camera vector to return
     * @param vec The returned vector
     */
    virtual void get_camera_vector( int index, util::vector_t& vec ) const = 0;
    
    /**
     * Sets either the eye, center, or up depending on the index.
     * 
     * @brief Sets either the eye, center, or up depending on the index.
     * @param index Determines which camera vector to return
     * @param new_vec Used to set the camera vector
     */
    virtual void set_camera_vector( int index, const util::vector_t& new_vec ) = 0;

    /**
     * Computes and sets the center rotational horizontal position given the rotation value.
     *
     * @brief Sets center horizontal rotational position.
     *
     * @param rotation The rotational value.
     */
    virtual void horizontal_rotation( double rot ) = 0;
    
    /**
     * Computes and sets the center rotational vertical position given the rotation value.
     *
     * @brief Sets center vertical rotational position.
     *
     * @param rotation The rotational value.
     */
    virtual void vertical_rotation( double rot ) = 0;
    
    /**
     * Recomputes the eye and center vectors in order to move the camera up
     * along the Y coordinate, not relative to the camera's view.
     *
     * @brief Moves the camera up in the Y direction
     */
    virtual void move_up( ) = 0;

    /**
     * Recomputes the eye and center vectors in order to move the camera down
     * along the Y coordinate, not relative to the camera's view.
     *
     * @brief Moves the camera down in the Y direction
     */
    virtual void move_down( ) = 0;

    /**
     * Computes and sets the in/out movement of the camera position given the speed value.
     *
     * @brief Sets camera in/out(forward/back) position.
     *
     * @param forward A flag indicating whether to move in or out.
     */
    virtual void set_move_pos_in( bool forward ) = 0;
    
    /**
     * Computes and sets the sideways movement of the camera position given the speed value.
     * 
     * Setting \c forward "true" moves the camera to the right, false moves it to the left. 
     *
     * @brief Sets camera sideways position.
     *
     * @param forward A flag indicating whether to move left or right.
     */
    virtual void set_move_pos_side( bool forward ) = 0;

    /**
     * Resets the camera's position and speed to default values
     *
     * @brief Reset camera parameters
     *
     */
    virtual void reset( ) = 0; // puts the camera back into its original position

    /**
     * Speeds up the speed of the camera's translation speed
     *
     * @brief Makes the camera move faster
     *
     */
    virtual void speed_up( ) = 0;
    
    /**
     * Slows down the speed of the camera's translation speed
     *
     * @brief Makes the camera move slower
     *
     */
    virtual void speed_down( ) = 0;

    // checks if the camera is orthographic

    /**
     * Checks if the camera is orthographic
     *
     * @brief True if the camera is orthographic, false otherwise
     *
     */
    virtual bool is_ortho( ) const = 0;
    
    /**
     * Sets a camera's orthographic mode
     *
     * @brief Sets \c ortho
     *
     */
    virtual void set_ortho( bool orthographic ) = 0;
    
    /**
     * Checks if the camera is following an object
     *
     * @brief True if the camera is following, false otherwise
     *
     */
    virtual bool is_follower( ) const = 0;
    
    /**
     * Toggles the camera's following mode
     * 
     * @brief Toggles the camera's following mode
     */
    virtual void toggle_follow( ) = 0;

    /**
     * Returns if the camera has its own default initial values
     *
     * @brief Returns initialized
     *
     */
    virtual bool is_initialized() const = 0;
    
    /**
     * Sets the camera's \c initialized variable
     * 
     * @brief Sets the camera's \c initialized variable
     * @param init Used to set \c initialized
     */
    virtual void set_initialized( bool init ) = 0;
    
    /**
     * @brief Retrieves the type of the camera
     * @return Represents what kind of camera this is
     */
    virtual int get_type() const = 0;

    /**
     * This function outputs information on the data members of the class for
     * debugging purposes.
     *
     * @brief Camera class debug printing function
     */
    virtual std::string print() = 0;
};

    }
 }

#endif
