/**
 * @file osg_camera.cpp 
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

#include <osgGA/UFOManipulator>

#include "prx/visualization/PLUGINS/OSG/osg_camera.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <osgGA/UFOManipulator>
#include <osgGA/TrackballManipulator>

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

osg_camera_t::osg_camera_t()
{
    reset_view();
}

osg_camera_t::~osg_camera_t()
{
    
}
void osg_camera_t::reset_view()
{
    if (ortho)
    {
        eye =  vector_t(0.0, 0.0, 20.0);;
        center = vector_t (0.0,0.0,0.0);
    }
    else
    {
        eye = vector_t(0.0,0.0,-130.0);
        center = vector_t(0.0,.0,0.0);
    }

    vector_t diff = center - eye;
    vector_t x_vec (1.0,0.0,0.0);

    up.cross_product(diff, x_vec);
    //up = vector_t (0.0,1.0,0.0);

    camera_speed  = 3.0;
    camera_rotate = 0.001;
}
void osg_camera_t::set_view ( osg::View* current_view, osg::Group* selectedNode, double ratio )
{

    if( follower && !ortho && selectedNode )
    {
        osg::Vec3 node_position = selectedNode->getBound().center();
        osg::Vec3 diff_vec = toVec3(eye - center);
        osg::Vec3 new_eye = node_position + diff_vec;


//        printf ("Current camera values during follow mode\n");
//        eye.print();
//        center.print();
//        up.print();
        current_view->getCamera()->setViewMatrixAsLookAt ( new_eye, node_position, toVec3(up));

        eye = fromVec3(new_eye);

        center = fromVec3(node_position);

    }
    else 
    {
        if ( ortho )
            current_view->getCamera()->setProjectionMatrixAsOrtho( right, left, top, bottom, zNear, zFar);
        else
            current_view->getCamera()->setProjectionMatrixAsPerspective( 75.0, ratio, 0.1, 10000 );

//        printf ("Current camera values\n");
//        eye.print();
//        center.print();
//        up.print();
        current_view->getCamera()->setViewMatrixAsLookAt( toVec3(eye), toVec3(center), toVec3(up) );
    }
}


std::string osg_camera_t::print()
{
    std::stringstream out(std::stringstream::out);
    out << "Camera::" << std::endl << "Eye: " << eye << " Center: " << center << " Up: " << up << std::endl;
    return out.str();
}


void osg_camera_t::get_camera_vector( int index, vector_t& vec ) const
{
    if( index == PRX_CAMERA_EYE )
	vec = eye;
    else if( index == PRX_CAMERA_CENTER )
	vec = center;
    else
	vec = up;
}

// stub
void osg_camera_t::set_camera_vector ( int index, const vector_t& new_vec )
{
    if( index == PRX_CAMERA_EYE )
	eye = new_vec;
    else if( index == PRX_CAMERA_CENTER )
	center = new_vec;
    else
	up = new_vec;
}


void osg_camera_t::horizontal_rotation( double rotation )
{
    rotation *= camera_rotate;

    double D     = sqrt(  pow(center[0]-eye[0],2) + pow(center[1]-eye[1],2) );
    double theta = atan2( center[0]-eye[0], center[1]-eye[1] );
    double roll  = atan2( center[2]-eye[2], D );
    double dist  = sqrt(  pow(center[0]-eye[0],2) + pow(center[2]-eye[2],2) + pow(center[1]-eye[1],2) );

    theta -= rotation;

    center[2] =  eye[2] + dist * sin(roll) ;
    D = dist * cos(roll);
    center[0] =  eye[0] + D*sin(theta);
    center[1] =  eye[1] + D*cos(theta) ;
}

void osg_camera_t::vertical_rotation(double rotation)
{
//    if (camera_type != 2)
    {
        rotation *= camera_rotate;

        double D     = sqrt(  pow(center[0]-eye[0],2) + pow(center[1]-eye[1],2) );
        double theta = atan2( center[0]-eye[0], center[1]-eye[1] );
        double roll  = atan2( center[2]-eye[2], D );
        double dist  = sqrt(  pow(center[0]-eye[0],2) + pow(center[2]-eye[2],2) + pow(center[1]-eye[1],2) );

        roll -= rotation;

        center[2] = eye[2] + dist * sin(roll);
        D = dist * cos(roll);
        center[0] = eye[0] + D*sin(theta);
        center[1] = eye[1] + D*cos(theta);
    }
//    else
//    {
////        if (rotation < 0)
////            direction.push_back(3);
////        else
////            direction.push_back(4);
//    }
}


void osg_camera_t::move_down( )
{
    if (ortho)
    {
        top -= camera_speed;
        bottom -= camera_speed;
    }
    else
    {
        eye[2] -= camera_speed;
        center[2] -= camera_speed;
    }
}


void osg_camera_t::move_up( )
{
    if (ortho)
    {
        top += camera_speed;
        bottom += camera_speed;
    }
    else
    {
        eye[2] += camera_speed;
        center[2] += camera_speed;
    }
}

void osg_camera_t::set_move_pos_in( bool forward )
{
    in = center - eye;
    in.normalize();
    in *= camera_speed ;

    if( forward )
    {
        if (camera_type == 2)
        {
 //           direction.push_back(1);
           // in[1] = 0.0;
           // eye+= in;
           // center+=in;


        }
        else if (ortho)
        {
            left   += camera_speed;
            right  -= camera_speed;
            top    -= camera_speed;
            bottom += camera_speed;
        }
        else
        {
            eye += in;
            if( !follower || eye.distance( center ) < 1 )
            {

                center += in;
            }
        }
    }
    else
    {
        
        if (camera_type == 2)
        {
 //           direction.push_back(2);
//            in[1] = 0.0;
//            eye-= in;
//            center-=in;


        }
        else if (ortho)
        {
            left   -= camera_speed;
            right  += camera_speed;
            top    += camera_speed;
            bottom -= camera_speed;
        }
        else
        {
            eye -= in;
            if ( !follower )
                center -= in;
        }
    }
}


void osg_camera_t::set_move_pos_side( bool forward )
{
    in = center - eye;
    cross.cross_product( in, up );
    cross.normalize();
    cross *= camera_speed;

    if( forward )
    {
        if (ortho)
        {
            left += camera_speed;
            right += camera_speed;

        }
        else
        {
            center += cross;
            eye += cross;
        }
    }
    else
    {
        if (ortho)
        {
            left -= camera_speed;
            right -= camera_speed;
        }
        else
        {
            center -= cross;
            eye -= cross;
        }
    }
}


void osg_camera_t::reset()
{

    eye = init_eye;
    center = init_center;
    up = init_up;
    camera_speed  = init_cam_speed;
    camera_rotate = init_cam_rotate;
}


void osg_camera_t::speed_up()
{
    camera_speed+=0.2;
}

void osg_camera_t::speed_down()
{
    camera_speed-=0.2;
    if (camera_speed < 0.1)
        camera_speed = 0.1;
}


bool osg_camera_t::is_ortho() const
{
    return ortho;
}


void osg_camera_t::set_ortho( bool orthographic)
{
    ortho = orthographic;
}

bool osg_camera_t::is_follower() const
{
    return follower;
}

void osg_camera_t::toggle_follow()
{
    follower = !follower;
}


bool osg_camera_t::is_initialized() const
{
    return initialized;
}

void osg_camera_t::set_initialized( bool init)
{
    initialized = init;
}

int osg_camera_t::get_type() const
{
    return camera_type;
}

void osg_camera_t::camera_frame()
{
    in = center - eye;
    cross.cross_product( in, up );
    cross.normalize();
    in.normalize();
    cross *= 3;
    in *= 3 ;

    if (camera_type == 2)
    {
        for (int i = 0; i < 8; ++i)
        {
            if (direction[i])
            {
//                PRX_ERROR_S ("Direction: " << i);
                if (i == 0)
                {
                    center -= cross;
                    eye -= cross;
                }
                else if (i == 1)
                {
//                    in[2] = 0.0;
                    eye-= in;
                    center-=in;
                }
                else if (i == 2)
                {
                    center += cross;
                    eye += cross;
                }
                else if (i == 3)
                {
                    double rotation = 42;
                    rotation *= .42;
                    
                    horizontal_rotation(rotation);

//                    double D     = sqrt(  pow(center[0]-eye[0],2) + pow(center[1]-eye[1],2) );
//                    double theta = atan2( center[0]-eye[0], center[1]-eye[1] );
//                    double roll  = atan2( center[2]-eye[2], D );
//                    double dist  = sqrt(  pow(center[0]-eye[0],2) + pow(center[2]-eye[2],2) + pow(center[1]-eye[1],2) );
//
//                    theta -= rotation;
//
//                    center[2] =  eye[2] + dist * sin(roll) ;
//                    D = dist * cos(roll);
//                    center[0] =  eye[0] + D*cos(theta);
//                    center[1] =  eye[1] + D*sin(theta) ;
                }
                else if (i == 4)
                {
//                    in[2] = 0.0;
                    eye+= in;
                    center+=in;
                }

                else if (i == 5)
                {
                    double rotation = -42;
                    rotation *= .42;
                    
                    horizontal_rotation(rotation);

//                    double D     = sqrt(  pow(center[0]-eye[0],2) + pow(center[1]-eye[1],2) );
//                    double theta = atan2( center[0]-eye[0], center[1]-eye[1] );
//                    double roll  = atan2( center[2]-eye[2], D );
//                    double dist  = sqrt(  pow(center[0]-eye[0],2) + pow(center[2]-eye[2],2) + pow(center[1]-eye[1],2) );
//
//                    theta -= rotation;
//
//                    center[2] =  eye[2] + dist * sin(roll) ;
//                    D = dist * cos(roll);
//                    center[0] =  eye[0] + D*cos(theta);
//                    center[1] =  eye[1] + D*sin(theta) ;
                }
                else if (i == 6)
                {
                    eye[2] += camera_speed;
                    center[2] += camera_speed;
                }
                else if (i == 7)
                {
                    eye[2] -= camera_speed;
 
                    center[2] -= camera_speed;
     
                }
            }
        }

        if (eye[2] < 1.0)
        {
            eye[2] = 1.0;
        }
        if (center[2] < 1.0)
        {
            center[2] = 1.0;
        }
    }
}

osg::ref_ptr<osg::Camera> osg_camera_t::get_wrapped_camera() const
{
    return camera;
}

void osg_camera_t::init(const parameter_reader_t* reader)
{
    reset();
    initialized = false;
    follower = false;
    ortho = false;
    camera_speed = INIT_CAM_SPEED;
    camera_rotate = INIT_CAM_ROTATE;
    for (int i = 0; i < 8; ++i)
            direction[i] = false;

    camera = new osg::Camera();

    ortho = reader->get_attribute_as<bool>("ortho");
    camera_type = reader->get_attribute_as<int>("camera_type", 0);


    if (ortho)
    {
        reset_view();
        left = reader->get_attribute_as<double>("ortho_param/left");
        right = reader->get_attribute_as<double>("ortho_param/right");
        bottom = reader->get_attribute_as<double>("ortho_param/bottom");
        top = reader->get_attribute_as<double>("ortho_param/top");
        zNear = reader->get_attribute_as<double>("ortho_param/zNear");
        zFar = reader->get_attribute_as<double>("ortho_param/zFar");

    }
    else
    {
        eye = reader->get_attribute_as<vector_t>("eye");
        center = reader->get_attribute_as<vector_t>("center");

        up.set(0.0,0.0,1.0);
        init_eye = eye;
        init_center = center;
        init_up = up;
        set_initialized(true);
    }

    camera_speed = reader->get_attribute_as<double>("speed/move");
    camera_rotate = reader->get_attribute_as<double>("speed/rotate");
    init_cam_speed = camera_speed;
    init_cam_rotate = camera_rotate;

    PRX_DEBUG_S("Created osg_camera_t.");
}

    }
 }
