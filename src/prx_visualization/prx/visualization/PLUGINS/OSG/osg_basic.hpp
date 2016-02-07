/**
 * @file osg_basic.hpp 
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

#ifndef PRACSYS_OSG_BASIC_HPP
#define PRACSYS_OSG_BASIC_HPP

#include <osg/Texture2D>
#include <osg/Image>
#include <osg/ImageSequence>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Drawable>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/PrimitiveSet>
#include <osg/Depth>
#include <osg/NodeVisitor>
#include <osg/BoundingBox>
#include <osg/Billboard>
#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <osg/Projection>

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgGA/GUIEventHandler>
#include <osgGA/GUIActionAdapter>

#include <osgUtil/CullVisitor>
#include <osgUtil/Optimizer>

#endif
