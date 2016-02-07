PROJECT(prx_visualization)
cmake_minimum_required(VERSION 2.6.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-Wall")

#rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


find_package( catkin REQUIRED COMPONENTS roscpp rospy roslib tf)

# Automatically add all .cpp files in these directories. 
include_directories(${PROJECT_SOURCE_DIR})
aux_source_directory(${PROJECT_SOURCE_DIR}/prx/visualization/
	SRC_VISUALIZATION)

# Define using STL flag for TinyXML
add_definitions(-DTIXML_USE_STL)

# Optional OSG plugin
find_package(OpenSceneGraph 2.8 COMPONENTS
    osgDB
    osgGA
    osgUtil
    osgViewer
    osgText)

# Optional OpenGL plugin
find_package(OpenGL)
find_package(GLUT)

if(OPENSCENEGRAPH_FOUND)
    include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
    aux_source_directory(${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG
    	SRC_OSG_PLUGIN)
endif(OPENSCENEGRAPH_FOUND)

find_package(Boost REQUIRED)

#rosbuild_link_boost(${PROJECT_NAME}_lib
#        filesystem
#        system
#        program_options
#        )
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${Boost_LIBRARIES})#filesystem system program_options)


#find_package(prx_utilities)
include_directories(${PROJECT_SOURCE_DIR}/../prx_utilities)
include_directories(${PROJECT_SOURCE_DIR}/../../devel/include)


add_library(${PROJECT_NAME}_lib
        ${PROJECT_SOURCE_DIR}/prx/visualization/ui_topic.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/viewer.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/visualization.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/visualization_comm.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_camera.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_geode.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_ghost_switch.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_handler_wrapper.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_hud.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_light.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_material.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_scene.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_texture.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_viewer.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_visualization.cpp
        ${PROJECT_SOURCE_DIR}/prx/visualization/PLUGINS/OSG/osg_window.cpp
	${SRC_VISUALIZATION}
        )




if(OPENSCENEGRAPH_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib
        ${OPENSCENEGRAPH_LIBRARIES}
        )
endif(OPENSCENEGRAPH_FOUND)


target_link_libraries(${PROJECT_NAME}_lib prx_utilities)
target_link_libraries(${PROJECT_NAME}_lib  class_loader)
target_link_libraries(${PROJECT_NAME}_lib  tinyxml)
target_link_libraries(${PROJECT_NAME}_lib  ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${OPENSCENEGRAPH_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${OPENGL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${GLUT_LIBRARY})

################
## Executable ##
################
add_executable(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/prx/visualization/main.cpp
        )
# Locating Local Library
#
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

# Define using STL flag for TinyXML
add_definitions(-DTIXML_USE_STL)
