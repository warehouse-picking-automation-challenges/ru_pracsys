cmake_minimum_required(VERSION 2.8)
project(object_recognition_linemod)

option(LINEMOD_VIZ_IMG "visualize images with detected objects features" OFF)
option(LINEMOD_VIZ_PCD "visualize point clouds from the reference image and its match from DB" OFF)

find_package(catkin REQUIRED 
  object_recognition_core 
  object_recognition_renderer 
  opencv_candidate 
  sensor_msgs 
  tf
  tf_conversions 
  pcl_conversions
  pcl_ros
  image_geometry 
)
 
find_package (cmake_modules REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PCL 1.8.0 REQUIRED PATHS /usr NO_DEFAULT_PATH)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(CATKIN_DEPENDS pcl_conversions pcl_ros 
		ecto object_recognition_core object_recognition_renderer 
		roscpp rospy
)

#install targets for all things python
catkin_python_setup()

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
) 
include_directories(${sensor_msgs_INCLUDE_DIRS})

# Compile the module
add_subdirectory(python)
add_subdirectory(sample)
add_subdirectory(src)

# add the tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# build the docs
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)

# install the conf files
install(DIRECTORY conf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
