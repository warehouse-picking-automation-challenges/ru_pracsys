cmake_minimum_required(VERSION 2.8)
project(ecto_ros)

find_package(catkin)
find_package(catkin REQUIRED cmake_modules geometry_msgs nav_msgs rosbag roscpp sensor_msgs std_msgs)
catkin_package(CFG_EXTRAS ecto_ros-extras.cmake
               INCLUDE_DIRS include
)

find_package(ecto REQUIRED)

add_definitions(-Wno-strict-aliasing)

include_directories(include)

add_subdirectory(cmake)
add_subdirectory(src)

# configure and install Python
catkin_python_setup()

# Deal with the tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)
