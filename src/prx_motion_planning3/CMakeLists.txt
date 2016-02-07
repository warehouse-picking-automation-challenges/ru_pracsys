cmake_minimum_required(VERSION 2.8.3)
project(prx_motion_planning3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure 
)

#add dynamic reconfigure api
catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/prx_motion_planning3.cfg)
include_directories(include cfg/cpp)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prx_motion_planning3
#  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(prx_motion_planning3.py)
add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

