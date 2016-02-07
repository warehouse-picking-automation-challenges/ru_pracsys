#############################
## PRELIMINARIES
#############################

# Set the required version of CMake
cmake_minimum_required(VERSION 2.8.3)

# Define the project name
project(prx_planning)

#############################
## SET ENVIRONMENT VARIABLES
#############################

# Set the build type.  Options are Coverage, Debug, Release, RelWitheDebInfo, MinSizeRel
set(CMAKE_BUILD_TYPE Release)

# Locate CMake Modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

#############################
## FIND PACKAGES
#############################

# catkin
find_package( catkin REQUIRED 
              COMPONENTS roscpp rospy roslib message_generation geometry_msgs std_msgs sensor_msgs baxter_core_msgs class_loader tf prx_utilities prx_simulation actionlib_msgs actionlib prx_decision_making)

add_action_files(DIRECTORY action FILES compute_plan.action execute_plan.action)

catkin_python_setup()


# Octave
find_package(OCTAVE)

# Locating Boost 
find_package(Boost REQUIRED)

# OCTOMAP
find_package( octomap )

#Yaml-Cpp
find_package( YamlCpp )

# ANN
find_library(ANNLIB ANN
	     paths ${PROJECT_SOURCE_DIR}/../prx_external/lib
	     doc "ANN library")
find_path(ANNINCLUDE ANN.h
	  paths ${PROJECT_SOURCE_DIR}/../prx_external/ANN
	  doc "ANN include directory")
if(ANNLIB AND ANNINCLUDE)
    message("-- Found ANN library and header files")
else(ANNLIB AND ANNINCLUDE)
    message(WARNING "Could not find ANN library and header files")
endif(ANNLIB AND ANNINCLUDE)

# PNG
find_library(PNGLIB PRXPNG
	     paths ${PROJECT_SOURCE_DIR}/../prx_external/lib
	     doc "PNG library")
find_path(PNGINCLUDE lodepng.hpp
	  paths ${PROJECT_SOURCE_DIR}/../prx_external/PNGCreator
	  doc "PNG include directory")
if(PNGLIB AND PNGINCLUDE)
    message("-- Found PNG library and header files")
else(PNGLIB AND PNGINCLUDE)
    message(WARNING "Could not find PNG library and header files")
endif(PNGLIB AND PNGINCLUDE)

#
# Locating PQP
#
find_library(PQPLIB PQP
	     paths /usr/local/lib /opt/local/lib ../prx_external/lib
	     doc "PQP library")
find_path(PQPINCLUDE PQP.h
	  paths /usr/local/include/pqp /opt/local/include/PQP ../prx_external/PQP
	  doc "PQP include directory")
if(PQPLIB AND PQPINCLUDE)
    message("-- Found PQP library and header files")
else(PQPLIB AND PQPINCLUDE)
    message(WARNING "Could not find PQP library and header files")
endif(PQPLIB AND PQPINCLUDE)

##
## MESSAGES, SERVICES AND ACTION TARGETS
##
file(GLOB_RECURSE SRV_PLAN RELATIVE ${PROJECT_SOURCE_DIR}/srv *.srv)
add_service_files(FILES ${SRV_PLAN})

generate_messages(DEPENDENCIES geometry_msgs std_msgs sensor_msgs baxter_core_msgs actionlib_msgs)

##
## REGISTER THIS AS A CATKIN PACKAGE
##
catkin_package(
  INCLUDE_DIRS "prx"
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib message_generation geometry_msgs std_msgs message_runtime sensor_msgs baxter_core_msgs class_loader tf prx_utilities prx_simulation
  DEPENDS OCTAVE Boost octomap
 )

##
## INCLUDE DIRECTORIES and ADD DEFINITIONS
##

# Our code...
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../prx_utilities)
include_directories(${PROJECT_SOURCE_DIR}/../prx_simulation)
include_directories(${PROJECT_SOURCE_DIR}/../prx_packages/manipulation)
include_directories(${PROJECT_SOURCE_DIR}/../prx_packages/baxter)

# Catkin packages
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(${prx_decision_making_INCLUDE_DIRS})

# Octave
if(OCTAVE_FOUND)
    include_directories(${OCTAVE_INCLUDE_DIR})
    add_definitions(-DOCTAVE_FOUND)
endif(OCTAVE_FOUND)

# Boost
include_directories(${Boost_INCLUDE_DIRS})

# Octomap
include_directories(${OCTOMAP_INCLUDE_DIRS})

# YamlCpp
include_directories(${YAMLCPP_INCLUDE_DIR})

# ANN
if(ANNLIB AND ANNINCLUDE)
    include_directories(${ANNINCLUDE})
endif(ANNLIB AND ANNINCLUDE)

# PNG
if(PNGLIB AND PNGINCLUDE)
    include_directories(${PNGINCLUDE})
endif(PNGLIB AND PNGINCLUDE)

# PQP
if(PQPLIB AND PQPINCLUDE)
    include_directories(${PQPINCLUDE})
endif(PQPLIB AND PQPINCLUDE)

##
## ADD LIBRARY and TARGET LINK LIBRARIES
##

# define the planning source files
file(GLOB_RECURSE SRC_PLANNING ${PROJECT_SOURCE_DIR}/prx/planning/*.cpp)

# add the source files from the Manipulation packages
get_filename_component(PARENT_DIR "${CMAKE_SOURCE_DIR}" PATH)
file(GLOB_RECURSE SRC_manipulation ${PROJECT_SOURCE_DIR}/../prx_packages/manipulation/planning/*.cpp)
set(SRC_PLANNING ${SRC_manipulation} ${SRC_PLANNING})

add_library(${PROJECT_NAME} ${SRC_PLANNING})

# target link libraries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${YAMLCPP_LIBRARY} ${ANNLIB} ${PNGLIB} ${PQPLIB} ${prx_utilities_LIBRARIES}  ${prx_simulation_LIBRARIES})

if(OCTAVE_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OCTAVE_LIBRARY} ${OCTAVE_INT_LIBRARY} ${OCTAVE_CRUFT_LIBRARY})
endif(OCTAVE_FOUND)

#target_link_libraries(${PROJECT_NAME}_lib  console_bridge)

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ENABLE_EXPORTS TRUE)

# add dependency to the generation of messages
add_dependencies( ${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp prx_decision_making_generate_messages_cpp )

################
## Executables
################
add_executable(${PROJECT_NAME}_exe ${PROJECT_SOURCE_DIR}/prx/main.cpp)
target_link_libraries(${PROJECT_NAME}_exe ${PROJECT_NAME})

add_executable(planning_experiment ${PROJECT_SOURCE_DIR}/prx/experiment.cpp)
target_link_libraries(planning_experiment ${PROJECT_NAME})

##
## TESTS AND INSTALL
##
install( DIRECTORY "prx"
         DESTINATION "include"
         FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
         )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_exe
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)