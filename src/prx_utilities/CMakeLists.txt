#############################
## PRELIMINARIES
#############################

# Set the required version of CMake
cmake_minimum_required(VERSION 2.8.3)

# Define the project name
project(prx_utilities)

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

find_package(cmake_modules REQUIRED)

# catkin
find_package( catkin REQUIRED 
              COMPONENTS roscpp rospy roslib message_generation geometry_msgs std_msgs class_loader)

# xml
find_package(LibXml2)

# Eigen
find_package(Eigen REQUIRED)

# Octave
find_package(OCTAVE)

# Locating Boost ?????????????????? what components do we use from Boost ????????????????
find_package(Boost REQUIRED)

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

# Assimp
find_path(ASSIMPINCLUDE assimp.h
	  paths /usr/local/include/assimp /opt/local/include/assimp /usr/include/assimp
	  doc "ASSIMP include directory")
find_library(ASSLIB assimp
             paths /usr/local/lib /opt/local/lib /usr/lib
	     doc "ASSIMP library")
if(ASSIMPINCLUDE AND ASSLIB)
    message("-- Found ASSIMP header files")
else(ASSIMPINCLUDE AND ASSLIB)
    message(WARNING "Could not find ASSIMP header files")
endif(ASSIMPINCLUDE AND ASSLIB)

##
## MESSAGES, SERVICES AND ACTION TARGETS
##
file(GLOB_RECURSE MSG_UTIL RELATIVE ${PROJECT_SOURCE_DIR}/msg *.msg)
add_message_files(FILES ${MSG_UTIL})

file(GLOB_RECURSE SRV_UTIL RELATIVE ${PROJECT_SOURCE_DIR}/srv *.srv)
add_service_files(FILES ${SRV_UTIL})

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

##
## REGISTER THIS AS A CATKIN PACKAGE
##
catkin_package(
  INCLUDE_DIRS "prx"
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib message_generation geometry_msgs std_msgs message_runtime class_loader
  DEPENDS LibXml2 Eigen OCTAVE Boost
  )

##
## INCLUDE DIRECTORIES and ADD DEFINITIONS
##

# Our code...
include_directories(${PROJECT_SOURCE_DIR})

# Catkin packages
include_directories( ${catkin_INCLUDE_DIRS} )

# XML
include_directories(${LIBXML2_INCLUDE_DIR})
add_definitions(-DTIXML_USE_STL)

# Eigen
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Octave
if(OCTAVE_FOUND)
    include_directories(${OCTAVE_INCLUDE_DIR})
    add_definitions(-DOCTAVE_FOUND)
endif(OCTAVE_FOUND)

# Boost
include_directories(${Boost_INCLUDE_DIRS})

# ANN
if(ANNLIB AND ANNINCLUDE)
    include_directories(${ANNINCLUDE})
endif(ANNLIB AND ANNINCLUDE)

# PNG
if(PNGLIB AND PNGINCLUDE)
    include_directories(${PNGINCLUDE})
endif(PNGLIB AND PNGINCLUDE)

# ASSIMP
if(ASSIMPINCLUDE)
    include_directories(${ASSIMPINCLUDE})
    add_definitions("-DOLD_ASSIMP")
endif(ASSIMPINCLUDE)

##
## ADD LIBRARY and TARGET LINK LIBRARIES
##

# add library
file(GLOB_RECURSE SRC_UTILITIES ${PROJECT_SOURCE_DIR}/prx/utilities/*.cpp)
add_library(${PROJECT_NAME} ${SRC_UTILITIES})

# target link libraries
target_link_libraries(${PROJECT_NAME} ${LIBXML2_LIBRARIES} ${Boost_LIBRARIES} ${ANNLIB} ${PNGLIB} ${ASSLIB} )

if(OCTAVE_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OCTAVE_LIBRARY} ${OCTAVE_INT_LIBRARY} ${OCTAVE_CRUFT_LIBRARY})
endif(OCTAVE_FOUND)

# add dependency to the generation of messages
add_dependencies( ${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp )

##
## TESTS AND INSTALL
##
install( DIRECTORY "prx"
         DESTINATION "include"
         FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
         )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)