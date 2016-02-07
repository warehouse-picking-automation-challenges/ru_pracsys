cmake_minimum_required(VERSION 2.8)

#project(KEBROS_DEPENDENCIES CXX C)

set(LIBRARY_OUTPUT_PATH    ${CMAKE_SOURCE_DIR}/lib)

add_definitions(-Wall -fPIC )

if(APPLE)
    add_definitions(-DAPPLE)

    include(CMakeFindFrameworks)
    CMAKE_FIND_FRAMEWORKS(vecLib)
    if(vecLib_FRAMEWORKS)
      set(
        prx_vecLib_FRAMEWORK ${vecLib_FRAMEWORKS}
        CACHE PATH "Path to vecLib framework"
      )
      message("Found veclib")
      mark_as_advanced(prx_vecLib_FRAMEWORK)
    endif(vecLib_FRAMEWORKS)

endif(APPLE)

set(BUILD_SHARED_LIBS TRUE)

#
# ANN
#

set( ANN_DIR "${CMAKE_SOURCE_DIR}/ANN" )

include_directories( ${ANN_DIR} )

add_library( ANN
		${ANN_DIR}/ANN.cc
		${ANN_DIR}/bd_pr_search.cc
		${ANN_DIR}/bd_search.cc
		${ANN_DIR}/bd_tree.cc
		${ANN_DIR}/brute.cc
		${ANN_DIR}/kd_pr_search.cc
		${ANN_DIR}/kd_search.cc
		${ANN_DIR}/kd_split.cc
		${ANN_DIR}/kd_tree.cc
		${ANN_DIR}/kd_util.cc
		${ANN_DIR}/perf.cc  )

#
# PNG
#

set( PNG_DIR "${CMAKE_SOURCE_DIR}/PNGCreator" )
include_directories( ${PNG_DIR} )
file(GLOB_RECURSE SRC_PNG ${PNG_DIR}/*.cpp)
add_library( PRXPNG ${SRC_PNG})


#
# LEVMAR
#

#set( LEVMAR_DIR "${CMAKE_SOURCE_DIR}/levmar-2.6" )
#add_subdirectory( ${LEVMAR_DIR} ) 
#include_directories( ${LEVMAR_DIR} )
#file(GLOB_RECURSE SRC_LEVMAR ${LEVMAR_DIR}/*.cpp)
#add_library( PRXLEVMAR ${SRC_LEVMAR})

#
# POSEST
#

#set( POSEST_DIR "${CMAKE_SOURCE_DIR}/posest-1.0" )
#add_subdirectory( ${POSEST_DIR} )
#include_directories( ${POSEST_DIR} ${LEVMAR_DIR} )
#file(GLOB_RECURSE SRC_POSEST ${POSEST_DIR}/*.cpp)
#add_library( PRXPOSEST ${SRC_POSEST})

if(APPLE)
#
# PQP
#
	set( PQP_DIR "${CMAKE_SOURCE_DIR}/PQP" )

	include_directories( ${PQP_DIR} )

	add_library( PQP
			${PQP_DIR}/Build.cpp
			${PQP_DIR}/BV.cpp
			${PQP_DIR}/PQP.cpp
			${PQP_DIR}/TriDist.cpp)

	file(COPY ${PQP_DIR}/MatVec.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/BVTQ.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/PQP.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/PQP_Compile.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/RectDist.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/TriDist.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/Tri.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/Build.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/BV.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/GetTime.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/OBB_Disjoint.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/PQP_Internal.h DESTINATION /usr/local/include )
else(APPLE)

	set( PQP_DIR "${CMAKE_SOURCE_DIR}/PQP" )

	include_directories( ${PQP_DIR} )

	add_library( PQP
			${PQP_DIR}/Build.cpp
			${PQP_DIR}/BV.cpp
			${PQP_DIR}/PQP.cpp
			${PQP_DIR}/TriDist.cpp)
endif(APPLE)
