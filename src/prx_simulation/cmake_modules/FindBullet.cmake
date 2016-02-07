# - Try to find Bullet
# Once done this will define
#  BULLET_FOUND - System has bullet
#  BULLET_INCLUDE_DIR - The bullet include directories
#  BULLET_LIBRARIES - The libraries needed to use bullet

find_path(BULLET_INCLUDE_DIR btBulletDynamicsCommon.h PATHS /usr/local/include/bullet )
find_library(BULLET_LIBRARY BulletDynamics PATHS /usr/local/lib} )
find_library(BULLET_COLL_LIBRARY BulletCollision PATHS /usr/local/lib)
find_library(BULLET_MATH_LIBRARY LinearMath PATHS /usr/local/lib )


set (BULLET_LIBRARIES ${BULLET_LIBRARY} ${BULLET_COLL_LIBRARY} ${BULLET_MATH_LIBRARY}) 

IF(BULLET_INCLUDE_DIR AND BULLET_LIBRARIES)
  MESSAGE(STATUS "BULLET_INCLUDE_DIR =${BULLET_INCLUDE_DIR}")
  MESSAGE(STATUS "BULLET_LIBRARIES =${BULLET_LIBRARIES}")
  set(BULLET_FOUND 1)
ELSE()
    MESSAGE(STATUS      "Bullet was not found.")
ENDIF()