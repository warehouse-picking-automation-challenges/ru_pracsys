cmake_minimum_required(VERSION 2.8.3)
project(unigripper_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs)

add_service_files(
  DIRECTORY srv
  FILES
  UnigripperVacuumOn.srv
)

generate_messages(
 DEPENDENCIES geometry_msgs sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
