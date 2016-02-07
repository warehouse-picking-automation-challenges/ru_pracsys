cmake_minimum_required(VERSION 2.8.3)
project(prx_decision_making)

find_package(catkin REQUIRED cv_bridge genmsg image_transport roscpp rospy std_msgs message_generation)

find_package(OpenCV REQUIRED)

#Json Parser
FIND_PACKAGE(PythonInterp 2.6 REQUIRED)

add_message_files(
  FILES
  DecisionMakingStateMessage.msg
  GraspPlanningStateMessage.msg
  MappingStateMessage.msg
  MotionPlanningStateMessage.msg
  ObjectPoseRecognitionStateMessage.msg 
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include 
#  LIBRARIES prx_decision_making
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(prx_decision_making_node src/prx_decision_making.cpp src/jsoncpp.cpp src/read_file.cpp)
add_executable(ru_decision_making src/ru_pracsys.cpp src/jsoncpp.cpp src/read_file.cpp)
add_executable(icra_decision_making src/grasp_decision_making.cpp src/jsoncpp.cpp src/read_file.cpp)

add_dependencies(prx_decision_making_node prx_decision_making_generate_messages_cpp)
add_dependencies(ru_decision_making prx_decision_making_generate_messages_cpp)
add_dependencies(icra_decision_making prx_decision_making_generate_messages_cpp)

target_link_libraries(prx_decision_making_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(ru_decision_making ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(icra_decision_making ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# message("################## include dest " ${CATKIN_PACKAGE_INCLUDE_DESTINATION}) 
