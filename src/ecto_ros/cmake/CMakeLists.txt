foreach(file pubsub_gen_wrap.cmake gen_msg_wrappers.py)
    file(COPY ${PROJECT_SOURCE_DIR}/cmake/${file}
         DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
         USE_SOURCE_PERMISSIONS
    )
endforeach()

# install the extra CMake files
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        COMPONENT main
        USE_SOURCE_PERMISSIONS
)

# install the include folder
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ecto_ros/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        COMPONENT main
)

include(pubsub_gen_wrap.cmake)
