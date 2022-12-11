add_executable(artslam_offline artslam_laser3d_offline.cpp)
target_link_libraries(artslam_offline
        artslam_lib
        ${catkin_LIBRARIES}
        ${Boost_LOG_LIBRARIES}
        ${Boost_LOG_SETUP_LIBRARIES}
        ${PCL_LIBRARIES}
        )
