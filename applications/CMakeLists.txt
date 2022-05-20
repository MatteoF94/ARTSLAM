add_executable(artslam_laser3d_offline artslam_laser3d_offline.cpp)
target_link_libraries(artslam_laser3d_offline
        artslam_laser_3d
        ${catkin_LIBRARIES}
        ${Boost_LOG_LIBRARIES}
        ${Boost_LOG_SETUP_LIBRARIES}
        ${PCL_LIBRARIES}
        )
