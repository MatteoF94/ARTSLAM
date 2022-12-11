add_library(artslam_lib SHARED
        configuration_parser.cpp
        configuration_parser.h
        prefilterer.cpp
        pretracker.cpp
        keyframe_laser_3d.cpp
        registration.cpp
        tracker.cpp
        ground_detector.cpp
        graph_handler.cpp
        information_matrix_calculator.cpp
        loop_detector.cpp
        scancontext/scancontext.cpp
        backend_handler.cpp)

target_link_libraries(artslam_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_SOLVER_PCG}
        ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
        ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
        ${G2O_TYPES_SLAM3D}
        ${G2O_TYPES_SLAM3D_ADDONS}
        )
