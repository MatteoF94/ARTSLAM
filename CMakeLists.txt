cmake_minimum_required(VERSION 3.17)
project(ARTSLAM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")
include_directories(include
                    include/configurations
                    include/observers
                    include/ukf
                    include/g2o
                    pc_io/include
                    ndt_omp/include
                    fast_gicp/include
                    )

file(GLOB artslam_SRC "src/*.cpp" "src/configuration/*.cpp" "src/g2o/*.cpp" "src/ukf/*.cpp")
add_executable(ARTSLAM main.cpp ${artslam_SRC})

find_package(PCL 1.8 REQUIRED)
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
    target_link_libraries(ARTSLAM ${PCL_LIBRARIES})
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(ARTSLAM ${EIGEN_LIBRARIES})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
target_link_libraries(ARTSLAM
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_SOLVER_PCG}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CHOLMOD}
        ${G2O_TYPES_SLAM3D}
        ${G2O_TYPES_SLAM3D_ADDONS}
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
target_link_libraries(ARTSLAM ${GeographicLib_LIBRARIES})

add_subdirectory(pc_io)
add_subdirectory(ndt_omp)
add_subdirectory(fast_gicp)
target_link_libraries(ARTSLAM pc_io_LIB ndt_omp_LIB fast_gicp)