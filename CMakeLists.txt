cmake_minimum_required(VERSION 3.0.2)
project(artslam_laser_3d)

# set the default path for built executables to the "bin" directory of the project
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set the default path for built libraries to the "lib" directory of the project
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -fopenmp -DBOOST_LOG_DYN_LINK")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# set the cmake build type
#set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        artslam_core
        ndt_omp
        fast_gicp
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS program_options log log_setup)
include_directories(${Boost_INCLUDE_DIR})
message("${PROJECT_NAME} - Boost version ${Boost_VERSION}")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
message("${PROJECT_NAME} - PCL version ${PCL_VERSION}")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("${PROJECT_NAME} - OpenCV version ${OpenCV_VERSION}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenMP REQUIRED)

find_library(VGICP_CUDA_FOUND NAMES fast_vgicp_cuda)
message(STATUS "VGICP_CUDA_FOUND:" ${VGICP_CUDA_FOUND})
if(VGICP_CUDA_FOUND)
    add_definitions(-DUSE_VGICP_CUDA)
endif()

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS src include
  LIBRARIES artslam_laser_3d
  CATKIN_DEPENDS artslam_core ndt_omp fast_gicp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/applications)
