cmake_minimum_required(VERSION 3.17)
project(PC_IO)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

find_package(PCL 1.8 REQUIRED)
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
endif()

file(GLOB pc_io_SRC "src/*.cpp")
add_library(pc_io_LIB STATIC ${pc_io_SRC})
target_include_directories(pc_io_LIB PUBLIC include)