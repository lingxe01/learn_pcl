cmake_minimum_required(VERSION 3.20)
project(pcl_basic LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED COMPONENTS common)

add_executable(pcl_basic src/main.cpp)
target_include_directories(pcl_basic PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcl_basic PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcl_basic PRIVATE ${PCL_LIBRARIES})
