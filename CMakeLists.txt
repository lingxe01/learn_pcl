cmake_minimum_required(VERSION 3.20)
project(pcl_basic LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED COMPONENTS common io filters visualization)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(pcl_basic src/main.cpp)
target_include_directories(pcl_basic PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcl_basic PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcl_basic PRIVATE ${PCL_LIBRARIES})

add_executable(lesson_01_io src/lesson_01_io.cpp)
target_include_directories(lesson_01_io PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(lesson_01_io PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(lesson_01_io PRIVATE ${PCL_LIBRARIES})

add_executable(lesson_kitti_io src/lesson_kitti_io.cpp)
target_include_directories(lesson_kitti_io PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(lesson_kitti_io PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(lesson_kitti_io PRIVATE ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)

add_executable(lesson_visualization src/lesson_visualization.cpp)
target_include_directories(lesson_visualization PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(lesson_visualization PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(lesson_visualization PRIVATE ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)

add_executable(pcd2bev src/pcd2bev.cpp)
target_include_directories(pcd2bev PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcd2bev PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcd2bev PRIVATE ${PCL_LIBRARIES} nlohmann_json::nlohmann_json opencv_core opencv_imgcodecs opencv_imgproc)

add_executable(pcd2web src/pcd2web.cpp)
target_include_directories(pcd2web PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcd2web PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcd2web PRIVATE ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)
