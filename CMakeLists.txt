cmake_minimum_required(VERSION 3.20)
project(pcl_basic LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED COMPONENTS common io filters visualization)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgcodecs imgproc)

function(configure_pcl_target target_name)
  target_include_directories(
    ${target_name}
    PRIVATE
      ${PROJECT_SOURCE_DIR}/include
      ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(${target_name} PRIVATE ${PCL_DEFINITIONS})
endfunction()

add_executable(
  pcl_basic
  src/main.cpp
  src/apps/simple_cloud_app.cpp
)
configure_pcl_target(pcl_basic)
target_link_libraries(pcl_basic PRIVATE ${PCL_LIBRARIES})

add_executable(
  lesson_01_io
  src/lesson_01_io.cpp
  src/apps/lesson_01_io_app.cpp
)
configure_pcl_target(lesson_01_io)
target_link_libraries(lesson_01_io PRIVATE ${PCL_LIBRARIES})

add_executable(
  lesson_kitti_io
  src/lesson_kitti_io.cpp
  src/apps/lesson_kitti_io_app.cpp
  src/common/point_cloud_io.cpp
)
configure_pcl_target(lesson_kitti_io)
target_link_libraries(
  lesson_kitti_io
  PRIVATE
    ${PCL_LIBRARIES}
    nlohmann_json::nlohmann_json
)

add_executable(
  lesson_visualization
  src/lesson_visualization.cpp
  src/apps/lesson_visualization_app.cpp
  src/common/point_cloud_io.cpp
)
configure_pcl_target(lesson_visualization)
target_link_libraries(
  lesson_visualization
  PRIVATE
    ${PCL_LIBRARIES}
    nlohmann_json::nlohmann_json
)

add_executable(
  pcd2bev
  src/pcd2bev.cpp
  src/apps/pcd2bev_app.cpp
  src/common/point_cloud_io.cpp
)
configure_pcl_target(pcd2bev)
target_link_libraries(
  pcd2bev
  PRIVATE
    ${PCL_LIBRARIES}
    nlohmann_json::nlohmann_json
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
)

add_executable(
  pcd2web
  src/pcd2web.cpp
  src/apps/pcd2web_app.cpp
  src/common/point_cloud_io.cpp
)
configure_pcl_target(pcd2web)
target_link_libraries(
  pcd2web
  PRIVATE
    ${PCL_LIBRARIES}
    nlohmann_json::nlohmann_json
)
