cmake_minimum_required(VERSION 3.20)
project(pointcloud_web_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 这个分支默认只构建 Web 点云查看功能，尽量把依赖收敛到最小集合。
# 如需重新启用其它示例，可在后续单独扩展。
find_package(PCL CONFIG REQUIRED COMPONENTS common io filters)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(pcd2web src/pcd2web.cpp)
target_include_directories(pcd2web PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcd2web PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcd2web PRIVATE ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)
