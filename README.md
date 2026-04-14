PCL C++ 库的学习

**简介**
本仓库用于记录 PCL（Point Cloud Library）在 C++ 中的学习与实践，包括环境配置、基础概念、常用算法、示例代码与踩坑记录。

**学习目标**
1. 理解点云数据的基础概念与常用格式。
2. 掌握 PCL 常见模块与算法的使用方法。
3. 能够用 C++ 构建可复用的点云处理流程。
4. 形成可持续迭代的示例与笔记。

**环境与依赖**
- 操作系统: Linux（优先）/ Windows（可选）
- 编译器: GCC/Clang（建议支持 C++17）
- 构建工具: CMake
- 依赖库: `PCL`、`Eigen`、`Boost`
- 可选工具: `VTK`（可视化）、`OpenNI`（传感器支持）

**构建方式**
1. 使用 vcpkg 工具链配置（推荐，当前仓库默认）:
```bash
/home/lingzhiying/learn/learn_pcl/vcpkg/downloads/tools/cmake-4.2.3-linux/cmake-4.2.3-linux-x86_64/bin/cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=/home/lingzhiying/learn/learn_pcl/vcpkg/scripts/buildsystems/vcpkg.cmake

/home/lingzhiying/learn/learn_pcl/vcpkg/downloads/tools/cmake-4.2.3-linux/cmake-4.2.3-linux-x86_64/bin/cmake --build build -j
./build/pcl_basic
```
2. 使用脚本一键构建:
```bash
./scripts/build.sh
./build/pcl_basic
```
3. 如需使用系统 CMake，请确保版本 >= 3.20。

**CMakeLists.txt 示例**
```cmake
cmake_minimum_required(VERSION 3.20)
project(pcl_learning LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL CONFIG REQUIRED COMPONENTS common)

add_executable(pcl_demo src/main.cpp)
target_include_directories(pcl_demo PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcl_demo PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcl_demo PRIVATE ${PCL_LIBRARIES})
```

**目录结构**
- `src/`：示例代码
- `include/`：头文件与工具封装
- `data/`：点云数据集（`*.pcd` 等）
- `notes/`：学习笔记与总结
- `scripts/`：辅助脚本
- `build/`：构建输出（本地生成）
- `vcpkg/`：依赖管理与工具链

**学习路线**
1. 点云基础: 点类型、坐标系、常见数据格式。
2. IO 模块: 读取/写入 `PCD` 与 `PLY`。
3. 过滤: 体素降采样、直通滤波、统计离群点移除。
4. 特征: 法线估计、FPFH 等。
5. 配准: ICP、NDT。
6. 分割: RANSAC 平面分割、聚类。
7. 可视化: `PCLVisualizer` 的基本使用。

**示例清单（规划）**
1. 读取与保存点云。
2. 体素降采样与直通滤波。
3. 法线估计与可视化。
4. 平面分割与聚类。
5. ICP 配准示例。

**最小示例**
```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 3;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  cloud.points[0].x = 1.0f; cloud.points[0].y = 2.0f; cloud.points[0].z = 3.0f;
  cloud.points[1].x = 2.0f; cloud.points[1].y = 3.0f; cloud.points[1].z = 4.0f;
  cloud.points[2].x = 3.0f; cloud.points[2].y = 4.0f; cloud.points[2].z = 5.0f;
  pcl::io::savePCDFileASCII("data/demo.pcd", cloud);
  return 0;
}
```

**常见问题**
- 找不到 `PCLConfig.cmake`: 确认已安装 `PCL` 开发包，并将其 CMake 配置路径加入 `CMAKE_PREFIX_PATH`。
- 编译报缺少 `VTK` 或 `Boost`: 安装对应依赖并确保版本匹配。
- 运行时找不到库: 检查 `LD_LIBRARY_PATH` 或系统动态库路径。

**下一步**
- 在 `src/` 中逐步加入示例，并在 `notes/` 中记录算法要点与参数选择经验。
- 增加数据集与可视化示例，形成完整流程。
