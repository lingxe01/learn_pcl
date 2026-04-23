# learn_pcl

`learn_pcl` 是一个用于学习和实践 `PCL (Point Cloud Library)` 的 C++ 项目。
当前仓库围绕一条完整的点云处理链路展开，包括基础点云操作、PCD 读写、KITTI 点云读取、可视化、BEV 导出，以及面向浏览器的 JSON 导出。

与早期的单文件示例相比，这个项目已经演进为更清晰的分层结构，主要包含 `src`、`src/apps`、`src/common` 和 `include` 等模块。

## 项目目标

当前仓库主要覆盖以下内容：

- `PCL` 基础点云数据结构与常用算法
- `.pcd` 文件的读取与写入
- KITTI `.bin` 点云的读取
- 体素下采样与基础预处理
- 点云可视化与截图导出
- 点云转 `BEV` 图像
- 点云转浏览器可使用的 `JSON`

## 开发环境

- 操作系统：Linux
- 编译器：支持 `C++17` 的 `g++` 或 `clang++`
- 构建工具：`CMake >= 3.20`
- Python：`python3`，用于启动轻量级 Web 服务

## 依赖管理

项目依赖通过 `vcpkg.json` 管理。
当前实际使用的核心库包括：

- `pcl`
- `eigen3`
- `boost`
- `nlohmann-json`
- `opencv4`

仓库内同时包含 `vcpkg/` 和 `vcpkg_installed/`，这样可以提高依赖安装与构建的可复现性。

## 构建方式

### 推荐方式

```bash
./scripts/build.sh
```

这个脚本会：

- 自动定位仓库根目录
- 使用仓库内的 `vcpkg` toolchain
- 在需要时优先使用 `.local/bin/autoconf`
- 将构建产物输出到 `build/`

### 手动构建

```bash
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=/home/lingzhiying/learn/learn_pcl/vcpkg/scripts/buildsystems/vcpkg.cmake

cmake --build build -j
```

## 可执行程序

当前 `CMakeLists.txt` 会构建以下 6 个可执行文件：

- `build/pcl_basic`
- `build/lesson_01_io`
- `build/lesson_kitti_io`
- `build/lesson_visualization`
- `build/pcd2bev`
- `build/pcd2web`

常见运行方式：

```bash
./build/pcl_basic
./build/lesson_01_io
./build/lesson_kitti_io
./build/lesson_visualization
./build/pcd2bev
./build/pcd2web
```

## 仓库结构

当前仓库采用分层组织方式。

### 入口层：`src/*.cpp`

这些文件都比较薄，主要负责程序入口以及调用对应的 App 类。

- `src/main.cpp`
- `src/lesson_01_io.cpp`
- `src/lesson_kitti_io.cpp`
- `src/lesson_visualization.cpp`
- `src/pcd2bev.cpp`
- `src/pcd2web.cpp`

### 应用层：`src/apps/` 与 `include/apps/`

这一层承载各个示例或工具的主要业务逻辑。

- `simple_cloud_app`
  最基础的 PCL 示例，构造一个小型 `PointXYZ` 点云并计算其质心。

- `lesson_01_io_app`
  最基础的 `.pcd` 文件写入与读取示例。

- `lesson_kitti_io_app`
  读取 KITTI `.bin` 点云并进行体素下采样。

- `lesson_visualization_app`
  读取点云、执行可视化并导出截图。

- `pcd2bev_app`
  将点云转换为 BEV 图像及相关特征图。

- `pcd2web_app`
  将 `.bin` 或 `.pcd` 点云转换为浏览器可加载的 JSON。

### 公共模块层：`src/common/` 与 `include/common/`

这一层放置多个程序共用的可复用模块。

- `point_cloud_io`
  统一的点云读取工具。
  当前支持：
  - KITTI `.bin`
  - `.pcd`

该模块被 `lesson_kitti_io`、`lesson_visualization`、`pcd2bev` 和 `pcd2web` 复用。

## 目录说明

### 根目录关键文件

- `CMakeLists.txt`
  主构建文件，负责声明依赖、头文件路径和各个可执行目标。

- `README.md`
  英文版项目说明文档。

- `README.zh-CN.md`
  中文版项目说明文档。

- `vcpkg.json`
  依赖清单。

### `include/`

按模块组织的头文件目录。

- `include/apps/`
  各个 App 类的接口声明。

- `include/common/`
  公共工具模块的接口声明。

### `src/`

源码目录，主要分为三部分：

- `src/*.cpp`
  可执行程序入口。

- `src/apps/`
  各个示例与工具的主要实现。

- `src/common/`
  公共复用实现。

### `config/`

运行时配置目录。

- `config/pointcloud.json`
  点云工具统一配置文件，主要包含：
  - 输入点云路径
  - 体素下采样尺寸
  - 截图输出路径
  - BEV 输出路径
  - Web JSON 输出路径
  - Web 服务端口
  - BEV 范围与分辨率

### `data/`

示例数据目录。

- `data/lesson_01_io.pcd`
  由 `lesson_01_io` 生成的示例 PCD 文件。

### `output/`

运行输出目录。当前常见产物包括：

- `kitti_000000_bev.png`
- `kitti_000000_bev_height.png`
- `kitti_000000_bev_intensity.png`
- `kitti_000000_bev_density.png`
- `web_runtime_pointcloud.json`

### `scripts/`

辅助脚本目录。

- `scripts/build.sh`
  一键构建脚本。

- `scripts/serve_web.py`
  基于 Python 的轻量级 Web 服务脚本，用于提供静态页面和点云转换接口。

### `web/`

浏览器端点云查看页面。

- `web/index.html`
  浏览器查看点云的入口页面。

### `build/`

构建输出目录，包含：

- CMake 生成文件
- 编译后的可执行程序
- 中间构建产物

这是生成目录，不是核心源码目录。

### `vcpkg/`

仓库内置的 `vcpkg` 工具链目录。

这样做的好处包括：

- 依赖管理更稳定
- 环境搭建更直接
- 减少对系统全局库的依赖

### `vcpkg_installed/`

通过 `vcpkg` 安装后的依赖目录。

### `.local/`

仓库内自带的本地工具目录。
当前主要用于提供某些依赖构建过程需要的辅助工具，例如较新的 `autoconf`。

### `notes/`

预留目录，可用于记录学习笔记、实验过程和问题排查。

## 程序说明

### 1. `pcl_basic`

最基础的 PCL 入门示例：

- 手工构造点云
- 计算三维质心
- 输出点云大小与质心结果

适合用于理解：

- `pcl::PointCloud<T>`
- `PointXYZ`
- `pcl::compute3DCentroid`

### 2. `lesson_01_io`

最基础的 `.pcd` IO 示例：

- 生成一个简单点云
- 将其写入 `data/lesson_01_io.pcd`
- 再次读回并输出示例数据

### 3. `lesson_kitti_io`

KITTI 点云读取示例：

- 从配置文件读取输入路径
- 将 `.bin` 点云加载为 `PointXYZI`
- 执行体素下采样
- 输出过滤前后的点数信息

### 4. `lesson_visualization`

点云可视化示例：

- 读取并下采样点云
- 使用 `PCLVisualizer` 进行可视化
- 根据配置导出截图

### 5. `pcd2bev`

点云转 BEV 工具：

- 读取点云
- 按配置范围裁剪
- 生成高度图、强度图、密度图
- 合成为一张 BEV 图像
- 将结果输出到 `output/`

### 6. `pcd2web`

点云转 Web JSON 工具：

- 同时支持 `.bin` 与 `.pcd`
- 支持命令行参数覆盖
- 支持体素过滤和空间范围过滤
- 输出前端可直接加载的 JSON 数据

## Web 查看方式

### 启动服务

```bash
python3 scripts/serve_web.py --host 127.0.0.1 --port 8080
```

### 浏览器访问

```text
http://127.0.0.1:8080/web/
```

如果服务运行在远程 Linux 服务器上，可以通过本地端口转发访问：

```bash
ssh -N -L 8080:127.0.0.1:8080 <user>@<server>
```

然后在本地浏览器打开：

```text
http://127.0.0.1:8080/web/
```

## 推荐学习顺序

建议按下面的顺序学习：

1. `pcl_basic`
   先理解最基本的点云容器与质心计算。

2. `lesson_01_io`
   学习 `.pcd` 文件的读写过程。

3. `lesson_kitti_io`
   学习真实激光点云的读取与过滤。

4. `lesson_visualization`
   学习如何对点云进行可视化观察。

5. `pcd2bev`
   学习点云如何转换为栅格化的 BEV 表示。

6. `pcd2web`
   学习点云如何导出为浏览器友好的数据格式。

## 当前架构特点

与早期的单文件示例相比，当前仓库已经有两个比较明显的改进：

- 每个可执行程序的主体逻辑都被收敛到独立的 App 类中
- 共享的点云读取逻辑被抽取到了可复用的公共模块中

这使得整个项目更容易扩展、更容易维护，也更适合作为一个结构化的 PCL 学习工程长期演进。
