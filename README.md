PCL Web 点云可视化（最小分支）

**简介**
这个分支专门用于“服务器端处理点云，本地浏览器通过 SSH 端口转发查看 3D 点云”。
目标不是覆盖全部 PCL 示例，而是把 Web 可视化链路收敛到最小可用集合，方便别人直接拉取代码后使用。

**分支定位**
- 默认只构建 `pcd2web`
- 默认只依赖 `PCL + nlohmann-json + Python3`
- 默认通过 `web/index.html` + `scripts/serve_web.py` 提供浏览器查看能力
- 默认支持网页端直接输入点云路径、体素尺寸、x/y/z 范围

**环境与依赖**
- 服务器端:
  - Linux
  - GCC / Clang，支持 `C++17`
  - `CMake >= 3.20`
  - `PCL`
  - `nlohmann-json`
  - `python3`
  - 可通过 SSH 登录
- 本地端:
  - `ssh` 客户端
  - 浏览器，例如 Chrome / Edge / Firefox

**最小依赖说明**
- `pcd2web` 实际只需要 PCL 的 `common`、`io`、`filters` 组件
- 这个分支默认不要求 `PCL visualization`
- 这个分支默认不要求 `OpenCV`
- 本地端不需要安装 `PCL`

**构建方式**
1. 如果你本机已经有 `vcpkg`，可以直接用工具链构建:
```bash
CMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake \
bash scripts/build.sh
```

2. 如果依赖已经通过系统包安装好，直接执行:
```bash
bash scripts/build.sh
./build/pcd2web --help
```

3. `scripts/build.sh` 默认只编译 `pcd2web`，这是这个分支推荐的入口。

**给别人直接使用的最短路径**
```bash
git clone -b web-viewer-minimal git@github.com:lingxe01/learn_pcl.git
cd learn_pcl
bash scripts/build.sh
python3 scripts/serve_web.py --host 127.0.0.1 --port 8080
```

然后在本地执行：
```bash
ssh -N -L 8080:127.0.0.1:8080 <你的用户名>@10.93.143.46
```

浏览器打开：
```text
http://127.0.0.1:8080/web/
```

**当前分支的最小 CMake 形态**
```cmake
cmake_minimum_required(VERSION 3.20)
project(pointcloud_web_viewer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL CONFIG REQUIRED COMPONENTS common io filters)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(pcd2web src/pcd2web.cpp)
target_include_directories(pcd2web PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcd2web PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcd2web PRIVATE ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)
```

**目录结构**
- `src/pcd2web.cpp`：服务器端点云转 JSON
- `scripts/serve_web.py`：Web 服务与转换接口
- `web/index.html`：浏览器端 3D 点云查看器
- `config/pointcloud.json`：可选默认配置
- `scripts/build.sh`：最小化构建脚本
- `build/`：构建输出（本地生成）

**学习路线**
1. 点云基础: 点类型、坐标系、常见数据格式。
2. IO 模块: 读取/写入 `PCD` 与 `PLY`。
3. 过滤: 体素降采样、直通滤波、统计离群点移除。
4. 特征: 法线估计、FPFH 等。
5. 配准: ICP、NDT。
6. 分割: RANSAC 平面分割、聚类。
7. 可视化: `PCLVisualizer` 的基本使用。

**常用 API 速查**
- 点云与点类型（`pcl/point_types.h`, `pcl/point_cloud.h`）: `pcl::PointCloud<T>`, `pcl::PointXYZ`, `pcl::PointXYZRGB`, `pcl::PointNormal`
- IO（`pcl/io/pcd_io.h`, `pcl/io/ply_io.h`）: `pcl::io::loadPCDFile`, `pcl::io::savePCDFileASCII`, `pcl::io::savePCDFileBinary`, `pcl::io::loadPLYFile`, `pcl::io::savePLYFileBinary`
- 过滤（`pcl/filters/*`）: `pcl::VoxelGrid`, `pcl::PassThrough`, `pcl::StatisticalOutlierRemoval`, `pcl::RadiusOutlierRemoval`
- 法线（`pcl/features/normal_3d.h`）: `pcl::NormalEstimation`, `pcl::NormalEstimationOMP`
- 特征（`pcl/features/*`）: `pcl::FPFHEstimation`, `pcl::SHOTEstimation`
- 分割（`pcl/segmentation/*`）: `pcl::SACSegmentation`, `pcl::ExtractIndices`, `pcl::EuclideanClusterExtraction`
- 配准（`pcl/registration/*`）: `pcl::IterativeClosestPoint`, `pcl::NormalDistributionsTransform`
- 搜索结构（`pcl/search/*`, `pcl/kdtree/*`）: `pcl::search::KdTree`, `pcl::KdTreeFLANN`
- 变换与几何（`pcl/common/*`）: `pcl::transformPointCloud`, `pcl::getMinMax3D`, `pcl::compute3DCentroid`
- 可视化（`pcl/visualization/*`）: `pcl::visualization::PCLVisualizer`, `pcl::visualization::CloudViewer`

**示例清单（规划）**
1. 读取与保存点云（`src/lesson_01_io.cpp`）。
2. 体素降采样与直通滤波。
3. 法线估计与可视化。
4. 平面分割与聚类。
5. ICP 配准示例。
6. Web 方式查看点云（`src/pcd2web.cpp` + `web/index.html`）。

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
- 编译报缺少 `nlohmann_json`: 确认已经安装 `nlohmann-json`，或者通过 `vcpkg` 提供工具链。
- 运行时找不到库: 检查 `LD_LIBRARY_PATH` 或系统动态库路径。

**Web 查看点云**
这一套功能的目标是：
- 代码和数据都放在服务器端运行。
- 本地电脑不需要安装 PCL，也不需要拷贝点云文件。
- 本地只通过浏览器访问一个 Web 页面，就能查看服务器上的 3D 点云。

**原理**
整体链路如下：

```text
本地浏览器
    |
    | 访问 http://127.0.0.1:8080/web/
    v
SSH 端口转发
    |
    | 将本地 8080 转发到服务器 127.0.0.1:8080
    v
服务器 scripts/serve_web.py
    |
    | 1. 提供 web/index.html 和输出 JSON
    | 2. 接收 /api/convert 请求
    v
服务器 build/pcd2web
    |
    | 读取 .bin / .pcd 点云
    | 体素下采样
    | 按 x/y/z 范围裁剪
    | 导出扁平 JSON
    v
output/*.json
    |
    v
web/index.html 在浏览器中加载并绘制 3D 点云
```

再从“服务器端 / 本地端”职责角度看，可以把流程拆成下面这样：

```text
+----------------------------+                  +----------------------------+
| 本地电脑                   |                  | 服务器 10.93.143.46        |
|----------------------------|                  |----------------------------|
| 浏览器                     |                  | scripts/serve_web.py       |
| 打开 http://127.0.0.1:8080 |                  | 监听 127.0.0.1:8080        |
|                            |                  |                            |
| ssh -N -L 8080:127.0.0.1   | <------------->  | build/pcd2web              |
| :8080 user@10.93.143.46    |     SSH 隧道     | 读取 .bin/.pcd             |
|                            |                  | 下采样 / 裁剪 / 导出 JSON  |
| 本地不存点云数据           |                  |                            |
| 不需要本地安装 PCL         |                  | data/*.bin, output/*.json  |
+----------------------------+                  +----------------------------+
```

可以把它理解成：
- 浏览器虽然访问的是本地 `127.0.0.1:8080`。
- 但这个端口实际被 SSH 映射到了服务器 `127.0.0.1:8080`。
- 所以页面、接口、点云转换、JSON 文件都仍然发生在服务器端。
- 本地只负责显示，不负责点云处理。

核心点有 3 个：
- `src/pcd2web.cpp` 负责把服务器上的点云转换成浏览器容易读取的 JSON。
- `scripts/serve_web.py` 负责提供静态网页，并暴露 `/api/convert` 接口，在服务器端调用 `build/pcd2web`。
- `web/index.html` 负责在浏览器中发请求、加载 JSON、用 Canvas 渲染 3D 点云。

之所以要加 SSH 端口转发，是因为 Python Web 服务只监听服务器本机的 `127.0.0.1`。这样服务不会直接暴露到局域网或公网，本地通过 SSH 安全地把这个端口“接”过来即可。

**数据流**
1. 你在网页里填服务器上的点云路径，例如 `/home/lingzhiying/learn/learn_pcl/data/kitti/training/velodyne/000000.bin`。
2. 你在网页里可选设置体素尺寸，例如 `0.2,0.2,0.2`。
3. 你在网页里可选设置点云范围：
   - `x_range`：前后范围
   - `y_range`：左右范围
   - `z_range`：高低范围
4. 网页把这些参数以 JSON 形式 `POST` 到服务器的 `/api/convert`。
5. 服务器调用：
```bash
./build/pcd2web \
  --input <点云路径> \
  --output <输出 JSON 路径> \
  --voxel <x,y,z> \
  --x-range <min,max> \
  --y-range <min,max> \
  --z-range <min,max>
```
6. `pcd2web` 在服务器端完成裁剪、下采样和导出。
7. 浏览器再去加载这个输出 JSON，并把点云显示出来。

**涉及文件**
- `src/pcd2web.cpp`
  负责读取 `.bin` / `.pcd` 点云，做体素下采样、按范围过滤，并导出浏览器可读 JSON。
- `scripts/serve_web.py`
  负责提供网页与 JSON 文件，并把网页参数转成 `pcd2web` 命令行参数。
- `web/index.html`
  负责 Web 交互和 3D 渲染。
- `config/pointcloud.json`
  负责默认输入路径、默认输出路径等配置。

**实现这个功能需要的最小环境依赖**
如果只关心“服务器端转换 + 本地浏览器查看”这条链路，最小依赖可以分成服务器端和本地端两部分。

服务器端最小运行依赖：
- Linux
- `python3`：运行 `scripts/serve_web.py`
- `ssh` 服务可登录：供本地做端口转发
- 现代 C++ 编译器，支持 `C++17`
- `CMake >= 3.20`
- `PCL`
- `nlohmann-json`

服务器端构建时，`pcd2web` 实际使用到的 PCL 模块主要是：
- `common`
- `io`
- `filters`

说明：
- 当前分支默认只构建 `pcd2web`。
- 当前分支默认不要求 `OpenCV`，也不要求 `PCL visualization`。
- 如果你只是想把服务器点云发到本地浏览器查看，这个最小分支已经足够。

本地最小运行依赖：
- `ssh` 客户端
- 现代浏览器，例如 Chrome / Edge / Firefox

也就是说，这个功能的一个重要特点是：
- 服务器端负责编译、点云读取、裁剪、下采样、导出 JSON
- 本地只负责 SSH 转发和浏览器显示
- 本地不需要安装 PCL，不需要下载点云数据

**服务器端怎么用**
1. 编译：
```bash
bash scripts/build.sh
```

2. 如果只想命令行测试转换，可以直接运行：
```bash
./build/pcd2web \
  --input /home/lingzhiying/learn/learn_pcl/data/kitti/training/velodyne/000000.bin \
  --output output/web_runtime_pointcloud.json \
  --voxel 0.2,0.2,0.2 \
  --x-range -10,60 \
  --y-range -20,20 \
  --z-range -3,2
```
当前支持 `.bin` 与 `.pcd`。

3. 启动 Web 服务：
```bash
python3 scripts/serve_web.py --host 127.0.0.1 --port 8080
```

说明：
- `--host 127.0.0.1` 表示只监听服务器本机回环地址，配合 SSH 转发更安全。
- `--port 8080` 是服务器端实际监听端口。

**本地怎么用**
假设：
- 服务器 IP 是 `10.93.143.46`
- 本地机器通过 SSH 登录服务器

1. 在本地机器执行端口转发：
```bash
ssh -N -L 8080:127.0.0.1:8080 <你的用户名>@10.93.143.46
```

这条命令的含义是：
- 本地监听 `127.0.0.1:8080`
- 通过 SSH，把这个端口转发到服务器的 `127.0.0.1:8080`

2. 本地浏览器打开：
```text
http://127.0.0.1:8080/web/
```

此时虽然浏览器访问的是本地 `127.0.0.1:8080`，但实际请求会经 SSH 隧道转发到服务器端的 Web 服务。

**网页里能做什么**
当前网页支持：
- 直接填写服务器端点云路径
- 直接填写体素尺寸 `x,y,z`
- 直接填写 `x/y/z` 三个方向的裁剪范围 `min,max`
- 重新触发服务器端转换
- 只重载已有 JSON
- 调整颜色模式、点大小、透明度
- 鼠标拖动旋转、滚轮缩放、双击重置视角

常用参数建议：
- 体素尺寸 `0.1,0.1,0.1`：点更多，细节更多，但网页更重
- 体素尺寸 `0.2,0.2,0.2`：比较均衡
- 体素尺寸 `0.3,0.3,0.3`：更轻，适合远程浏览
- `x_range` 常见可先设为 `-10,60`
- `y_range` 常见可先设为 `-20,20`
- `z_range` 常见可先设为 `-3,2`

**只加载指定 JSON**
如果你已经提前生成过 JSON，也可以不走网页转换接口，直接通过查询参数加载：
```text
http://127.0.0.1:8080/web/?data=/output/your_pointcloud.json
```

**输出 JSON 的格式**
`pcd2web` 导出的 JSON 采用扁平数组格式：
```json
{
  "meta": {
    "format": "xyzi_flat_array",
    "point_stride": 4,
    "raw_point_count": 115384,
    "filtered_point_count": 107195,
    "point_count": 9900,
    "source_path": "...",
    "voxel_leaf_size": [0.2, 0.2, 0.2],
    "range_filter": {
      "x": [-10.0, 60.0],
      "y": [-20.0, 20.0],
      "z": [-3.0, 2.0]
    },
    "bounds": {
      "x": [-9.9, 59.8],
      "y": [-19.7, 19.8],
      "z": [-2.8, 1.9]
    }
  },
  "points": [x0, y0, z0, i0, x1, y1, z1, i1]
}
```

采用扁平数组而不是对象数组，主要是为了：
- 减少 JSON 体积
- 降低浏览器解析开销
- 简化前端遍历和绘制逻辑

**一次完整流程示例**
1. 服务器端启动服务：
```bash
python3 scripts/serve_web.py --host 127.0.0.1 --port 8080
```

2. 本地机器建立 SSH 端口转发：
```bash
ssh -N -L 8080:127.0.0.1:8080 <你的用户名>@10.93.143.46
```

3. 本地浏览器打开：
```text
http://127.0.0.1:8080/web/
```

4. 在网页中填写：
- 点云路径：`/home/lingzhiying/learn/learn_pcl/data/kitti/training/velodyne/000000.bin`
- 体素尺寸：`0.2,0.2,0.2`
- `x_range`：`-10,60`
- `y_range`：`-20,20`
- `z_range`：`-3,2`

5. 点击“转换并加载点云”。

6. 浏览器会看到服务器端处理后的 3D 点云。

**实际操作速查**
如果你只是想最快跑通，直接按下面 3 步执行。

1. 服务器端编译并启动服务：
```bash
bash scripts/build.sh
python3 scripts/serve_web.py --host 127.0.0.1 --port 8080
```

2. 本地机器建立 SSH 端口转发：
```bash
ssh -N -L 8080:127.0.0.1:8080 <你的用户名>@10.93.143.46
```

3. 本地浏览器打开：
```text
http://127.0.0.1:8080/web/
```

建议第一次先在网页中填这一组参数：
- 点云路径：`/home/lingzhiying/learn/learn_pcl/data/kitti/training/velodyne/000000.bin`
- 体素尺寸：`0.2,0.2,0.2`
- `x_range`：`-10,60`
- `y_range`：`-20,20`
- `z_range`：`-3,2`

然后点击“转换并加载点云”。

**排查方法**
- 浏览器打不开页面：
  先确认服务器端 `scripts/serve_web.py` 是否在运行，再确认本地 SSH 转发命令是否还在保持连接。
- 点云转换失败：
  先确认网页里填的是服务器端文件路径，不是本地机器路径。
- 页面显示为空：
  很可能是范围裁剪太严，导致 `x/y/z` 过滤后没有点了。先清空范围输入，再试一次。
- 页面很卡：
  先把体素尺寸调大，例如从 `0.1` 提到 `0.2` 或 `0.3`。
- 只想看前方区域：
  优先限制 `x_range` 和 `y_range`，这样比只调相机更直接。

**下一步**
- 在 `src/` 中逐步加入示例，并在 `notes/` 中记录算法要点与参数选择经验。
- 增加数据集与可视化示例，形成完整流程。
