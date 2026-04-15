PCL Web 点云可视化

**简介**
这个分支只保留一个功能：在服务器端读取点云、做基础裁剪和体素下采样，然后通过网页在本地浏览器中可视化 3D 点云。

适用场景：
- 点云数据在服务器上
- 服务器已经能编译运行 C++ 程序
- 本地只想通过浏览器查看，不想安装 PCL

**保留内容**
- `src/pcd2web.cpp`
  读取 `.bin` / `.pcd` 点云，执行体素下采样和 x/y/z 范围裁剪，导出浏览器可读 JSON。
- `scripts/serve_web.py`
  提供静态网页，并暴露 `/api/convert` 接口，在服务器端调用 `pcd2web`。
- `web/index.html`
  浏览器端 3D 点云查看器。
- `config/pointcloud.json`
  可选默认配置。
- `scripts/build.sh`
  默认只构建 `pcd2web`。

**最小依赖**
服务器端：
- Linux
- `C++17` 编译器
- `CMake >= 3.20`
- `PCL`
- `nlohmann-json`
- `python3`
- 可通过 SSH 登录

本地端：
- `ssh`
- 现代浏览器，例如 Chrome / Edge / Firefox

说明：
- `pcd2web` 实际只需要 PCL 的 `common`、`io`、`filters`
- 当前分支默认不依赖 `OpenCV`
- 当前分支默认不依赖 `PCL visualization`
- 本地端不需要安装 `PCL`

**构建**
如果你已经通过系统包安装好依赖：
```bash
bash scripts/build.sh
./build/pcd2web --help
```

如果你使用 `vcpkg`：
```bash
CMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake \
bash scripts/build.sh
```

**给别人直接使用**
1. 拉取分支：
```bash
git clone -b web-viewer-minimal git@github.com:lingxe01/learn_pcl.git
cd learn_pcl
```

2. 编译：
```bash
bash scripts/build.sh
```

3. 在服务器端启动 Web 服务：
```bash
python3 scripts/serve_web.py --host 127.0.0.1 --port 8080
```

4. 在本地机器做 SSH 端口转发：
```bash
ssh -N -L 8080:127.0.0.1:8080 <你的用户名>@10.93.143.46
```

5. 本地浏览器打开：
```text
http://127.0.0.1:8080/web/
```

**工作原理**
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
    | 提供网页和 /api/convert
    v
服务器 build/pcd2web
    |
    | 读取点云 / 裁剪 / 下采样 / 导出 JSON
    v
output/*.json
    |
    v
web/index.html 在浏览器中绘制 3D 点云
```

关键点：
- 浏览器访问的是本地 `127.0.0.1:8080`
- 实际请求经 SSH 隧道转发到服务器 `127.0.0.1:8080`
- 点云读取、处理、JSON 导出都发生在服务器端
- 本地只负责显示

**命令行使用 `pcd2web`**
示例：
```bash
./build/pcd2web \
  --input /path/to/pointcloud.bin \
  --output output/web_runtime_pointcloud.json \
  --voxel 0.2,0.2,0.2 \
  --x-range -10,60 \
  --y-range -20,20 \
  --z-range -3,2
```

支持的输入格式：
- `.bin`，KITTI velodyne 格式，单点为 `x y z intensity`
- `.pcd`

支持的参数：
- `--input`
- `--output`
- `--voxel x,y,z`
- `--x-range min,max`
- `--y-range min,max`
- `--z-range min,max`

如果 `config/pointcloud.json` 存在，会先读取默认值；如果不存在，也可以完全依赖命令行或网页传参。

**网页里能调什么**
网页当前支持：
- 服务器端点云路径
- 体素尺寸 `x,y,z`
- `x/y/z` 三个方向的范围裁剪
- 颜色模式
- 点大小
- 点透明度
- 重置视角
- 仅重载 JSON

建议的初始参数：
- 点云路径：`data/your_pointcloud.bin`
- 体素尺寸：`0.2,0.2,0.2`
- `x_range`：`-10,60`
- `y_range`：`-20,20`
- `z_range`：`-3,2`

**输出 JSON 格式**
`pcd2web` 导出的 JSON 采用扁平数组格式：

```json
{
  "meta": {
    "format": "xyzi_flat_array",
    "point_stride": 4,
    "raw_point_count": 115384,
    "filtered_point_count": 107195,
    "point_count": 12953,
    "source_path": "/path/to/pointcloud.bin",
    "voxel_leaf_size": [0.25, 0.25, 0.25],
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

这样做的原因：
- JSON 更小
- 浏览器遍历更快
- 前端实现更简单

**最短流程**
1. 服务器端：
```bash
bash scripts/build.sh
python3 scripts/serve_web.py --host 127.0.0.1 --port 8080
```

2. 本地端：
```bash
ssh -N -L 8080:127.0.0.1:8080 <你的用户名>@10.93.143.46
```

3. 浏览器打开：
```text
http://127.0.0.1:8080/web/
```

4. 在网页中填写服务器上的点云路径，然后点击“转换并加载点云”。

**常见问题**
- 打不开网页：
  先确认服务器端 `scripts/serve_web.py` 是否在运行，再确认本地 SSH 端口转发是否还在连接中。
- 转换失败：
  先确认网页里填的是服务器端路径，不是本地路径。
- 页面为空：
  很可能是范围裁剪过严，先清空 `x/y/z range` 再试。
- 页面卡顿：
  先把体素尺寸调大，例如从 `0.1` 改到 `0.2` 或 `0.3`。
