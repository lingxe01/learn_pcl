#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

namespace {

// Web 导出配置。
// 这里沿用同一个 JSON 配置文件，避免为 Web 查看单独维护另一套参数。
struct WebConfig {
  std::string pointcloud_path;
  std::string web_pointcloud_path = "output/kitti_000000_web.json";
  float voxel_leaf_x = 0.2f;
  float voxel_leaf_y = 0.2f;
  float voxel_leaf_z = 0.2f;
  bool use_x_range = false;
  bool use_y_range = false;
  bool use_z_range = false;
  float x_min = 0.0f;
  float x_max = 0.0f;
  float y_min = 0.0f;
  float y_max = 0.0f;
  float z_min = 0.0f;
  float z_max = 0.0f;
};

// 读取 KITTI velodyne 原始点云。
// 每个点按 x, y, z, intensity 四个 float 顺序存储。
bool LoadKittiBin(const std::string& file_path,
                  pcl::PointCloud<pcl::PointXYZI>* cloud) {
  if (!cloud) {
    return false;
  }

  std::ifstream in(file_path, std::ios::binary);
  if (!in) {
    std::cerr << "Failed to open point cloud: " << file_path << "\n";
    return false;
  }

  in.seekg(0, std::ios::end);
  const std::streamsize file_size = in.tellg();
  in.seekg(0, std::ios::beg);
  if (file_size <= 0 || file_size % (4 * static_cast<std::streamsize>(sizeof(float))) != 0) {
    std::cerr << "Unexpected KITTI bin size: " << file_size << " bytes\n";
  }

  const std::size_t point_count =
      static_cast<std::size_t>(file_size / (4 * sizeof(float)));
  cloud->clear();
  cloud->reserve(point_count);

  float data[4];
  while (in.read(reinterpret_cast<char*>(data), sizeof(data))) {
    pcl::PointXYZI point;
    point.x = data[0];
    point.y = data[1];
    point.z = data[2];
    point.intensity = data[3];
    cloud->push_back(point);
  }
  return !cloud->empty();
}

// 统一加载输入点云。
// 当前支持：
// 1. KITTI 原始二进制 .bin
// 2. PCL 常见的 .pcd
bool LoadPointCloud(const std::string& file_path,
                    pcl::PointCloud<pcl::PointXYZI>* cloud) {
  if (!cloud) {
    return false;
  }

  const std::filesystem::path path(file_path);
  const std::string ext = path.extension().string();
  if (ext == ".bin") {
    return LoadKittiBin(file_path, cloud);
  }
  if (ext == ".pcd") {
    if (pcl::io::loadPCDFile(file_path, *cloud) != 0) {
      std::cerr << "Failed to open PCD point cloud: " << file_path << "\n";
      return false;
    }
    return !cloud->empty();
  }

  std::cerr << "Unsupported point cloud format: " << file_path << "\n";
  std::cerr << "Currently supported: .bin, .pcd\n";
  return false;
}

// 解析 [a, b, c] 形式的三元数组。
bool ParseFloatTriplet(const nlohmann::json& value, float* a, float* b, float* c) {
  if (!a || !b || !c || !value.is_array() || value.size() != 3) {
    return false;
  }
  *a = value.at(0).get<float>();
  *b = value.at(1).get<float>();
  *c = value.at(2).get<float>();
  return true;
}

// 解析 [min, max] 形式的二元数组，用于点云范围裁剪。
bool ParseFloatPair(const nlohmann::json& value, float* a, float* b) {
  if (!a || !b || !value.is_array() || value.size() != 2) {
    return false;
  }
  *a = value.at(0).get<float>();
  *b = value.at(1).get<float>();
  return true;
}

// 解析命令行中的 "min,max" 字符串。
bool ParseFloatPairString(const std::string& text, float* min_value, float* max_value) {
  if (!min_value || !max_value) {
    return false;
  }

  std::stringstream ss(text);
  char comma = 0;
  if (!(ss >> *min_value >> comma >> *max_value) || comma != ',') {
    return false;
  }
  return true;
}

// 对配置做基础合法性检查，避免前端传入非法参数后生成不可预期结果。
bool ValidateConfig(const WebConfig& config) {
  if (config.pointcloud_path.empty()) {
    std::cerr << "Empty point cloud input path\n";
    return false;
  }
  if (config.web_pointcloud_path.empty()) {
    std::cerr << "Empty web output path\n";
    return false;
  }
  if (config.voxel_leaf_x <= 0.0f || config.voxel_leaf_y <= 0.0f ||
      config.voxel_leaf_z <= 0.0f) {
    std::cerr << "voxel leaf size must be positive\n";
    return false;
  }
  if (config.use_x_range && config.x_min > config.x_max) {
    std::cerr << "x range must satisfy min <= max\n";
    return false;
  }
  if (config.use_y_range && config.y_min > config.y_max) {
    std::cerr << "y range must satisfy min <= max\n";
    return false;
  }
  if (config.use_z_range && config.z_min > config.z_max) {
    std::cerr << "z range must satisfy min <= max\n";
    return false;
  }
  return true;
}

// 读取配置文件。
bool LoadConfig(const std::string& config_path, WebConfig* config) {
  if (!config) {
    return false;
  }

  try {
    std::ifstream cfg(config_path);
    if (!cfg) {
      std::cerr << "Failed to open config: " << config_path << "\n";
      return false;
    }

    nlohmann::json j;
    cfg >> j;

    config->pointcloud_path = j.at("pointcloud_path").get<std::string>();
    if (config->pointcloud_path.empty()) {
      std::cerr << "Empty pointcloud_path in config: " << config_path << "\n";
      return false;
    }

    if (j.contains("web_pointcloud_path")) {
      config->web_pointcloud_path = j.at("web_pointcloud_path").get<std::string>();
      if (config->web_pointcloud_path.empty()) {
        std::cerr << "Empty web_pointcloud_path in config: " << config_path << "\n";
        return false;
      }
    }

    if (j.contains("voxel_leaf_size") &&
        !ParseFloatTriplet(j.at("voxel_leaf_size"),
                           &config->voxel_leaf_x,
                           &config->voxel_leaf_y,
                           &config->voxel_leaf_z)) {
      std::cerr << "voxel_leaf_size must be an array of 3 numbers\n";
      return false;
    }

    if (j.contains("web_x_range")) {
      config->use_x_range = true;
      if (!ParseFloatPair(j.at("web_x_range"), &config->x_min, &config->x_max)) {
        std::cerr << "web_x_range must be an array of 2 numbers\n";
        return false;
      }
    }

    if (j.contains("web_y_range")) {
      config->use_y_range = true;
      if (!ParseFloatPair(j.at("web_y_range"), &config->y_min, &config->y_max)) {
        std::cerr << "web_y_range must be an array of 2 numbers\n";
        return false;
      }
    }

    if (j.contains("web_z_range")) {
      config->use_z_range = true;
      if (!ParseFloatPair(j.at("web_z_range"), &config->z_min, &config->z_max)) {
        std::cerr << "web_z_range must be an array of 2 numbers\n";
        return false;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse config: " << config_path << "\n";
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }

  return ValidateConfig(*config);
}

// 解析命令行参数，允许服务端按需指定输入点云路径和输出 JSON 路径。
// 这样 Web 页面就可以通过后端接口动态触发转换，而不必每次改配置文件。
bool ApplyCommandLineArgs(int argc, char** argv, WebConfig* config) {
  if (!config) {
    return false;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto read_value = [&](const std::string& option) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << option << "\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--input") {
      const char* value = read_value(arg);
      if (!value) {
        return false;
      }
      config->pointcloud_path = value;
    } else if (arg == "--output") {
      const char* value = read_value(arg);
      if (!value) {
        return false;
      }
      config->web_pointcloud_path = value;
    } else if (arg == "--voxel") {
      const char* value = read_value(arg);
      if (!value) {
        return false;
      }
      std::stringstream ss(value);
      char comma1 = 0;
      char comma2 = 0;
      if (!(ss >> config->voxel_leaf_x >> comma1 >> config->voxel_leaf_y >> comma2 >>
            config->voxel_leaf_z) ||
          comma1 != ',' || comma2 != ',') {
        std::cerr << "Invalid --voxel format, expected x,y,z\n";
        return false;
      }
    } else if (arg == "--x-range") {
      const char* value = read_value(arg);
      if (!value || !ParseFloatPairString(value, &config->x_min, &config->x_max)) {
        std::cerr << "Invalid --x-range format, expected min,max\n";
        return false;
      }
      config->use_x_range = true;
    } else if (arg == "--y-range") {
      const char* value = read_value(arg);
      if (!value || !ParseFloatPairString(value, &config->y_min, &config->y_max)) {
        std::cerr << "Invalid --y-range format, expected min,max\n";
        return false;
      }
      config->use_y_range = true;
    } else if (arg == "--z-range") {
      const char* value = read_value(arg);
      if (!value || !ParseFloatPairString(value, &config->z_min, &config->z_max)) {
        std::cerr << "Invalid --z-range format, expected min,max\n";
        return false;
      }
      config->use_z_range = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: pcd2web [--input path] [--output path] [--voxel x,y,z]"
          << " [--x-range min,max] [--y-range min,max] [--z-range min,max]\n";
      return false;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return false;
    }
  }

  return ValidateConfig(*config);
}

}  // namespace

int main(int argc, char** argv) {
  // 1. 读取配置
  const std::string config_path = "config/pointcloud.json";
  WebConfig config;
  if (!LoadConfig(config_path, &config)) {
    return 1;
  }
  if (!ApplyCommandLineArgs(argc, argv, &config)) {
    return 1;
  }

  // 2. 加载点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!LoadPointCloud(config.pointcloud_path, cloud.get())) {
    return 1;
  }

  // 3. 按需做空间范围裁剪。
  // 这样网页端可以直接控制只看前方、近处或某个高度层的点云区域。
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  cloud_filtered->reserve(cloud->size());
  for (const auto& point : cloud->points) {
    if (config.use_x_range && (point.x < config.x_min || point.x > config.x_max)) {
      continue;
    }
    if (config.use_y_range && (point.y < config.y_min || point.y > config.y_max)) {
      continue;
    }
    if (config.use_z_range && (point.z < config.z_min || point.z > config.z_max)) {
      continue;
    }
    cloud_filtered->push_back(point);
  }

  if (cloud_filtered->empty()) {
    std::cerr << "No points remain after applying the selected range filters\n";
    return 1;
  }

  // 4. 做一次体素下采样，减少浏览器端渲染压力。
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_down(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(cloud_filtered);
  voxel.setLeafSize(config.voxel_leaf_x, config.voxel_leaf_y, config.voxel_leaf_z);
  voxel.filter(*cloud_down);

  if (cloud_down->empty()) {
    std::cerr << "No points remain after voxel downsampling, try a smaller voxel size\n";
    return 1;
  }

  // 5. 统计边界范围，方便前端初始化相机视角。
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float min_z = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();

  // 采用扁平数组导出：points = [x0, y0, z0, i0, x1, y1, z1, i1, ...]
  // 这样比数组嵌套更紧凑，浏览器端也更容易直接遍历。
  nlohmann::json points = nlohmann::json::array();

  for (const auto& point : cloud_down->points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);

    points.push_back(point.x);
    points.push_back(point.y);
    points.push_back(point.z);
    points.push_back(point.intensity);
  }

  // 6. 组织导出 JSON。
  nlohmann::json root;
  root["meta"] = {
      {"format", "xyzi_flat_array"},
      {"point_stride", 4},
      {"raw_point_count", cloud->size()},
      {"filtered_point_count", cloud_filtered->size()},
      {"point_count", cloud_down->size()},
      {"source_path", config.pointcloud_path},
      {"voxel_leaf_size", {config.voxel_leaf_x, config.voxel_leaf_y, config.voxel_leaf_z}},
      {"range_filter", {
           {"x", config.use_x_range ? nlohmann::json::array({config.x_min, config.x_max})
                                    : nlohmann::json()},
           {"y", config.use_y_range ? nlohmann::json::array({config.y_min, config.y_max})
                                    : nlohmann::json()},
           {"z", config.use_z_range ? nlohmann::json::array({config.z_min, config.z_max})
                                    : nlohmann::json()},
       }},
      {"bounds", {
           {"x", {min_x, max_x}},
           {"y", {min_y, max_y}},
           {"z", {min_z, max_z}},
       }},
  };
  root["points"] = std::move(points);

  // 7. 写入磁盘，供 Web 页面通过 HTTP 直接加载。
  std::filesystem::path output_path(config.web_pointcloud_path);
  if (!output_path.parent_path().empty()) {
    std::filesystem::create_directories(output_path.parent_path());
  }

  std::ofstream out(config.web_pointcloud_path);
  if (!out) {
    std::cerr << "Failed to open output file: " << config.web_pointcloud_path << "\n";
    return 1;
  }
  out << root.dump();

  std::cout << "Loaded KITTI cloud: " << cloud->size() << " points\n";
  std::cout << "Range filtered cloud: " << cloud_filtered->size() << " points\n";
  std::cout << "Downsampled cloud: " << cloud_down->size() << " points\n";
  std::cout << "Saved web point cloud: " << config.web_pointcloud_path << "\n";

  return 0;
}
