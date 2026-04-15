#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace {

// 统一管理 BEV 生成所需的配置项。
// 这里保留一个主输出路径，单通道图像路径由主路径自动推导，避免配置过长。
struct BevConfig {
  std::string pointcloud_path;
  std::string bev_image_path = "output/kitti_000000_bev.png";
  float voxel_leaf_x = 0.2f;
  float voxel_leaf_y = 0.2f;
  float voxel_leaf_z = 0.2f;
  float resolution = 0.1f;
  float x_min = 0.0f;
  float x_max = 70.4f;
  float y_min = -40.0f;
  float y_max = 40.0f;
  float z_min = -3.0f;
  float z_max = 1.0f;
};

// 每个 BEV 网格单元统计三类信息：
// 1. 最高点高度，用于构建 height map
// 2. 最大反射强度，用于构建 intensity map
// 3. 落入该网格的点数量，用于构建 density map
struct CellStat {
  float max_height = -std::numeric_limits<float>::infinity();
  float max_intensity = 0.0f;
  int count = 0;
};

// 读取 KITTI 原始点云。KITTI 的 velodyne 文件按 float4 顺序存储：
// x, y, z, intensity。
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

// 解析长度为 3 的浮点数组，用于体素尺寸等配置。
bool ParseFloatTriplet(const nlohmann::json& value, float* a, float* b, float* c) {
  if (!a || !b || !c || !value.is_array() || value.size() != 3) {
    return false;
  }
  *a = value.at(0).get<float>();
  *b = value.at(1).get<float>();
  *c = value.at(2).get<float>();
  return true;
}

// 解析长度为 2 的浮点数组，用于 x/y/z 范围配置。
bool ParseFloatPair(const nlohmann::json& value, float* a, float* b) {
  if (!a || !b || !value.is_array() || value.size() != 2) {
    return false;
  }
  *a = value.at(0).get<float>();
  *b = value.at(1).get<float>();
  return true;
}

// 从主输出路径派生兄弟文件，例如：
// output/kitti.png -> output/kitti_height.png
std::string DeriveSiblingPath(const std::string& file_path, const std::string& suffix) {
  const std::filesystem::path path(file_path);
  return (path.parent_path() / (path.stem().string() + suffix + path.extension().string()))
      .string();
}

// 将 0~1 的归一化值安全映射到 0~255。
unsigned char ToByte(float normalized_value) {
  return static_cast<unsigned char>(
      std::round(std::clamp(normalized_value, 0.0f, 1.0f) * 255.0f));
}

// 配置加载与合法性检查。
bool LoadConfig(const std::string& config_path, BevConfig* config) {
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

    if (j.contains("voxel_leaf_size") &&
        !ParseFloatTriplet(j.at("voxel_leaf_size"),
                           &config->voxel_leaf_x,
                           &config->voxel_leaf_y,
                           &config->voxel_leaf_z)) {
      std::cerr << "voxel_leaf_size must be an array of 3 numbers\n";
      return false;
    }

    if (j.contains("bev_image_path")) {
      config->bev_image_path = j.at("bev_image_path").get<std::string>();
      if (config->bev_image_path.empty()) {
        std::cerr << "Empty bev_image_path in config: " << config_path << "\n";
        return false;
      }
    }

    if (j.contains("bev_resolution")) {
      config->resolution = j.at("bev_resolution").get<float>();
    }
    if (j.contains("bev_x_range") &&
        !ParseFloatPair(j.at("bev_x_range"), &config->x_min, &config->x_max)) {
      std::cerr << "bev_x_range must be an array of 2 numbers\n";
      return false;
    }
    if (j.contains("bev_y_range") &&
        !ParseFloatPair(j.at("bev_y_range"), &config->y_min, &config->y_max)) {
      std::cerr << "bev_y_range must be an array of 2 numbers\n";
      return false;
    }
    if (j.contains("bev_z_range") &&
        !ParseFloatPair(j.at("bev_z_range"), &config->z_min, &config->z_max)) {
      std::cerr << "bev_z_range must be an array of 2 numbers\n";
      return false;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse config: " << config_path << "\n";
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }

  if (config->resolution <= 0.0f) {
    std::cerr << "bev_resolution must be positive\n";
    return false;
  }
  if (config->x_max <= config->x_min || config->y_max <= config->y_min ||
      config->z_max <= config->z_min) {
    std::cerr << "Invalid BEV ranges in config\n";
    return false;
  }
  return true;
}

// 在融合后的 BEV 图上叠加参考网格。
// 常见自动驾驶可视化会显示固定间隔的距离线，便于直观判断尺度。
void DrawRangeGrid(const BevConfig& config, cv::Mat* image) {
  if (!image || image->empty()) {
    return;
  }

  const int width = image->cols;
  const int height = image->rows;
  const float step_meter = 10.0f;
  const cv::Scalar grid_color(40, 40, 40);

  for (float x = config.x_min; x <= config.x_max; x += step_meter) {
    const int row = height - 1 - static_cast<int>((x - config.x_min) / config.resolution);
    if (row >= 0 && row < height) {
      cv::line(*image, cv::Point(0, row), cv::Point(width - 1, row), grid_color, 1);
    }
  }

  for (float y = config.y_min; y <= config.y_max; y += step_meter) {
    const int col = static_cast<int>((y - config.y_min) / config.resolution);
    if (col >= 0 && col < width) {
      cv::line(*image, cv::Point(col, 0), cv::Point(col, height - 1), grid_color, 1);
    }
  }
}

// 在图像底部中心画一个简化自车轮廓。
// 这是自动驾驶场景下常见的 BEV 视觉锚点。
void DrawEgoVehicle(const BevConfig& config, cv::Mat* image) {
  if (!image || image->empty()) {
    return;
  }

  const float ego_length_meter = 4.5f;
  const float ego_width_meter = 1.8f;
  const int height = image->rows;

  const int car_center_col =
      static_cast<int>((0.0f - config.y_min) / config.resolution);
  const int car_bottom_row =
      height - 1 - static_cast<int>((0.0f - config.x_min) / config.resolution);
  const int half_width_px =
      std::max(1, static_cast<int>(std::round((ego_width_meter * 0.5f) / config.resolution)));
  const int length_px =
      std::max(1, static_cast<int>(std::round(ego_length_meter / config.resolution)));

  const cv::Rect ego_rect(
      std::max(0, car_center_col - half_width_px),
      std::max(0, car_bottom_row - length_px),
      std::min(image->cols - std::max(0, car_center_col - half_width_px), half_width_px * 2),
      std::min(image->rows - std::max(0, car_bottom_row - length_px), length_px));

  if (ego_rect.width > 0 && ego_rect.height > 0) {
    cv::rectangle(*image, ego_rect, cv::Scalar(255, 255, 255), 1);
    cv::line(*image,
             cv::Point(car_center_col, ego_rect.y),
             cv::Point(car_center_col, ego_rect.y + ego_rect.height),
             cv::Scalar(255, 255, 255),
             1);
  }
}

}  // namespace

int main() {
  // 1. 读取配置
  const std::string config_path = "config/pointcloud.json";
  BevConfig config;
  if (!LoadConfig(config_path, &config)) {
    return 1;
  }

  // 2. 加载 KITTI 点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!LoadKittiBin(config.pointcloud_path, cloud.get())) {
    return 1;
  }

  // 3. 体素下采样
  // 自动驾驶 BEV 一般不需要保留所有原始点，先做稀疏化可明显降低网格投影开销。
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_down(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(config.voxel_leaf_x, config.voxel_leaf_y, config.voxel_leaf_z);
  voxel.filter(*cloud_down);

  const int width =
      static_cast<int>(std::ceil((config.y_max - config.y_min) / config.resolution));
  const int height =
      static_cast<int>(std::ceil((config.x_max - config.x_min) / config.resolution));
  if (width <= 0 || height <= 0) {
    std::cerr << "Invalid BEV output size\n";
    return 1;
  }

  // 4. 初始化二维统计网格
  // 行对应前后方向（x），列对应左右方向（y）。
  // 这里保持“前方朝上、车辆位于图像底部中间”的常见自动驾驶展示方式。
  std::vector<CellStat> grid(static_cast<std::size_t>(width) * height);
  int valid_points = 0;

  // 5. 将点云投影到 BEV 平面，并更新每个格子的统计量
  for (const auto& point : cloud_down->points) {
    if (point.x < config.x_min || point.x >= config.x_max ||
        point.y < config.y_min || point.y >= config.y_max ||
        point.z < config.z_min || point.z > config.z_max) {
      continue;
    }

    const int col = static_cast<int>((point.y - config.y_min) / config.resolution);
    const int row = height - 1 -
                    static_cast<int>((point.x - config.x_min) / config.resolution);
    if (row < 0 || row >= height || col < 0 || col >= width) {
      continue;
    }

    CellStat& cell = grid[static_cast<std::size_t>(row) * width + col];
    cell.max_height = std::max(cell.max_height, point.z);
    cell.max_intensity = std::max(cell.max_intensity, point.intensity);
    ++cell.count;
    ++valid_points;
  }

  // 6. 构建三张典型的 BEV 单通道特征图：
  // height map、intensity map、density map。
  // 这三张图是很多 3D 检测/感知工作流中的标准输入表达。
  cv::Mat height_map(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat intensity_map(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat density_map(height, width, CV_8UC1, cv::Scalar(0));
  const float z_span = config.z_max - config.z_min;

  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const CellStat& cell = grid[static_cast<std::size_t>(row) * width + col];
      if (cell.count == 0) {
        continue;
      }

      const float height_norm =
          std::clamp((cell.max_height - config.z_min) / z_span, 0.0f, 1.0f);
      const float intensity_norm = std::clamp(cell.max_intensity, 0.0f, 1.0f);
      const float density_norm =
          std::clamp(std::log1p(static_cast<float>(cell.count)) / std::log(64.0f),
                     0.0f,
                     1.0f);

      height_map.at<unsigned char>(row, col) = ToByte(height_norm);
      intensity_map.at<unsigned char>(row, col) = ToByte(intensity_norm);
      density_map.at<unsigned char>(row, col) = ToByte(density_norm);
    }
  }

  // 7. 融合成常见的三通道自动驾驶 BEV 图：
  // OpenCV 使用 BGR 排列，因此这里按
  // B=intensity, G=height, R=density
  // 的顺序融合。
  cv::Mat bev;
  cv::merge(std::vector<cv::Mat>{intensity_map, height_map, density_map}, bev);

  // 8. 添加尺度网格和自车轮廓，使图像更接近实际自动驾驶调试面板风格。
  DrawRangeGrid(config, &bev);
  DrawEgoVehicle(config, &bev);

  std::filesystem::path output_path(config.bev_image_path);
  if (!output_path.parent_path().empty()) {
    std::filesystem::create_directories(output_path.parent_path());
  }

  // 9. 除了融合图，再额外输出三张单通道图，便于调参与分析。
  const std::string height_path = DeriveSiblingPath(config.bev_image_path, "_height");
  const std::string intensity_path = DeriveSiblingPath(config.bev_image_path, "_intensity");
  const std::string density_path = DeriveSiblingPath(config.bev_image_path, "_density");

  if (!cv::imwrite(config.bev_image_path, bev)) {
    std::cerr << "Failed to save BEV image: " << config.bev_image_path << "\n";
    return 1;
  }
  if (!cv::imwrite(height_path, height_map) ||
      !cv::imwrite(intensity_path, intensity_map) ||
      !cv::imwrite(density_path, density_map)) {
    std::cerr << "Failed to save BEV channel images\n";
    return 1;
  }

  std::cout << "Loaded KITTI cloud: " << cloud->size() << " points\n";
  std::cout << "Downsampled cloud: " << cloud_down->size() << " points\n";
  std::cout << "Projected points in BEV range: " << valid_points << "\n";
  std::cout << "Saved BEV image: " << config.bev_image_path << "\n";
  std::cout << "Saved height map: " << height_path << "\n";
  std::cout << "Saved intensity map: " << intensity_path << "\n";
  std::cout << "Saved density map: " << density_path << "\n";
  std::cout << "BEV size: " << width << " x " << height << "\n";

  return 0;
}
