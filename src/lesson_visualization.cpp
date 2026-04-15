#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vtkRenderWindow.h>

namespace {

// 读取 KITTI 原始二进制点云。
// 每个点按 x、y、z、intensity 的 float4 顺序连续存储。
bool LoadKittiBin(const std::string& file_path,
                  pcl::PointCloud<pcl::PointXYZI>* cloud) {
  if (!cloud) {
    return false;
  }
  std::ifstream in(file_path, std::ios::binary);
  if (!in) {
    std::cerr << "Failed to open: " << file_path << "\n";
    return false;
  }

  in.seekg(0, std::ios::end);
  const std::streamsize file_size = in.tellg();
  in.seekg(0, std::ios::beg);
  if (file_size <= 0 || file_size % (4 * sizeof(float)) != 0) {
    std::cerr << "Unexpected KITTI bin size: " << file_size << " bytes\n";
  }

  const std::size_t point_count =
      static_cast<std::size_t>(file_size / (4 * sizeof(float)));
  cloud->clear();
  cloud->reserve(point_count);

  float data[4];
  while (in.read(reinterpret_cast<char*>(data), sizeof(data))) {
    pcl::PointXYZI p;
    p.x = data[0];
    p.y = data[1];
    p.z = data[2];
    p.intensity = data[3];
    cloud->push_back(p);
  }
  return !cloud->empty();
}

}  // namespace

int main() {
  // 从 JSON 配置读取点云路径、体素大小和截图路径。
  // 这里沿用与其他示例一致的配置入口，便于示例之间共享参数。
  const std::string config_path = "config/pointcloud.json";
  std::string file_path;
  float leaf_x = 0.2f;
  float leaf_y = 0.2f;
  float leaf_z = 0.2f;
  std::string screenshot_path = "output/kitti_000000.png";

  try {
    // 打开配置文件并解析成 JSON。
    std::ifstream cfg(config_path);
    if (!cfg) {
      std::cerr << "Failed to open config: " << config_path << "\n";
      return 1;
    }
    nlohmann::json j;
    cfg >> j;
    file_path = j.at("pointcloud_path").get<std::string>();
    if (file_path.empty()) {
      std::cerr << "Empty pointcloud_path in config: " << config_path << "\n";
      return 1;
    }

    // 可选读取体素尺寸，用于控制渲染前的点云稀疏程度。
    if (j.contains("voxel_leaf_size")) {
      const auto& v = j.at("voxel_leaf_size");
      if (!v.is_array() || v.size() != 3) {
        std::cerr << "voxel_leaf_size must be an array of 3 numbers\n";
        return 1;
      }
      leaf_x = v.at(0).get<float>();
      leaf_y = v.at(1).get<float>();
      leaf_z = v.at(2).get<float>();
    }

    // 可选读取截图路径，方便把渲染结果落到指定位置。
    if (j.contains("screenshot_path")) {
      screenshot_path = j.at("screenshot_path").get<std::string>();
      if (screenshot_path.empty()) {
        std::cerr << "Empty screenshot_path in config: " << config_path << "\n";
        return 1;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse config: " << config_path << "\n";
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  // 加载 KITTI 点云。
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!LoadKittiBin(file_path, cloud.get())) {
    std::cerr << "Load failed: " << file_path << "\n";
    return 1;
  }

  // 体素下采样。
  // 对可视化而言，适度降采样可以减少渲染压力，同时保留整体形状。
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_down(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf_x, leaf_y, leaf_z);
  voxel.filter(*cloud_down);

  std::cout << "Loaded KITTI cloud: " << cloud->size() << " points\n";
  std::cout << "Downsampled cloud: " << cloud_down->size() << " points\n";

  // 初始化 PCLVisualizer。
  // 这里仍然尝试使用离屏渲染，但是否真正脱离显示环境取决于底层 VTK 构建方式。
  pcl::visualization::PCLVisualizer viewer("PCL Visualizer");
  viewer.getRenderWindow()->SetOffScreenRendering(1);
  viewer.setBackgroundColor(0.05, 0.05, 0.05);

  // 基于 intensity 字段设置颜色映射。
  // 如果底层颜色处理器不可用，则退化为默认颜色显示。
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
      color_handler(cloud_down, "intensity");

  if (color_handler.isCapable()) {
    viewer.addPointCloud<pcl::PointXYZI>(cloud_down, color_handler, "cloud");
  } else {
    viewer.addPointCloud<pcl::PointXYZI>(cloud_down, "cloud");
  }
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

  // 触发一次渲染。
  // 对于只截图不交互的场景，spinOnce + Render 基本足够。
  viewer.spinOnce(100);
  viewer.getRenderWindow()->Render();

  // 确保输出目录存在。
  std::filesystem::path out_path(screenshot_path);
  if (!out_path.parent_path().empty()) {
    std::filesystem::create_directories(out_path.parent_path());
  }

  // 保存截图，并在文件不存在时给出明确错误。
  viewer.saveScreenshot(screenshot_path);
  if (!std::filesystem::exists(out_path)) {
    std::cerr << "Failed to save screenshot: " << screenshot_path << "\n";
    return 1;
  }
  std::cout << "Saved screenshot: " << screenshot_path << "\n";

  return 0;
}
