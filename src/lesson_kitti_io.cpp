#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

int main() {
  // 从 JSON 配置读取点云路径与体素尺寸。
  // 这样做的好处是：代码逻辑固定，但输入数据和参数可在配置文件中灵活调整。
  const std::string config_path = "config/pointcloud.json";
  std::string file_path;
  float leaf_x = 0.2f;
  float leaf_y = 0.2f;
  float leaf_z = 0.2f;
  try {
    // 打开配置文件并反序列化为 JSON 对象。
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

    // voxel_leaf_size 是一个长度为 3 的数组，分别对应 x/y/z 方向的体素尺寸。
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
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse config: " << config_path << "\n";
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  // 读取 KITTI 原始二进制点云。
  // KITTI velodyne 文件里每个点由 4 个 float 组成：
  // x、y、z、intensity。
  std::ifstream in(file_path, std::ios::binary);
  if (!in) {
    std::cerr << "Failed to open: " << file_path << "\n";
    return 1;
  }

  // 通过文件大小估算点数。
  // 每个点固定占 16 字节，因此可以先根据文件长度做一次粗校验。
  in.seekg(0, std::ios::end);
  const std::streamsize file_size = in.tellg();
  in.seekg(0, std::ios::beg);
  if (file_size <= 0 || file_size % (4 * sizeof(float)) != 0) {
    std::cerr << "Unexpected KITTI bin size: " << file_size << " bytes\n";
  }

  const std::size_t point_count =
      static_cast<std::size_t>(file_size / (4 * sizeof(float)));

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(point_count);

  float data[4];
  // 逐点读取并存入 PCL 点云对象。
  // 这里使用 PointXYZI，是因为 KITTI 数据除了坐标外还带有反射强度。
  while (in.read(reinterpret_cast<char*>(data), sizeof(data))) {
    pcl::PointXYZI p;
    p.x = data[0];
    p.y = data[1];
    p.z = data[2];
    p.intensity = data[3];
    cloud.push_back(p);
  }

  // 体素下采样。
  // 这一步会把落在同一体素中的点聚合成一个代表点，
  // 是点云预处理里最常见的降采样方式之一。
  pcl::PointCloud<pcl::PointXYZI> cloud_down;
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(cloud.makeShared());
  voxel.setLeafSize(leaf_x, leaf_y, leaf_z);  // 体素大小来自配置
  voxel.filter(cloud_down);

  // 输出下采样前后的点数，便于观察参数是否合适。
  std::cout << "Loaded KITTI cloud: " << cloud.size() << " points\n";
  std::cout << "Downsampled cloud: " << cloud_down.size() << " points\n";

  // 打印下采样后第一条点记录，作为最小结果校验。
  if (!cloud_down.empty()) {
    const auto& p0 = cloud_down.front();
    std::cout << "First point(after voxel): [" << p0.x << ", " << p0.y << ", "
              << p0.z << "], intensity=" << p0.intensity << "\n";
  }

  return 0;
}
