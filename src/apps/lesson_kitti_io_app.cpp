#include "apps/lesson_kitti_io_app.h"

#include "common/point_cloud_io.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

namespace pcl_basic {

bool LessonKittiIoApp::LoadConfig(const std::string& config_path,
                                  Config* config) const {
  if (!config) {
    return false;
  }

  try {
    std::ifstream config_stream(config_path);
    if (!config_stream) {
      std::cerr << "Failed to open config: " << config_path << "\n";
      return false;
    }

    nlohmann::json root;
    config_stream >> root;
    config->pointcloud_path = root.at("pointcloud_path").get<std::string>();
    if (config->pointcloud_path.empty()) {
      std::cerr << "Empty pointcloud_path in config: " << config_path << "\n";
      return false;
    }

    if (root.contains("voxel_leaf_size")) {
      const auto& voxel_leaf_size = root.at("voxel_leaf_size");
      if (!voxel_leaf_size.is_array() || voxel_leaf_size.size() != 3) {
        std::cerr << "voxel_leaf_size must be an array of 3 numbers\n";
        return false;
      }
      config->voxel_leaf_x = voxel_leaf_size.at(0).get<float>();
      config->voxel_leaf_y = voxel_leaf_size.at(1).get<float>();
      config->voxel_leaf_z = voxel_leaf_size.at(2).get<float>();
    }
  } catch (const std::exception& error) {
    std::cerr << "Failed to parse config: " << config_path << "\n";
    std::cerr << "Error: " << error.what() << "\n";
    return false;
  }

  return true;
}

int LessonKittiIoApp::Run() const {
  const std::string config_path = "config/pointcloud.json";
  Config config;
  if (!LoadConfig(config_path, &config)) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!PointCloudIO::LoadKittiBin(config.pointcloud_path, cloud.get())) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI> downsampled_cloud;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(config.voxel_leaf_x,
                           config.voxel_leaf_y,
                           config.voxel_leaf_z);
  voxel_filter.filter(downsampled_cloud);

  std::cout << "Loaded KITTI cloud: " << cloud->size() << " points\n";
  std::cout << "Downsampled cloud: " << downsampled_cloud.size() << " points\n";

  if (!downsampled_cloud.empty()) {
    const auto& point = downsampled_cloud.front();
    std::cout << "First point(after voxel): [" << point.x << ", " << point.y
              << ", " << point.z << "], intensity=" << point.intensity << "\n";
  }

  return 0;
}

}  // namespace pcl_basic

