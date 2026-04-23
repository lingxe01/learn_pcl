#pragma once

#include <string>

namespace pcl_basic {

class Pcd2BevApp {
 public:
  struct Config {
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

  int Run() const;

 private:
  bool LoadConfig(const std::string& config_path, Config* config) const;
};

}  // namespace pcl_basic
