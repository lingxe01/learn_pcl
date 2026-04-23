#pragma once

#include <string>

namespace pcl_basic {

class Pcd2WebApp {
 public:
  int Run(int argc, char** argv) const;

 private:
  struct Config {
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

  bool LoadConfig(const std::string& config_path, Config* config) const;
  bool ApplyCommandLineArgs(int argc, char** argv, Config* config) const;
  bool ValidateConfig(const Config& config) const;
};

}  // namespace pcl_basic

