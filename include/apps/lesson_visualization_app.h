#pragma once

#include <string>

namespace pcl_basic {

class LessonVisualizationApp {
 public:
  int Run() const;

 private:
  struct Config {
    std::string pointcloud_path;
    float voxel_leaf_x = 0.2f;
    float voxel_leaf_y = 0.2f;
    float voxel_leaf_z = 0.2f;
    std::string screenshot_path = "output/kitti_000000.png";
  };

  bool LoadConfig(const std::string& config_path, Config* config) const;
};

}  // namespace pcl_basic

