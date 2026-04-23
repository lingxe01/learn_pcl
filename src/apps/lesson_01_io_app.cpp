#include "apps/lesson_01_io_app.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>

namespace pcl_basic {

int Lesson01IoApp::Run() const {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 5;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = static_cast<float>(i);
    cloud.points[i].y = static_cast<float>(i) * 0.1f;
    cloud.points[i].z = static_cast<float>(i) * 0.2f;
  }

  const std::string file_name = "data/lesson_01_io.pcd";
  if (pcl::io::savePCDFileASCII(file_name, cloud) != 0) {
    std::cerr << "Failed to write PCD: " << file_name << "\n";
    return 1;
  }
  std::cout << "Saved PCD: " << file_name << " (" << cloud.size() << " points)\n";

  pcl::PointCloud<pcl::PointXYZ> loaded;
  if (pcl::io::loadPCDFile(file_name, loaded) != 0) {
    std::cerr << "Failed to read PCD: " << file_name << "\n";
    return 1;
  }
  std::cout << "Loaded PCD: " << file_name << " (" << loaded.size() << " points)\n";

  if (!loaded.empty()) {
    const auto& point = loaded.points.front();
    std::cout << "First point: (" << point.x << ", " << point.y << ", "
              << point.z << ")\n";
  }

  return 0;
}

}  // namespace pcl_basic

