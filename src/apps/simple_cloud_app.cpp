#include "apps/simple_cloud_app.h"

#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

namespace pcl_basic {

int SimpleCloudApp::Run() const {
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

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(cloud, centroid);

  std::cout << "Point cloud size: " << cloud.size() << "\n";
  std::cout << "Centroid: [" << centroid[0] << ", " << centroid[1] << ", "
            << centroid[2] << "]\n";
  return 0;
}

}  // namespace pcl_basic

