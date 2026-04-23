#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace pcl_basic {

class PointCloudIO {
 public:
  static bool LoadKittiBin(const std::string& file_path,
                           pcl::PointCloud<pcl::PointXYZI>* cloud);

  static bool LoadPointCloud(const std::string& file_path,
                             pcl::PointCloud<pcl::PointXYZI>* cloud);
};

}  // namespace pcl_basic

