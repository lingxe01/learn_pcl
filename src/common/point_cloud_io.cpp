#include "common/point_cloud_io.h"

#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>
#include <iostream>

namespace pcl_basic {

bool PointCloudIO::LoadKittiBin(const std::string& file_path,
                                pcl::PointCloud<pcl::PointXYZI>* cloud) {
  if (!cloud) {
    return false;
  }

  std::ifstream input(file_path, std::ios::binary);
  if (!input) {
    std::cerr << "Failed to open point cloud: " << file_path << "\n";
    return false;
  }

  input.seekg(0, std::ios::end);
  const std::streamsize file_size = input.tellg();
  input.seekg(0, std::ios::beg);
  if (file_size <= 0 ||
      file_size % (4 * static_cast<std::streamsize>(sizeof(float))) != 0) {
    std::cerr << "Unexpected KITTI bin size: " << file_size << " bytes\n";
  }

  const std::size_t point_count =
      static_cast<std::size_t>(file_size / (4 * sizeof(float)));
  cloud->clear();
  cloud->reserve(point_count);

  float data[4];
  while (input.read(reinterpret_cast<char*>(data), sizeof(data))) {
    pcl::PointXYZI point;
    point.x = data[0];
    point.y = data[1];
    point.z = data[2];
    point.intensity = data[3];
    cloud->push_back(point);
  }
  return !cloud->empty();
}

bool PointCloudIO::LoadPointCloud(const std::string& file_path,
                                  pcl::PointCloud<pcl::PointXYZI>* cloud) {
  if (!cloud) {
    return false;
  }

  const std::filesystem::path path(file_path);
  const std::string extension = path.extension().string();
  if (extension == ".bin") {
    return LoadKittiBin(file_path, cloud);
  }

  if (extension == ".pcd") {
    if (pcl::io::loadPCDFile(file_path, *cloud) != 0) {
      std::cerr << "Failed to open PCD point cloud: " << file_path << "\n";
      return false;
    }
    return !cloud->empty();
  }

  std::cerr << "Unsupported point cloud format: " << file_path << "\n";
  std::cerr << "Currently supported: .bin, .pcd\n";
  return false;
}

}  // namespace pcl_basic

