#include "apps/pcd2web_app.h"

#include "common/point_cloud_io.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

namespace {

bool ParseFloatTriplet(const nlohmann::json& value, float* a, float* b, float* c) {
  if (!a || !b || !c || !value.is_array() || value.size() != 3) {
    return false;
  }
  *a = value.at(0).get<float>();
  *b = value.at(1).get<float>();
  *c = value.at(2).get<float>();
  return true;
}

bool ParseFloatPair(const nlohmann::json& value, float* a, float* b) {
  if (!a || !b || !value.is_array() || value.size() != 2) {
    return false;
  }
  *a = value.at(0).get<float>();
  *b = value.at(1).get<float>();
  return true;
}

bool ParseFloatPairString(const std::string& text, float* min_value, float* max_value) {
  if (!min_value || !max_value) {
    return false;
  }

  std::stringstream stream(text);
  char comma = 0;
  if (!(stream >> *min_value >> comma >> *max_value) || comma != ',') {
    return false;
  }
  return true;
}

}  // namespace

namespace pcl_basic {

bool Pcd2WebApp::ValidateConfig(const Config& config) const {
  if (config.pointcloud_path.empty()) {
    std::cerr << "Empty point cloud input path\n";
    return false;
  }
  if (config.web_pointcloud_path.empty()) {
    std::cerr << "Empty web output path\n";
    return false;
  }
  if (config.voxel_leaf_x <= 0.0f || config.voxel_leaf_y <= 0.0f ||
      config.voxel_leaf_z <= 0.0f) {
    std::cerr << "voxel leaf size must be positive\n";
    return false;
  }
  if (config.use_x_range && config.x_min > config.x_max) {
    std::cerr << "x range must satisfy min <= max\n";
    return false;
  }
  if (config.use_y_range && config.y_min > config.y_max) {
    std::cerr << "y range must satisfy min <= max\n";
    return false;
  }
  if (config.use_z_range && config.z_min > config.z_max) {
    std::cerr << "z range must satisfy min <= max\n";
    return false;
  }
  return true;
}

bool Pcd2WebApp::LoadConfig(const std::string& config_path, Config* config) const {
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

    if (root.contains("web_pointcloud_path")) {
      config->web_pointcloud_path = root.at("web_pointcloud_path").get<std::string>();
      if (config->web_pointcloud_path.empty()) {
        std::cerr << "Empty web_pointcloud_path in config: " << config_path << "\n";
        return false;
      }
    }

    if (root.contains("voxel_leaf_size") &&
        !ParseFloatTriplet(root.at("voxel_leaf_size"),
                           &config->voxel_leaf_x,
                           &config->voxel_leaf_y,
                           &config->voxel_leaf_z)) {
      std::cerr << "voxel_leaf_size must be an array of 3 numbers\n";
      return false;
    }

    if (root.contains("web_x_range")) {
      config->use_x_range = true;
      if (!ParseFloatPair(root.at("web_x_range"), &config->x_min, &config->x_max)) {
        std::cerr << "web_x_range must be an array of 2 numbers\n";
        return false;
      }
    }
    if (root.contains("web_y_range")) {
      config->use_y_range = true;
      if (!ParseFloatPair(root.at("web_y_range"), &config->y_min, &config->y_max)) {
        std::cerr << "web_y_range must be an array of 2 numbers\n";
        return false;
      }
    }
    if (root.contains("web_z_range")) {
      config->use_z_range = true;
      if (!ParseFloatPair(root.at("web_z_range"), &config->z_min, &config->z_max)) {
        std::cerr << "web_z_range must be an array of 2 numbers\n";
        return false;
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "Failed to parse config: " << config_path << "\n";
    std::cerr << "Error: " << error.what() << "\n";
    return false;
  }

  return ValidateConfig(*config);
}

bool Pcd2WebApp::ApplyCommandLineArgs(int argc, char** argv, Config* config) const {
  if (!config) {
    return false;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    auto read_value = [&](const std::string& option) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << option << "\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (argument == "--input") {
      const char* value = read_value(argument);
      if (!value) {
        return false;
      }
      config->pointcloud_path = value;
    } else if (argument == "--output") {
      const char* value = read_value(argument);
      if (!value) {
        return false;
      }
      config->web_pointcloud_path = value;
    } else if (argument == "--voxel") {
      const char* value = read_value(argument);
      if (!value) {
        return false;
      }

      std::stringstream stream(value);
      char comma1 = 0;
      char comma2 = 0;
      if (!(stream >> config->voxel_leaf_x >> comma1 >> config->voxel_leaf_y >>
            comma2 >> config->voxel_leaf_z) ||
          comma1 != ',' || comma2 != ',') {
        std::cerr << "Invalid --voxel format, expected x,y,z\n";
        return false;
      }
    } else if (argument == "--x-range") {
      const char* value = read_value(argument);
      if (!value || !ParseFloatPairString(value, &config->x_min, &config->x_max)) {
        std::cerr << "Invalid --x-range format, expected min,max\n";
        return false;
      }
      config->use_x_range = true;
    } else if (argument == "--y-range") {
      const char* value = read_value(argument);
      if (!value || !ParseFloatPairString(value, &config->y_min, &config->y_max)) {
        std::cerr << "Invalid --y-range format, expected min,max\n";
        return false;
      }
      config->use_y_range = true;
    } else if (argument == "--z-range") {
      const char* value = read_value(argument);
      if (!value || !ParseFloatPairString(value, &config->z_min, &config->z_max)) {
        std::cerr << "Invalid --z-range format, expected min,max\n";
        return false;
      }
      config->use_z_range = true;
    } else if (argument == "--help" || argument == "-h") {
      std::cout
          << "Usage: pcd2web [--input path] [--output path] [--voxel x,y,z]"
          << " [--x-range min,max] [--y-range min,max] [--z-range min,max]\n";
      return false;
    } else {
      std::cerr << "Unknown argument: " << argument << "\n";
      return false;
    }
  }

  return ValidateConfig(*config);
}

int Pcd2WebApp::Run(int argc, char** argv) const {
  const std::string config_path = "config/pointcloud.json";
  Config config;
  if (!LoadConfig(config_path, &config)) {
    return 1;
  }
  if (!ApplyCommandLineArgs(argc, argv, &config)) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!PointCloudIO::LoadPointCloud(config.pointcloud_path, cloud.get())) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  filtered_cloud->reserve(cloud->size());
  for (const auto& point : cloud->points) {
    if (config.use_x_range && (point.x < config.x_min || point.x > config.x_max)) {
      continue;
    }
    if (config.use_y_range && (point.y < config.y_min || point.y > config.y_max)) {
      continue;
    }
    if (config.use_z_range && (point.z < config.z_min || point.z > config.z_max)) {
      continue;
    }
    filtered_cloud->push_back(point);
  }
  if (filtered_cloud->empty()) {
    std::cerr << "No points remain after applying the selected range filters\n";
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(filtered_cloud);
  voxel_filter.setLeafSize(config.voxel_leaf_x,
                           config.voxel_leaf_y,
                           config.voxel_leaf_z);
  voxel_filter.filter(*downsampled_cloud);
  if (downsampled_cloud->empty()) {
    std::cerr << "No points remain after voxel downsampling, try a smaller voxel size\n";
    return 1;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float min_z = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  nlohmann::json points = nlohmann::json::array();

  for (const auto& point : downsampled_cloud->points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);

    points.push_back(point.x);
    points.push_back(point.y);
    points.push_back(point.z);
    points.push_back(point.intensity);
  }

  nlohmann::json root;
  root["meta"] = {
      {"format", "xyzi_flat_array"},
      {"point_stride", 4},
      {"raw_point_count", cloud->size()},
      {"filtered_point_count", filtered_cloud->size()},
      {"point_count", downsampled_cloud->size()},
      {"source_path", config.pointcloud_path},
      {"voxel_leaf_size", {config.voxel_leaf_x, config.voxel_leaf_y, config.voxel_leaf_z}},
      {"range_filter",
       {
           {"x", config.use_x_range ? nlohmann::json::array({config.x_min, config.x_max})
                                    : nlohmann::json()},
           {"y", config.use_y_range ? nlohmann::json::array({config.y_min, config.y_max})
                                    : nlohmann::json()},
           {"z", config.use_z_range ? nlohmann::json::array({config.z_min, config.z_max})
                                    : nlohmann::json()},
       }},
      {"bounds",
       {
           {"x", {min_x, max_x}},
           {"y", {min_y, max_y}},
           {"z", {min_z, max_z}},
       }},
  };
  root["points"] = std::move(points);

  const std::filesystem::path output_path(config.web_pointcloud_path);
  if (!output_path.parent_path().empty()) {
    std::filesystem::create_directories(output_path.parent_path());
  }

  std::ofstream output(config.web_pointcloud_path);
  if (!output) {
    std::cerr << "Failed to open output file: " << config.web_pointcloud_path << "\n";
    return 1;
  }
  output << root.dump();

  std::cout << "Loaded KITTI cloud: " << cloud->size() << " points\n";
  std::cout << "Range filtered cloud: " << filtered_cloud->size() << " points\n";
  std::cout << "Downsampled cloud: " << downsampled_cloud->size() << " points\n";
  std::cout << "Saved web point cloud: " << config.web_pointcloud_path << "\n";
  return 0;
}

}  // namespace pcl_basic

