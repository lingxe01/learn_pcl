#include "apps/pcd2bev_app.h"

#include "common/point_cloud_io.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace {

struct CellStat {
  float max_height = -std::numeric_limits<float>::infinity();
  float max_intensity = 0.0f;
  int count = 0;
};

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

std::string DeriveSiblingPath(const std::string& file_path, const std::string& suffix) {
  const std::filesystem::path path(file_path);
  return (path.parent_path() /
          (path.stem().string() + suffix + path.extension().string()))
      .string();
}

unsigned char ToByte(float normalized_value) {
  return static_cast<unsigned char>(
      std::round(std::clamp(normalized_value, 0.0f, 1.0f) * 255.0f));
}

void DrawRangeGrid(const pcl_basic::Pcd2BevApp::Config& config, cv::Mat* image) {
  if (!image || image->empty()) {
    return;
  }

  const int width = image->cols;
  const int height = image->rows;
  const float step_meter = 10.0f;
  const cv::Scalar grid_color(40, 40, 40);

  for (float x = config.x_min; x <= config.x_max; x += step_meter) {
    const int row =
        height - 1 - static_cast<int>((x - config.x_min) / config.resolution);
    if (row >= 0 && row < height) {
      cv::line(*image, cv::Point(0, row), cv::Point(width - 1, row), grid_color, 1);
    }
  }

  for (float y = config.y_min; y <= config.y_max; y += step_meter) {
    const int col = static_cast<int>((y - config.y_min) / config.resolution);
    if (col >= 0 && col < width) {
      cv::line(*image, cv::Point(col, 0), cv::Point(col, height - 1), grid_color, 1);
    }
  }
}

void DrawEgoVehicle(const pcl_basic::Pcd2BevApp::Config& config, cv::Mat* image) {
  if (!image || image->empty()) {
    return;
  }

  const float ego_length_meter = 4.5f;
  const float ego_width_meter = 1.8f;
  const int height = image->rows;

  const int center_col = static_cast<int>((0.0f - config.y_min) / config.resolution);
  const int bottom_row =
      height - 1 - static_cast<int>((0.0f - config.x_min) / config.resolution);
  const int half_width_pixels = std::max(
      1, static_cast<int>(std::round((ego_width_meter * 0.5f) / config.resolution)));
  const int length_pixels = std::max(
      1, static_cast<int>(std::round(ego_length_meter / config.resolution)));

  const cv::Rect ego_rect(
      std::max(0, center_col - half_width_pixels),
      std::max(0, bottom_row - length_pixels),
      std::min(image->cols - std::max(0, center_col - half_width_pixels),
               half_width_pixels * 2),
      std::min(image->rows - std::max(0, bottom_row - length_pixels), length_pixels));

  if (ego_rect.width > 0 && ego_rect.height > 0) {
    cv::rectangle(*image, ego_rect, cv::Scalar(255, 255, 255), 1);
    cv::line(*image,
             cv::Point(center_col, ego_rect.y),
             cv::Point(center_col, ego_rect.y + ego_rect.height),
             cv::Scalar(255, 255, 255),
             1);
  }
}

}  // namespace

namespace pcl_basic {

bool Pcd2BevApp::LoadConfig(const std::string& config_path, Config* config) const {
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

    if (root.contains("voxel_leaf_size") &&
        !ParseFloatTriplet(root.at("voxel_leaf_size"),
                           &config->voxel_leaf_x,
                           &config->voxel_leaf_y,
                           &config->voxel_leaf_z)) {
      std::cerr << "voxel_leaf_size must be an array of 3 numbers\n";
      return false;
    }

    if (root.contains("bev_image_path")) {
      config->bev_image_path = root.at("bev_image_path").get<std::string>();
      if (config->bev_image_path.empty()) {
        std::cerr << "Empty bev_image_path in config: " << config_path << "\n";
        return false;
      }
    }

    if (root.contains("bev_resolution")) {
      config->resolution = root.at("bev_resolution").get<float>();
    }
    if (root.contains("bev_x_range") &&
        !ParseFloatPair(root.at("bev_x_range"), &config->x_min, &config->x_max)) {
      std::cerr << "bev_x_range must be an array of 2 numbers\n";
      return false;
    }
    if (root.contains("bev_y_range") &&
        !ParseFloatPair(root.at("bev_y_range"), &config->y_min, &config->y_max)) {
      std::cerr << "bev_y_range must be an array of 2 numbers\n";
      return false;
    }
    if (root.contains("bev_z_range") &&
        !ParseFloatPair(root.at("bev_z_range"), &config->z_min, &config->z_max)) {
      std::cerr << "bev_z_range must be an array of 2 numbers\n";
      return false;
    }
  } catch (const std::exception& error) {
    std::cerr << "Failed to parse config: " << config_path << "\n";
    std::cerr << "Error: " << error.what() << "\n";
    return false;
  }

  if (config->resolution <= 0.0f) {
    std::cerr << "bev_resolution must be positive\n";
    return false;
  }
  if (config->x_max <= config->x_min || config->y_max <= config->y_min ||
      config->z_max <= config->z_min) {
    std::cerr << "Invalid BEV ranges in config\n";
    return false;
  }
  return true;
}

int Pcd2BevApp::Run() const {
  const std::string config_path = "config/pointcloud.json";
  Config config;
  if (!LoadConfig(config_path, &config)) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!PointCloudIO::LoadKittiBin(config.pointcloud_path, cloud.get())) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(config.voxel_leaf_x,
                           config.voxel_leaf_y,
                           config.voxel_leaf_z);
  voxel_filter.filter(*downsampled_cloud);

  const int width =
      static_cast<int>(std::ceil((config.y_max - config.y_min) / config.resolution));
  const int height =
      static_cast<int>(std::ceil((config.x_max - config.x_min) / config.resolution));
  if (width <= 0 || height <= 0) {
    std::cerr << "Invalid BEV output size\n";
    return 1;
  }

  std::vector<CellStat> grid(static_cast<std::size_t>(width) * height);
  int valid_points = 0;

  for (const auto& point : downsampled_cloud->points) {
    if (point.x < config.x_min || point.x >= config.x_max ||
        point.y < config.y_min || point.y >= config.y_max ||
        point.z < config.z_min || point.z > config.z_max) {
      continue;
    }

    const int col = static_cast<int>((point.y - config.y_min) / config.resolution);
    const int row = height - 1 -
                    static_cast<int>((point.x - config.x_min) / config.resolution);
    if (row < 0 || row >= height || col < 0 || col >= width) {
      continue;
    }

    CellStat& cell = grid[static_cast<std::size_t>(row) * width + col];
    cell.max_height = std::max(cell.max_height, point.z);
    cell.max_intensity = std::max(cell.max_intensity, point.intensity);
    ++cell.count;
    ++valid_points;
  }

  cv::Mat height_map(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat intensity_map(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat density_map(height, width, CV_8UC1, cv::Scalar(0));
  const float z_span = config.z_max - config.z_min;

  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const CellStat& cell = grid[static_cast<std::size_t>(row) * width + col];
      if (cell.count == 0) {
        continue;
      }

      const float height_norm =
          std::clamp((cell.max_height - config.z_min) / z_span, 0.0f, 1.0f);
      const float intensity_norm = std::clamp(cell.max_intensity, 0.0f, 1.0f);
      const float density_norm =
          std::clamp(std::log1p(static_cast<float>(cell.count)) / std::log(64.0f),
                     0.0f,
                     1.0f);

      height_map.at<unsigned char>(row, col) = ToByte(height_norm);
      intensity_map.at<unsigned char>(row, col) = ToByte(intensity_norm);
      density_map.at<unsigned char>(row, col) = ToByte(density_norm);
    }
  }

  cv::Mat bev_image;
  cv::merge(std::vector<cv::Mat>{intensity_map, height_map, density_map}, bev_image);
  DrawRangeGrid(config, &bev_image);
  DrawEgoVehicle(config, &bev_image);

  const std::filesystem::path output_path(config.bev_image_path);
  if (!output_path.parent_path().empty()) {
    std::filesystem::create_directories(output_path.parent_path());
  }

  const std::string height_path = DeriveSiblingPath(config.bev_image_path, "_height");
  const std::string intensity_path =
      DeriveSiblingPath(config.bev_image_path, "_intensity");
  const std::string density_path = DeriveSiblingPath(config.bev_image_path, "_density");

  if (!cv::imwrite(config.bev_image_path, bev_image)) {
    std::cerr << "Failed to save BEV image: " << config.bev_image_path << "\n";
    return 1;
  }
  if (!cv::imwrite(height_path, height_map) ||
      !cv::imwrite(intensity_path, intensity_map) ||
      !cv::imwrite(density_path, density_map)) {
    std::cerr << "Failed to save BEV channel images\n";
    return 1;
  }

  std::cout << "Loaded KITTI cloud: " << cloud->size() << " points\n";
  std::cout << "Downsampled cloud: " << downsampled_cloud->size() << " points\n";
  std::cout << "Projected points in BEV range: " << valid_points << "\n";
  std::cout << "Saved BEV image: " << config.bev_image_path << "\n";
  std::cout << "Saved height map: " << height_path << "\n";
  std::cout << "Saved intensity map: " << intensity_path << "\n";
  std::cout << "Saved density map: " << density_path << "\n";
  std::cout << "BEV size: " << width << " x " << height << "\n";
  return 0;
}

}  // namespace pcl_basic

