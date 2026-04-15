#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>

#include <iostream>

int main() {
  // 创建一个最基础的 PointXYZ 点云对象。
  // 这里的点类型只包含 x/y/z 三个坐标，不带颜色、法线、强度等额外字段。
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // width 和 height 共同描述点云的组织方式。
  // width=5, height=1 表示这是一个“非结构化”的一维点列表。
  cloud.width = 5;
  cloud.height = 1;

  // is_dense=false 表示点云中可能存在非法点（例如 NaN）。
  // 虽然这个示例里没有 NaN，但学习阶段保留这个字段有助于理解 PCL 点云元信息。
  cloud.is_dense = false;

  // 根据 width * height 预分配点数量。
  cloud.points.resize(cloud.width * cloud.height);

  // 逐个写入点坐标，构造一组简单的线性分布样本点。
  // 这些点将沿着 x 方向递增，同时 y 和 z 以不同斜率变化。
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = static_cast<float>(i);
    cloud.points[i].y = static_cast<float>(i) * 0.1f;
    cloud.points[i].z = static_cast<float>(i) * 0.2f;
  }

  // 计算三维质心。
  // pcl::compute3DCentroid 会把点云所有点的平均坐标写入 Eigen::Vector4f，
  // 其中前三个元素分别是 x/y/z 的均值，第四个元素通常是齐次坐标分量。
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(cloud, centroid);

  // 输出点云规模与质心，作为最小可运行示例。
  std::cout << "Point cloud size: " << cloud.size() << "\n";
  std::cout << "Centroid: [" << centroid[0] << ", " << centroid[1] << ", "
            << centroid[2] << "]\n";

  return 0;
}
