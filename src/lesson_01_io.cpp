#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

int main() {
  // 1) 创建一个简单的点云对象。
  // 这里仍然使用最基础的 PointXYZ 点类型，适合演示 PCD 的读写流程。
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 设置点云尺寸信息。
  // 它表示当前点云有 5 个点，且不是二维有序点云。
  cloud.width = 5;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  // 填充点坐标，构造一组可预测的测试数据。
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = static_cast<float>(i);
    cloud.points[i].y = static_cast<float>(i) * 0.1f;
    cloud.points[i].z = static_cast<float>(i) * 0.2f;
  }

  // 2) 将点云保存为 ASCII 形式的 PCD 文件。
  // ASCII 格式可读性强，适合调试和学习；实际大规模数据通常更常用 binary 以节省空间。
  const std::string filename = "data/lesson_01_io.pcd";
  if (pcl::io::savePCDFileASCII(filename, cloud) != 0) {
    std::cerr << "Failed to write PCD: " << filename << "\n";
    return 1;
  }
  std::cout << "Saved PCD: " << filename << " (" << cloud.size() << " points)\n";

  // 3) 再把刚才保存的文件读回来，验证 IO 流程是否正确。
  pcl::PointCloud<pcl::PointXYZ> loaded;
  if (pcl::io::loadPCDFile(filename, loaded) != 0) {
    std::cerr << "Failed to read PCD: " << filename << "\n";
    return 1;
  }
  std::cout << "Loaded PCD: " << filename << " (" << loaded.size() << " points)\n";

  // 读取第一条点记录并输出，便于确认反序列化后的坐标是否符合预期。
  if (!loaded.empty()) {
    const auto &p = loaded.points.front();
    std::cout << "First point: (" << p.x << ", " << p.y << ", " << p.z << ")\n";
  }

  return 0;
}
