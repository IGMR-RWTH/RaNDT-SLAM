#include "radar_preprocessing/cluster_generator.h"

namespace rc {
namespace navigation {
namespace ndt {
// for legacy reasons, the clusters are seperately generated from the map.
void Grid::cluster(const size_t n_clusters, const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& labels) {
  int row_size = static_cast<int>(std::sqrt(n_clusters));
  float resolution = max_sensor_range_ * 2 / (row_size);
  for (size_t i = 0; i < cloud->size(); i++) {
    int index = static_cast<int>(cloud->points[i].x/resolution) + row_size * static_cast<int>(cloud->points[i].y/resolution);
    labels[i] = index;
  }
}
}
}
}