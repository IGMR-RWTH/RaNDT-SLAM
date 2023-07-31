#ifndef CLUSTER_GENERATOR_H
#define CLUSTER_GENERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ml/kmeans.h>
#include "ndt_slam/ndt_slam_parameters.h"

namespace rc {
namespace navigation {
namespace ndt {

enum class ClusteringType {Grid = 0};

class ClusterGenerator {
  public:
    /** @brief generate label for each point
    *  @param n_clusters number of clusters
    *  @param cloud point cloud
    *  @param indizes the resulting labels are written in this vector 
    */
    virtual void cluster(const size_t n_clusters, const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& labels) = 0;

    /** @brief use labels to generate clusters
    *  @param cloud incoming cloud
    *  @param polar_point points in polar representation
    *  @param labels provided labels
    *  @param labeled_clouds vector of points clouds each corresponding to a cluster
    *  @param polar_points vector of vector of polar points each corresponding to a cluster. Needed for pNDT 
    */
    void labelClouds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::vector<std::pair<double, double>>& polar_point, const std::vector<int>& labels, std::vector<pcl::PointCloud<pcl::PointXYZI>>& labeled_clouds, std::vector<std::vector<std::pair<double, double>>>& polar_points);
    
    /** @brief get closest cells of cell
    *  @param cloud point cloud of cluster to split
    *  @param n_new_clusters number of new clusters
    *  @param labels new labels 
    */
    void splitCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const int& n_new_clusters, std::vector<int>& labels);
    
    inline void setMaxRange(float max_sensor_range) {
      max_sensor_range_ = max_sensor_range;
    };

  protected:
    float max_sensor_range_;
};
class Grid : public ClusterGenerator {
  public:
    void cluster(const size_t n_clusters, const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& labels) override;
};

}
}
}

#endif