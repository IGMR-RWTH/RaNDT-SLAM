#ifndef RADAR_PREPROCESSOR_H
#define RADAR_PREPROCESSOR_H

#include <sensor_msgs/PointCloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#include <boost/shared_ptr.hpp>

#include "radar_preprocessing/cluster_generator.h"
#include "ndt_slam/ndt_slam_parameters.h"

namespace rc {
namespace navigation {
namespace ndt {

class RadarPreprocessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** @brief initialize preprocessor
    *  @param clustering_type type of clustering, only Grid
    *  @param parameters parameters of the preprocessor
    *  @param initial_transform_radar_baselink transform between robot centric frame and sensor frame
    */
    void initialize(const ClusteringType& clustering_type, RadarPreprocessorParameters parameters, const Eigen::Affine3f& initial_transform_radar_baselink);
    
    /** @brief process an incoming scan
    *  @param cloud_in pointcloud message to process
    *  @param clusters clustered pointclouds resulting from the preprocessing
    *  @param polar_points clustered polar points resulting from the preprocessing for pNDT
    *  @param max_detections maximum intensity points per azimuth resulting from the preprocessing for OGM
    */
    void processScan(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters, std::vector<std::vector<std::pair<double, double>>>& polar_points, std::vector<std::tuple<double, double, double>>& max_detections);
    
    /** @brief further split cluster deprecated
    */
    void splitClusters(std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters, std::vector<std::vector<std::pair<double, double>>>& polar_points);
    /** @brief get filtered point Cloud
    */
    inline pcl::PointCloud<pcl::PointXYZI>::Ptr getDebugCloud() const {
      return debug_cloud;
    }
  private:

  /** @brief filter a scan
    *  @param cloud_in incoming radar scan message
    *  @param filtered_cloud outgoing filtered point cloud 
    *  @param polar_points clustered polar points resulting from the preprocessing for pNDT
    *  @param max_detections maximum intensity points per azimuth resulting from the preprocessing for OGM
    */
  void filterScan(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, std::vector<std::pair<double, double>>& polar_point, std::vector<std::tuple<double, double, double>>& max_detections);
    Eigen::Affine3f initial_transform_radar_baselink_;
    boost::shared_ptr<ClusterGenerator> _cluster_generator;

		float min_distance_;
    float max_distance_;
		float min_intensity_;

    RadarPreprocessorParameters parameters_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr debug_cloud;
    ClusteringType clustering_type_;
};
}
}
}

#endif