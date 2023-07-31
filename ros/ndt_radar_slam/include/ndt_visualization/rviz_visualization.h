#ifndef RVIZ_VISUALIZATION_H
#define RVIZ_VISUALIZATION_H

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ndt_msgs/Covariance.h>
#include <ndt_msgs/Mean.h>
#include <ndt_msgs/NormalDistribution.h>
#include <ndt_msgs/NormalDistributions.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rc {
namespace navigation {
namespace ndt {
class RvizVisualizer {
  public:
    RvizVisualizer();
	private:
    void createDistributionMarker(const Eigen::Vector2f& mean, const Eigen::Matrix2f& cov, visualization_msgs::Marker& marker);
    void createDistributionMarker(const Eigen::Vector3f& mean, const Eigen::Matrix3f& cov, visualization_msgs::Marker& marker);
    void loadParameters();
		void normalDistributionsCallback(const ndt_msgs::NormalDistributionsConstPtr& msg);
		void alignedNormalDistributionsCallback(const ndt_msgs::NormalDistributionsConstPtr& msg);
    void getRainbowColor(float value, visualization_msgs::Marker& marker);

		ros::NodeHandle nh_;
		ros::Publisher marker_pub_;
		ros::Subscriber normal_distributions_sub_;
		ros::Publisher aligned_marker_pub_;
		ros::Subscriber aligned_normal_distributions_sub_;

    size_t max_markers_ = 0;
    size_t aligned_max_markers_ = 0;
};
}
}
}

#endif