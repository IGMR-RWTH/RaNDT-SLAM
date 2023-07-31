#include "ndt_visualization/rviz_visualization.h"

namespace rc {
namespace navigation {
namespace ndt {

RvizVisualizer::RvizVisualizer() : nh_("~")
{
  std::cout << "constructing visualizer\n";
  loadParameters();
  
  // ROS publishers
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ndt_cells", 5);
  aligned_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/aligned_ndt_cells", 5);

  // ROS subscribers
  normal_distributions_sub_ = nh_.subscribe("/normal_distribution", 1, &RvizVisualizer::normalDistributionsCallback, this);
  aligned_normal_distributions_sub_ = nh_.subscribe("/aligned_normal_distribution", 1, &RvizVisualizer::alignedNormalDistributionsCallback, this);
}

void RvizVisualizer::createDistributionMarker(const Eigen::Vector2f& mean, const Eigen::Matrix2f& cov, visualization_msgs::Marker& marker){
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.lifetime = ros::Duration(10);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> s(cov);
	marker.pose.position.x = mean[0];
	marker.pose.position.y = mean[1];
  Eigen::Vector2f eig_values {s.eigenvalues().real()};
  Eigen::Matrix2f eig_vectors {s.eigenvectors().real()};

  double angle = (atan2(eig_vectors(1, 0), eig_vectors(0, 0)));

  double major_len = sqrt(eig_values[0]);
  double minor_len = sqrt(eig_values[1]);

  marker.scale.x = major_len;
  marker.scale.y = minor_len;
  marker.scale.z = 0.002;

  marker.pose.orientation.w = cos(angle*0.5);
  marker.pose.orientation.z = sin(angle*0.5);

  marker.header.stamp = ros::Time::now();
}

void RvizVisualizer::createDistributionMarker(const Eigen::Vector3f& mean, const Eigen::Matrix3f& cov, visualization_msgs::Marker& marker){
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.lifetime = ros::Duration(10);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> s(cov);
	marker.pose.position.x = mean[0];
	marker.pose.position.y = mean[1];
	marker.pose.position.z = mean[2] * 0.01;
  //marker.pose.position.z = mean[2];
  Eigen::Vector3f eig_values {s.eigenvalues().real()};
  Eigen::Matrix3f eig_vectors {s.eigenvectors().real()};

  tf2::Matrix3x3 rotation_matrix(eig_vectors(0,0), eig_vectors(0,1), eig_vectors(0,2), 
                                 eig_vectors(1,0), eig_vectors(1,1), eig_vectors(1,2), 
                                 eig_vectors(2,0), eig_vectors(2,1), eig_vectors(2,2));
  tf2::Quaternion q;
  rotation_matrix.getRotation(q);

  double angle_yaw = (atan2(eig_vectors(1, 0), eig_vectors(0, 0)));

  double major_len = sqrt(eig_values[0]);
  double medium_len = sqrt(eig_values[1]);
  double minor_len = sqrt(eig_values[2]);

  marker.scale.x = major_len;
  marker.scale.y = medium_len;
  marker.scale.z = minor_len*0.01;

  marker.pose.orientation.x = q[0];
  marker.pose.orientation.y = q[1];
  marker.pose.orientation.z = q[2];
  marker.pose.orientation.w = q[3];

  marker.header.stamp = ros::Time::now();
}

void RvizVisualizer::loadParameters() {
	std::cout << "parameters loaded!\n";
}

void RvizVisualizer::normalDistributionsCallback(const ndt_msgs::NormalDistributionsConstPtr& msg) {
	visualization_msgs::MarkerArray marker_array;
  max_markers_ = std::max(max_markers_, msg->normal_distributions.size());
  for (int i = 0; i < msg->normal_distributions.size(); i++) {
    visualization_msgs::Marker marker;
    marker.color.a = 1.0f;
    marker.header = msg->header;
    marker.id = i;
    marker.lifetime = ros::Duration(10);
    ndt_msgs::NormalDistribution nd;
    nd = msg->normal_distributions[i];
    Eigen::Vector2f mean{nd.mean.x, nd.mean.y};
    Eigen::Matrix2f cov;
    cov << nd.covariance.xx, nd.covariance.xy, nd.covariance.xy, nd.covariance.yy;
    createDistributionMarker(mean, cov, marker);
    getRainbowColor(nd.mean.i, marker);
    marker_array.markers.push_back(marker);
  }
  for (size_t i = msg->normal_distributions.size(); i < max_markers_; i++) {
    visualization_msgs::Marker marker;
    marker.header = msg->header;
    marker.id = i;
    marker.action = marker.DELETE;
    marker_array.markers.push_back(marker);
  }
  max_markers_ = msg->normal_distributions.size();
  marker_pub_.publish(marker_array);
}

void RvizVisualizer::alignedNormalDistributionsCallback(const ndt_msgs::NormalDistributionsConstPtr& msg) {
	visualization_msgs::MarkerArray marker_array;
  aligned_max_markers_ = std::max(aligned_max_markers_, msg->normal_distributions.size());
  for (int i = 0; i < msg->normal_distributions.size(); i++) {
    visualization_msgs::Marker marker;
    marker.color.a = 1.0f;
    marker.header = msg->header;
    marker.id = i;
    marker.lifetime = ros::Duration(10);
    ndt_msgs::NormalDistribution nd;
    nd = msg->normal_distributions[i];
    Eigen::Vector2f mean{nd.mean.x, nd.mean.y};
    Eigen::Matrix2f cov;
    cov << nd.covariance.xx, nd.covariance.xy, nd.covariance.xy, nd.covariance.yy;
    createDistributionMarker(mean, cov, marker);
    getRainbowColor(nd.mean.i, marker);
    //getRainbowColor(nd.mean_intensity.data, marker);
    marker_array.markers.push_back(marker);
  }
  for (size_t i = msg->normal_distributions.size(); i < aligned_max_markers_; i++) {
    visualization_msgs::Marker marker;
    marker.header = msg->header;
    marker.id = i;
    marker.action = marker.DELETE;
    marker_array.markers.push_back(marker);
  }
  aligned_max_markers_ = msg->normal_distributions.size();
  aligned_marker_pub_.publish(marker_array);
}

void RvizVisualizer::getRainbowColor(float value, visualization_msgs::Marker& marker)
{
  // this is HSV color palette with hue values going only from 0.0 to 0.833333.

  value = std::min(value, 40.0f);
  value = std::max(value, 10.0f);
  value -= 10.f;
  value /= 30.f;

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  float n = 1 - f;

  if (i <= 1)
    marker.color.r = n, marker.color.g = 0, marker.color.b = 1;
  else if (i == 2)
    marker.color.r = 0, marker.color.g = n, marker.color.b = 1;
  else if (i == 3)
    marker.color.r = 0, marker.color.g = 1, marker.color.b = n;
  else if (i == 4)
    marker.color.r = n, marker.color.g = 1, marker.color.b = 0;
  else if (i >= 5)
    marker.color.r = 1, marker.color.g = n, marker.color.b = 0;
}

}
}
}