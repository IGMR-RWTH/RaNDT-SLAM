#include "radar_preprocessing/radar_preprocessor.h"

namespace rc {
namespace navigation {
namespace ndt {

void RadarPreprocessor::initialize(const ClusteringType& clustering_type, RadarPreprocessorParameters parameters, const Eigen::Affine3f& initial_transform_radar_baselink) {
  parameters_ = parameters;
  initial_transform_radar_baselink_ = initial_transform_radar_baselink;
  min_distance_ = parameters_.min_range;
  max_distance_ = parameters_.max_range;
  min_intensity_ = parameters_.min_intensity;
  clustering_type_ = clustering_type;
  switch (clustering_type)
  {
  // simplest case, others were discarded
  case ClusteringType::Grid:
    std::cout << "created Grid clustering \n";
    _cluster_generator.reset(new Grid());
    break;
  
  // here, other clustering points may be used
  default:
    std::cout << "No valid clustering algorithm!\n";
    break;
  }
  _cluster_generator->setMaxRange(parameters_.max_range);
}

void RadarPreprocessor::processScan(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters, std::vector<std::vector<std::pair<double, double>>>& polar_points, std::vector<std::tuple<double, double, double>>& max_detections) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<std::pair<double, double>> all_polar_points;  // polar representation of the points for pNDT
  filterScan(cloud_in, filtered_cloud, all_polar_points, max_detections);  // get smaller scan
  std::vector<int> labels;
  labels.resize(filtered_cloud->size()); 
  _cluster_generator->cluster(parameters_.n_clusters, filtered_cloud, labels);  // write labels to each points
  _cluster_generator->labelClouds(filtered_cloud, all_polar_points, labels, clusters, polar_points);  // generate individual point clouds and polar points for each cluster
  for (size_t i = 0; i < clusters.size(); i++) {
    pcl_conversions::toPCL(cloud_in->header, clusters[i].header);  // the points were already converted into the base frame, we have to do adjust the headers accordingly
    clusters[i].header.frame_id = parameters_.base_frame;
  }
  filtered_cloud->header = clusters[0].header;
}

void RadarPreprocessor::filterScan(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, std::vector<std::pair<double, double>>& polar_point, std::vector<std::tuple<double, double, double>>& max_detections) {
  const int point_cloud_size = cloud_in->height * cloud_in->width;
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>()); // pcl cloud of message
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_not_transformed(new pcl::PointCloud<pcl::PointXYZI>()); // filtered cloud in sensor frame
  raw_cloud->reserve(point_cloud_size);
  filtered_cloud_not_transformed->reserve(point_cloud_size);
  pcl::fromROSMsg(*cloud_in, *raw_cloud);  // write into pcl cloud
  float current_angle = 1000; // initialize large angle
  float max_intensity = 0;
  size_t current_max_idx = 0;
  std::vector<size_t> max_intensity_idzs;
  for (int i = 0; i < point_cloud_size; i++) {
    const float dist = std::hypot(raw_cloud->at(i).x, raw_cloud->at(i).y);  // polar representation
    const float angle = std::atan2(raw_cloud->at(i).y, raw_cloud->at(i).x);
    const float intensity = raw_cloud->at(i).intensity;
    
    if (std::abs(angle - current_angle) > 0.0001) {   // detect new azimuth. Assumes pcl to be organized in azimuths
      if (current_angle < 3*M_PI) { // hacky 
        if (max_intensity_idzs.empty() || max_intensity_idzs.back() != current_max_idx) {
        max_detections.emplace_back(std::make_tuple(current_angle, std::hypot(raw_cloud->at(current_max_idx).x, raw_cloud->at(current_max_idx).y), max_intensity)); // store maximum intensity point for raytracing
        max_intensity_idzs.push_back(current_max_idx);
        }
        max_intensity = 0;
      }
      current_angle = angle;
    }
    if (dist > min_distance_ && dist < max_distance_ && intensity > max_intensity) {
      max_intensity = intensity;
      current_max_idx = i;
    }
  }
  for (size_t i = 0; i < max_intensity_idzs.size(); i++) { // for each max_intensity point
    size_t search_max_idx = max_intensity_idzs[i];
    size_t closer_idx, further_idx;
    size_t index_distance = 0;
    while (true) { //searching for min intensity point closer to sensor
      if (search_max_idx - index_distance - 1 < 0 || search_max_idx - index_distance - 1 > raw_cloud->size()-1 ) {
        break;
      }
      if (((std::hypot(raw_cloud->at(search_max_idx - index_distance).x, raw_cloud->at(search_max_idx - index_distance).y) - std::hypot(raw_cloud->at(search_max_idx - index_distance - 1).x, raw_cloud->at(search_max_idx - index_distance - 1).y)) > parameters_.beam_distance_increment_threshold) || // sensor dependent
          (raw_cloud->at(search_max_idx - index_distance).intensity <= raw_cloud->at(search_max_idx - index_distance - 1).intensity) ||
          (std::hypot(raw_cloud->at(search_max_idx - index_distance).x, raw_cloud->at(search_max_idx - index_distance).y) < min_distance_)) {
        closer_idx = search_max_idx - index_distance; 
        break;
      }
      else {
        index_distance++;
      }
    }
    index_distance = 0;
    while (true) { //searching for min intensity point farther away from sensor
      if (search_max_idx + index_distance + 1 < 0 || search_max_idx + index_distance + 1 > raw_cloud->size()-1 ) {
        break;
      }
      if (((std::hypot(raw_cloud->at(search_max_idx + index_distance).x, raw_cloud->at(search_max_idx + index_distance).y) - std::hypot(raw_cloud->at(search_max_idx + index_distance + 1).x, raw_cloud->at(search_max_idx + index_distance + 1).y)) > parameters_.beam_distance_increment_threshold) || // sensor dependent
          (raw_cloud->at(search_max_idx + index_distance).intensity <= raw_cloud->at(search_max_idx + index_distance + 1).intensity) ||
          (std::hypot(raw_cloud->at(search_max_idx + index_distance).x, raw_cloud->at(search_max_idx + index_distance).y) < min_distance_)) {
        further_idx = search_max_idx + index_distance; 
        break;
      }
      else {
        index_distance++;
      }
    }
    // add all points to filtered cloud
    for (size_t j = closer_idx; j<= further_idx; j++) {
      const float dist = std::hypot(raw_cloud->at(j).x, raw_cloud->at(j).y);
      const float angle = std::atan2(raw_cloud->at(j).y, raw_cloud->at(j).x);
      const float intensity = raw_cloud->at(j).intensity;
      if (dist > min_distance_ && dist < max_distance_ && intensity > min_intensity_) {
        filtered_cloud_not_transformed->push_back(raw_cloud->at(j));
        polar_point.emplace_back(std::make_pair(angle, dist));
      }
    }
  }
  // for visualization
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  debug_cloud = filtered_cloud_not_transformed;
  debug_cloud->header.frame_id = parameters_.sensor_frame;
  pcl::transformPointCloud(*filtered_cloud_not_transformed, *filtered_cloud, initial_transform_radar_baselink_); 
}

void RadarPreprocessor::splitClusters(std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters, std::vector<std::vector<std::pair<double, double>>>& polar_points) {
  std::vector<pcl::PointCloud<pcl::PointXYZI>> new_clusters;
  std::vector<std::vector<std::pair<double,double>>> new_polar_points;
  std::vector<int> labels;
  for (int i = 0; i < clusters.size(); i++) { 
    labels.clear();
    labels.resize(clusters[i].size()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptr(new pcl::PointCloud<pcl::PointXYZI>(clusters[i]));
    std::vector<pcl::PointCloud<pcl::PointXYZI>> new_subclusters;
    std::vector<std::vector<std::pair<double,double>>> new_sub_polar_points;
    if (clustering_type_ == ClusteringType::Grid) {
      _cluster_generator->splitCluster(cluster_ptr, parameters_.n_clusters * 4, labels);
    } 
    else {
      _cluster_generator->splitCluster(cluster_ptr, 4, labels);
    }
    _cluster_generator->labelClouds(cluster_ptr, polar_points[i], labels, new_subclusters, new_sub_polar_points);
    new_clusters.insert(new_clusters.end(), new_subclusters.begin(), new_subclusters.end());
    new_polar_points.insert(new_polar_points.end(), new_sub_polar_points.begin(), new_sub_polar_points.end());
  }
  clusters = new_clusters;
}

// to generate vectors for each individual cluster
void ClusterGenerator::labelClouds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                                   const std::vector<std::pair<double, double>>& polar_point, 
                                   const std::vector<int>& labels, 
                                   std::vector<pcl::PointCloud<pcl::PointXYZI>>& labeled_clouds,
                                   std::vector<std::vector<std::pair<double, double>>>& polar_points) {
  std::vector<int> labels_2_sort = labels;
  std::sort(labels_2_sort.begin(), labels_2_sort.end());
  int n_clusters = std::unique(labels_2_sort.begin(), labels_2_sort.end()) - labels_2_sort.begin();
  std::map<int, int> signed_index_2_unsigned;
  for (size_t i = 0; i < n_clusters; i++) {
    signed_index_2_unsigned[labels_2_sort[i]] = i;
  }
  labeled_clouds.resize(n_clusters);
  polar_points.resize(n_clusters);
  for (size_t i = 0; i < cloud->size(); i++) {
    labeled_clouds[signed_index_2_unsigned[labels[i]]].push_back(cloud->at(i));
    polar_points[signed_index_2_unsigned[labels[i]]].push_back(polar_point[i]);
  }
}

void ClusterGenerator::splitCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const int& n_new_clusters, std::vector<int>& labels) {
  cluster(n_new_clusters, cloud, labels);
}
}
}
}