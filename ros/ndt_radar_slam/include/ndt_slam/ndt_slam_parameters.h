#ifndef NDT_SLAM_PARAMETERS_H
#define NDT_SLAM_PARAMETERS_H

#include <Eigen/Core>

namespace rc {
namespace navigation {
namespace ndt {

struct NDTCellParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3f beam_cov;
  bool use_pndt;
};

struct NDTMapParameters {
  int size_x;
  int size_y;
  double resolution;
  double ogm_resolution;
  double ogm_threshold;
  double max_neighbour_manhattan_distance;
  int min_points_per_cell;

  NDTCellParameters ndt_cell_parameters;
};

struct OGMMapParameters {
  int size_x;
  int size_y;
  int submap_size_x;
  int submap_size_y;
  double ndt_resolution;
  double resolution;
  double ogm_threshold;
};

struct RadarPreprocessorParameters {
  int n_clusters;
  double min_intensity;
  double min_range;
  double max_range;
  bool cluster_all_points;
  double beam_distance_increment_threshold;
  int min_points_per_cell;
  std::string sensor_frame;
  std::string base_frame;
};

struct NDTMatcherParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double,8,8> motion_sqrtI = Eigen::Matrix<double,8,8>::Identity();
  double covariance_scaling_factor;
  double weight_kinematics;
  double weight_imu;
  double weight_imu_bias;
  double initial_imu_bias;
  int gnc_steps;
  int smoothing_steps;
  double loss_function_convexity;
  double loss_function_scale;
  double gnc_control_parameter_divisor;
  int max_iteration;
  double pose_reject_translation;
  double pose_reject_rotation;
  int n_results_kd_lookup;
  double ndt_weight;
  bool use_analytic_expressions_for_optimization;
  bool use_intensity_as_dimension;
  bool use_constant_velocity_model;
  bool optimize_on_manifold;
  bool lookup_mahalanobis;
  bool use_imu;

  double csm_window_linear;
  double csm_window_angular;
  double csm_linear_step;
  double csm_cost_threshold;
  double csm_max_px_accurate_range;
  bool csm_ignore_overlap;
  int csm_n_iter;
};

struct ScanContextParameters {
  int    PC_NUM_RING = 20; // 20 in the original paper (IROS 18)
  int    PC_NUM_SECTOR = 60; // 60 in the original paper (IROS 18)
  double PC_MAX_RADIUS = 10.0; // 80 meter max in the original paper (IROS 18)
  double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
  double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

  // tree
  int    NUM_EXCLUDE_RECENT = 20; // simply just keyframe gap, but node position distance-based exclusion is ok. 
  int    NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)

  // loop thres
  double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
  //const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
  double SC_DIST_THRES = 0.8; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

  // config 
  int    TREE_MAKING_PERIOD_ = 10; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / if you want to find a very recent revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
  double assumed_drift;
  double odom_eps;
  double odom_weight;
  double intensity_factor;
};

struct LocalFuserParameters {
  std::string clustering_method = "Grid";//"Kmeans";
  int insertion_step;
  int insertion_delay;
  int submap_size_poses;
  int submap_overlap;
  int loop_closure_gnc_steps;
  double loop_closure_max_cs_divergence;
  double loop_closure_weight;
  bool use_intensity_in_loop_closure;
  double loop_closure_scale;
  double max_data_association_mahalanobis_dist;
  bool compute_dfs_loop_closure;
  bool use_scan_context_as_loop_closure;
  Eigen::Matrix<double,3,3> loop_sqrtI = Eigen::Matrix<double,3,3>::Identity();

  NDTMapParameters ndt_map_parameters;
  OGMMapParameters ogm_map_parameters;
  RadarPreprocessorParameters radar_preprocessor_parameters;
  NDTMatcherParameters ndt_matcher_parameters;  
  ScanContextParameters scan_context_parameters;
};

struct GlobalFuserParameters
{
//  bool optimize_on_manifold;
  double loss_function_scale;   
  bool use_robust_loss;
};

struct NDTSlamParameters {
  std::string scan_publisher_topic;
  std::string map_publisher_topic;
  std::string scan_subscriber_topic;
  std::string imu_subscriber_topic;
  std::string odom_subscriber_topic;
  std::string odom_publisher_topic;
  std::string fixed_frame;
  std::string odom_frame;
  std::string base_frame;
  std::string sensor_frame;
  bool visualize_ogm;
  bool visualize_current_scan;
  bool visualize_current_map;
  bool visualize_path;
  bool use_imu;
  bool initialize_from_tf;
  double visualizer_frequency;
  double pose_graph_frequency;
  double search_loop_closure_frequency;

  LocalFuserParameters local_fuser_parameters;
  GlobalFuserParameters global_fuser_parameters;
};

struct Parameters {
  NDTSlamParameters ndt_slam_parameters;
};

}}}
#endif