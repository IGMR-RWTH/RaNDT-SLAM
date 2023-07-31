#include "ndt_slam/ndt_slam.h"

namespace rc {
namespace navigation {
namespace ndt {

NDTSlam::NDTSlam(): _nh("~"), seq(ros::Duration(0.2), ros::Duration(0.01), 10) {
  
  std::cout << "created object!\n";
  readParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10.));
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
 
  if (parameters_.use_imu) {
    imu_sub.subscribe(_nh, parameters_.imu_subscriber_topic, 1);
    cache.setCacheSize(static_cast<size_t>(150));
    cache.connectInput(imu_sub);
    ros::topic::waitForMessage<sensor_msgs::Imu>(parameters_.imu_subscriber_topic);  // deprecated: one could specify a desync time between imu and radar
  }
  
  _cluster_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/normal_distribution_pcl", 5);

  if (parameters_.initialize_from_tf) {
    try { // get transform between base and odom, as well as between sensor and base
      init_transform_tf = tf_buffer_->lookupTransform(parameters_.base_frame, parameters_.odom_frame, ros::Time(0), ros::Duration(50));
      init_radar_baselink_transform = tf_buffer_->lookupTransform(parameters_.base_frame, parameters_.sensor_frame, ros::Time(0), ros::Duration(50));
      initial_transform_radar_baselink = tf2::transformToEigen(init_radar_baselink_transform).cast<float>();
      initial_transform_radar_baselink.translation()(2) = 0.f;
      current_transform.linear() = tf2::transformToEigen(init_transform_tf).matrix().block<2,2>(0,0).cast<float>();
      current_transform.translation() = tf2::transformToEigen(init_transform_tf).matrix().block<2,1>(0,3).cast<float>();
      std::cout << "initial transform: " << current_transform.translation()(0) << " " << current_transform.translation()(1) << " " << Sophus::SE2f::fitToSE2(current_transform.matrix()).log()(2) << std::endl;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  else { // set them to identity
    initial_transform_radar_baselink = Eigen::Affine3f::Identity();
    current_transform = Eigen::Affine2f::Identity();
  }

  // initialize frontend and backend
  _global_fuser.initialize(parameters_.global_fuser_parameters);
  _local_fuser.initialize(parameters_.local_fuser_parameters, current_transform, initial_transform_radar_baselink);

  // ROS subscribers
  _point_cloud_sub = _nh.subscribe(parameters_.scan_subscriber_topic, 1, &NDTSlam::radarCb, this);
  _odom_pub = _nh.advertise<nav_msgs::Odometry>(parameters_.odom_publisher_topic, 1);

  if (parameters_.visualize_current_scan) {
    _ndt_pub = _nh.advertise<ndt_msgs::NormalDistributions>(parameters_.scan_publisher_topic, 5);
  }
  if (parameters_.visualize_current_map) {
    _aligned_ndt_pub = _nh.advertise<ndt_msgs::NormalDistributions>(parameters_.map_publisher_topic, 5);
  }
  // initialize ros timers
  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZI>>(parameters_.scan_subscriber_topic);
  
  visualization_timer = _nh.createTimer(ros::Duration(1.0/parameters_.visualizer_frequency), &NDTSlam::visualizeMap, this);
  
  raytracing_timer = _nh.createTimer(ros::Duration(0.05), &NDTSlam::raytrace, this);
  
  global_fuser_timer = _nh.createTimer(ros::Duration(1.0/parameters_.pose_graph_frequency), &NDTSlam::optimizePoseGraph, this);

  search_loop_closure_timer = _nh.createTimer(ros::Duration(1.0/parameters_.search_loop_closure_frequency), &NDTSlam::searchLoopClosure, this);

  _trajectory_pub = _nh.advertise<nav_msgs::Path>("/estimated_trajectory", 5);
  _map_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/ndt_map", 1);
  std::cout << "finished initialization!" << std::endl;
}

void NDTSlam::radarCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  _last_header = msg->header;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  if (parameters_.use_imu) {
    _local_fuser.processImu(cache.getElemBeforeTime(cache.getLatestTime())); // process latest imu message
  }
  std::vector<pcl::PointCloud<pcl::PointXYZI>> _clusters;
	_local_fuser.processScan(msg, _clusters, nodes_, stamps_, edges_, submap_idzs_, nodes_mutex_); // process scan

  if (_local_fuser.submapComplete()) { // initialize new submap if old submap is complete
    Eigen::Affine2f current_local_transform = _local_fuser.getTransform();
    _local_fuser.initializeNewSubmap(current_local_transform);
	  _local_fuser.processScan(msg, _clusters, nodes_, stamps_, edges_, submap_idzs_, nodes_mutex_);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr debug_cloud = _local_fuser.getTransformedScan();  // vsualization clouds
  
  std_msgs::Header header = msg->header; 
  if (parameters_.visualize_current_scan) {
    HierarchicalMap hmap = _local_fuser.getLastMergedMap();
    if (!hmap.isEmpty()) {
      Eigen::Affine2f global_transform;
      global_transform = _local_fuser.getGlobalTransform();
      hmap.transformMap(global_transform);
      Map vis_map = hmap.getMap();
      ndt_msgs::NormalDistributions ndt;	
      header.frame_id = parameters_.fixed_frame; 
      createVisualizationMsg(vis_map, header, ndt);
      _ndt_pub.publish(ndt); 
    }
  }

  // compute map->odom transform
  Eigen::Affine2f current_local_transform = _local_fuser.getTransform(); 

  tf2::Stamped<tf2::Transform> odom_to_map;
  tf2::Quaternion q(0.,0.,0.,1.0);
  q.setRPY(0., 0., Sophus::SE2f::fitToSE2(current_local_transform.matrix()).log()(2));
  tf2::Stamped<tf2::Transform> base_to_map(tf2::Transform(q, tf2::Vector3(current_local_transform.translation()(0),
    current_local_transform.translation()(1), 0.0)).inverse(), msg->header.stamp, parameters_.base_frame);

  try
  {
    geometry_msgs::TransformStamped base_to_map_msg, odom_to_map_msg;
    tf2::convert(base_to_map, base_to_map_msg);
    odom_to_map_msg = tf_buffer_->transform(base_to_map_msg, parameters_.odom_frame);
    tf2::convert(odom_to_map_msg, odom_to_map);
  }
  catch(tf2::TransformException& e)
  {
    ROS_ERROR("Transform from base_link to odom failed: %s", e.what());
  }
  tf2::Transform tf_transform = tf2::Transform(tf2::Quaternion( odom_to_map.getRotation() ),
    tf2::Vector3( odom_to_map.getOrigin() ) ).inverse();
  geometry_msgs::TransformStamped trans_msg;
  tf2::convert(tf_transform, trans_msg.transform);
  trans_msg.child_frame_id = parameters_.odom_frame;
  trans_msg.header.frame_id = parameters_.fixed_frame;
  trans_msg.header.stamp = msg->header.stamp;
  tfb.sendTransform(trans_msg);

  Eigen::Affine3d current_transform_3d = Eigen::Affine3d::Identity();
  current_transform_3d.matrix().block<2,2>(0, 0) = current_local_transform.linear().cast<double>();
  current_transform_3d.matrix().block<2,1>(0, 3) = current_local_transform.translation().cast<double>();

  geometry_msgs::TransformStamped send_transform = tf2::eigenToTransform(current_transform_3d);
  send_transform.child_frame_id = "ndt_estimate";
  send_transform.header.stamp = msg->header.stamp;
  send_transform.header.frame_id = parameters_.fixed_frame;

  nav_msgs::Odometry odom_msg;
  odom_msg.header = send_transform.header;
  odom_msg.child_frame_id = parameters_.base_frame;
  odom_msg.pose.pose.position.x = send_transform.transform.translation.x;
  odom_msg.pose.pose.position.y = send_transform.transform.translation.y;
  odom_msg.pose.pose.position.z = send_transform.transform.translation.z;
  odom_msg.pose.pose.orientation = send_transform.transform.rotation;
  Eigen::Matrix<float,6,6> odom_cov = Eigen::Matrix<float,6,6>::Identity();
  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = odom_cov.data()[i];
  }
  _odom_pub.publish(odom_msg);
}

void NDTSlam::imuCb(const sensor_msgs::ImuConstPtr& msg) {
	_imu_msg = *msg;
}

void NDTSlam::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
	_odom_message = *msg;
}

//visualization stuff
void NDTSlam::visualizeMap(const ros::TimerEvent&) {
  HierarchicalMap hmap;
	Map vis_map;
  std_msgs::Header header;
  header = _last_header;
  header.frame_id = parameters_.fixed_frame; 

	_local_fuser.getSubmap(hmap);
  Eigen::Affine2f global_transform;
  global_transform = _local_fuser.getGlobalTransform();
  if (parameters_.visualize_ogm) {
    std::shared_ptr<nav_msgs::OccupancyGrid> map_ptr = _local_fuser.getOGM(ogm_mutex_); 
    map_ptr->header = header;
    _map_pub.publish(*map_ptr);
  }

  if (parameters_.visualize_current_map) {
    hmap.transformMap(global_transform);
    vis_map = hmap.getMap();
    ndt_msgs::NormalDistributions aligned_ndt;
    createVisualizationMsg(vis_map, header, aligned_ndt);
    _aligned_ndt_pub.publish(aligned_ndt);

  }
  
  if (parameters_.visualize_path) {
    nav_msgs::Path new_path;
    new_path.header = header;
    std::unique_lock<std::mutex> nl(nodes_mutex_);
    for (const auto& node: nodes_) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = node.second.pose.translation()(0);
      pose.pose.position.y = node.second.pose.translation()(1);
      pose.pose.orientation.w = std::cos(node.second.pose.log()(2)/2);
      pose.pose.orientation.z = std::sin(node.second.pose.log()(2)/2);
      pose.header = stamps_.at(node.first);
      new_path.poses.push_back(pose);
    }
    _trajectory_pub.publish(new_path);  
  }
}

// optimize pose graph
void NDTSlam::optimizePoseGraph(const ros::TimerEvent&) {
  if (!nodes_.empty() && !edges_.empty() && std::prev(submap_idzs_.end())->second > 0) {
    // calculate number of nodes per submap
    const int n_nodes_per_submap = std::ceil(static_cast<double>(parameters_.local_fuser_parameters.submap_size_poses - (parameters_.local_fuser_parameters.ndt_matcher_parameters.smoothing_steps-1)) / parameters_.local_fuser_parameters.insertion_step);
    int max_update_index = static_cast<int>(std::prev(nodes_.end())->first / n_nodes_per_submap) * n_nodes_per_submap;
    _global_fuser.optimizePoseGraph(nodes_, edges_, nodes_mutex_, max_update_index); 
    std::unique_lock<std::mutex> lock(nodes_mutex_);
    _local_fuser.updateSubmaps(nodes_, submap_idzs_); // we update the submaps with their new poses
  }
}

void NDTSlam::searchLoopClosure(const ros::TimerEvent&) {
  _local_fuser.detectLoopClosures(nodes_, edges_, submap_idzs_, nodes_mutex_);
}

void NDTSlam::raytrace(const ros::TimerEvent&) {
  _local_fuser.raytrace(ogm_mutex_);
}

void NDTSlam::createVisualizationMsg(const Map& map, const std_msgs::Header& header, ndt_msgs::NormalDistributions& ndt) {
	ndt.header = header;
  
	for (unsigned int i = 0; i < map.get_n_cells(); i++){
		ndt_msgs::NormalDistribution nd;
    Eigen::Vector3f mean;
		Eigen::Matrix3f cov;
		double intensity;
		if (map.getCellMeanAndCovariance(i, mean, cov)) {
			nd.mean.x = mean[0];
			nd.mean.y = mean[1];
      nd.mean.i = mean[2];
			nd.covariance.xx = cov(0,0);
			nd.covariance.xy = cov(0,1);
      nd.covariance.xi = cov(0,2);
			nd.covariance.yy = cov(1,1);
      nd.covariance.yi = cov(1,2);
      nd.covariance.ii = cov(2,2);
			intensity = map.getCells()[i].getMaxIntensity();
			nd.mean_intensity.data = intensity/100.0;
			ndt.normal_distributions.push_back(nd);
		}
	}
}

// Only read parameters

void NDTSlam::readParameters() {
  NDTSlamParameters ndt_slam_params;
  GlobalFuserParameters global_fuser_params;
  LocalFuserParameters local_fuser_params;
  ScanContextParameters scan_context_params;
  NDTMatcherParameters matcher_params;
  NDTMapParameters map_params;
  OGMMapParameters ogm_params;
  NDTCellParameters cell_params;
  RadarPreprocessorParameters radar_params;

  std::vector<double> motion_sqrtI_vector;
  std::vector<double> initial_sqrtI_vector;
  std::vector<double> loop_sqrtI_vector;
  std::vector<float> cell_cov_vector;

  if (!_nh.getParam("/ndt_slam/scan_publisher_topic", ndt_slam_params.scan_publisher_topic)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/map_publisher_topic", ndt_slam_params.map_publisher_topic)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/scan_subscriber_topic", ndt_slam_params.scan_subscriber_topic)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/imu_subscriber_topic", ndt_slam_params.imu_subscriber_topic)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/odom_publisher_topic", ndt_slam_params.odom_publisher_topic)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/fixed_frame", ndt_slam_params.fixed_frame)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/odom_frame", ndt_slam_params.odom_frame)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/base_frame", ndt_slam_params.base_frame)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/sensor_frame", ndt_slam_params.sensor_frame)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/visualize_ogm", ndt_slam_params.visualize_ogm)){
    ndt_slam_params.visualize_ogm = false;
    ROS_INFO("set parameter visualize_ogm to default value false");
  }
  if (!_nh.getParam("/ndt_slam/visualize_current_scan", ndt_slam_params.visualize_current_scan)){
    ndt_slam_params.visualize_current_scan= false;
    ROS_INFO("set parameter visualize_current_scan to default value false");
  }
  if (!_nh.getParam("/ndt_slam/visualize_current_map", ndt_slam_params.visualize_current_map)){
    ndt_slam_params.visualize_current_map = false;
    ROS_INFO("set parameter visualize_current_map to default value false");
  }
  if (!_nh.getParam("/ndt_slam/use_imu", ndt_slam_params.use_imu)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/visualize_path", ndt_slam_params.visualize_path)){
    ndt_slam_params.visualize_path = false;
    ROS_INFO("set parameter visualize_path to default value false");
  }
  if (!_nh.getParam("/ndt_slam/initialize_from_tf", ndt_slam_params.initialize_from_tf)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_slam/visualizer_frequency", ndt_slam_params.visualizer_frequency)){
    ndt_slam_params.visualizer_frequency = 0.5;
    ROS_INFO("set parameter visualizer_frequency to default value 0.5");
  }
  if (!_nh.getParam("/ndt_slam/pose_graph_frequency", ndt_slam_params.pose_graph_frequency)){
    ndt_slam_params.pose_graph_frequency = 0.1;
    ROS_INFO("set parameter pose_graph_frequency to default value 0.1");
  }
  if (!_nh.getParam("/ndt_slam/search_loop_closure_frequency", ndt_slam_params.search_loop_closure_frequency)){
    ndt_slam_params.search_loop_closure_frequency = 1;
    ROS_INFO("set parameter search_loop_closure_frequency to default value 1");
  }
  if (!_nh.getParam("/global_fuser/use_robust_loss", global_fuser_params.use_robust_loss)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/insertion_step", local_fuser_params.insertion_step)){
    ROS_WARN("failed to load parameter");
  }  
  if (!_nh.getParam("/local_fuser/submap_size_poses", local_fuser_params.submap_size_poses)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/submap_overlap", local_fuser_params.submap_overlap)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/loop_closure_max_cs_divergence", local_fuser_params.loop_closure_max_cs_divergence)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/loop_closure_weight", local_fuser_params.loop_closure_weight)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/max_data_association_mahalanobis_dist", local_fuser_params.max_data_association_mahalanobis_dist)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/compute_dfs_loop_closure", local_fuser_params.compute_dfs_loop_closure)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/use_scan_context_as_loop_closure", local_fuser_params.use_scan_context_as_loop_closure)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/loop_sqrtI", loop_sqrtI_vector)){
    ROS_WARN("failed to load parameter");
  }   
  Eigen::Matrix<double,3,3> loop_matrix(loop_sqrtI_vector.data());
  local_fuser_params.loop_sqrtI = loop_matrix;
  if (!_nh.getParam("/scan_context/num_ring", scan_context_params.PC_NUM_RING)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/num_sector", scan_context_params.PC_NUM_SECTOR)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/max_radius", scan_context_params.PC_MAX_RADIUS)){
    ROS_WARN("failed to load parameter");
  }
  scan_context_params.PC_UNIT_SECTORANGLE = 360.0 / double(scan_context_params.PC_NUM_SECTOR);
  scan_context_params.PC_UNIT_RINGGAP = scan_context_params.PC_MAX_RADIUS / double(scan_context_params.PC_NUM_RING);
  if (!_nh.getParam("/scan_context/num_exclude_recent", scan_context_params.NUM_EXCLUDE_RECENT)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/num_candidates_from_tree", scan_context_params.NUM_CANDIDATES_FROM_TREE)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/search_ratio", scan_context_params.SEARCH_RATIO)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/dist_thresh", scan_context_params.SC_DIST_THRES)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/tree_making_period", scan_context_params.TREE_MAKING_PERIOD_)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/assumed_drift", scan_context_params.assumed_drift)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/odom_eps", scan_context_params.odom_eps)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/odom_weight", scan_context_params.odom_weight)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/scan_context/intensity_factor", scan_context_params.intensity_factor)){
    ROS_WARN("failed to load parameter");
  }  
  if (!_nh.getParam("/ndt_matcher/motion_sqrtI", motion_sqrtI_vector)){
    ROS_WARN("failed to load parameter");
  }   
  Eigen::Matrix<double,8,8> matrix(motion_sqrtI_vector.data());
  matcher_params.motion_sqrtI = matrix;
  if (!_nh.getParam("/ndt_matcher/covariance_scaling_factor", matcher_params.covariance_scaling_factor)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/weight_imu", matcher_params.weight_imu)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/weight_imu_bias", matcher_params.weight_imu_bias)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/initial_imu_bias", matcher_params.initial_imu_bias)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/gnc_steps", matcher_params.gnc_steps)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/loop_closure_gnc_steps", local_fuser_params.loop_closure_gnc_steps)){
    local_fuser_params.loop_closure_gnc_steps = matcher_params.gnc_steps;
  }
  if (!_nh.getParam("/ndt_matcher/smoothing_steps", matcher_params.smoothing_steps)){
    ROS_WARN("failed to load parameter");
  }
  matcher_params.use_imu = ndt_slam_params.use_imu;
  local_fuser_params.insertion_delay = matcher_params.smoothing_steps + 1; 
  if (!_nh.getParam("/ndt_matcher/loss_function_scale", matcher_params.loss_function_scale)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/loop_closure_scale", local_fuser_params.loop_closure_scale)){
    local_fuser_params.loop_closure_scale = matcher_params.loss_function_scale;
  }
  if (!_nh.getParam("/ndt_matcher/loss_function_convexity", matcher_params.loss_function_convexity)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/gnc_control_parameter_divisor", matcher_params.gnc_control_parameter_divisor)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/max_iteration", matcher_params.max_iteration)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/pose_reject_translation", matcher_params.pose_reject_translation)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/pose_reject_rotation", matcher_params.pose_reject_rotation)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/n_results_nn_lookup", matcher_params.n_results_kd_lookup)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/ndt_weight", matcher_params.ndt_weight)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/use_analytic_expressions_for_optimization", matcher_params.use_analytic_expressions_for_optimization)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/use_intensity_as_dimension", matcher_params.use_intensity_as_dimension)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/local_fuser/use_intensity_in_loop_closure", local_fuser_params.use_intensity_in_loop_closure)){
    local_fuser_params.use_intensity_in_loop_closure = matcher_params.use_intensity_as_dimension;
  }
  if (!_nh.getParam("/ndt_matcher/use_constant_velocity_model", matcher_params.use_constant_velocity_model)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/optimize_on_manifold", matcher_params.optimize_on_manifold)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/lookup_distribution", matcher_params.lookup_mahalanobis)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/csm_window_linear", matcher_params.csm_window_linear)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/csm_window_angular", matcher_params.csm_window_angular)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/csm_linear_step", matcher_params.csm_linear_step)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/csm_cost_threshold", matcher_params.csm_cost_threshold)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/csm_max_px_accurate_range", matcher_params.csm_max_px_accurate_range)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_matcher/csm_n_iter", matcher_params.csm_n_iter)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_map/size_x", map_params.size_x)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_map/size_y", map_params.size_y)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_map/resolution", map_params.resolution)){
    ROS_WARN("failed to load parameter");
  }
  map_params.size_x/=map_params.resolution;
  map_params.size_y/=map_params.resolution;
  if (!_nh.getParam("/ogm_map/size_x", ogm_params.size_x)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ogm_map/size_y", ogm_params.size_y)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ogm_map/resolution", ogm_params.resolution)){
    ROS_WARN("failed to load parameter");
  }
  ogm_params.size_x/=ogm_params.resolution;
  ogm_params.size_y/=ogm_params.resolution;
  ogm_params.submap_size_x = map_params.size_x * map_params.resolution / ogm_params.resolution;
  ogm_params.submap_size_y = map_params.size_y * map_params.resolution / ogm_params.resolution;
  ogm_params.ndt_resolution = map_params.resolution;
  map_params.ogm_resolution = ogm_params.resolution;
  map_params.ogm_threshold = ogm_params.ogm_threshold;
  if (!_nh.getParam("/ndt_map/min_points_per_cell", map_params.min_points_per_cell)){
    ROS_WARN("failed to load parameter");
  } 
  else {
    radar_params.min_points_per_cell = map_params.min_points_per_cell;
  }
  if (!_nh.getParam("/ndt_map/max_neighbor_linf_distance", map_params.max_neighbour_manhattan_distance)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/ndt_cell/beam_cov", cell_cov_vector)){
    ROS_WARN("failed to load parameter");
  }
  Eigen::Matrix<float,3,3> cov_matrix(cell_cov_vector.data());
  cell_params.beam_cov = cov_matrix;
  if (!_nh.getParam("/ndt_cell/use_pndt", cell_params.use_pndt)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/radar_preprocessor/max_range", radar_params.max_range)){
    ROS_WARN("failed to load parameter");
  }
  radar_params.n_clusters = std::pow(2.0 * radar_params.max_range / map_params.resolution, 2);
  if (!_nh.getParam("/radar_preprocessor/min_intensity", radar_params.min_intensity)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/radar_preprocessor/min_range", radar_params.min_range)){
    ROS_WARN("failed to load parameter");
  }
  if (!_nh.getParam("/radar_preprocessor/beam_distance_increment_threshold", radar_params.beam_distance_increment_threshold)){
    ROS_WARN("failed to load parameter");
  }
  radar_params.base_frame = ndt_slam_params.base_frame;
  radar_params.sensor_frame = ndt_slam_params.sensor_frame;
  map_params.ndt_cell_parameters = cell_params;
  local_fuser_params.radar_preprocessor_parameters = radar_params;
  local_fuser_params.ndt_map_parameters = map_params;
  local_fuser_params.ogm_map_parameters = ogm_params;
  local_fuser_params.ndt_matcher_parameters = matcher_params;
  local_fuser_params.scan_context_parameters = scan_context_params;
  ndt_slam_params.local_fuser_parameters = local_fuser_params;
  ndt_slam_params.global_fuser_parameters = global_fuser_params;
  parameters_ = ndt_slam_params;
}

}
}
}