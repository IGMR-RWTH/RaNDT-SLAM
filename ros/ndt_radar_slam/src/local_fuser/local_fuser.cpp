#include "local_fuser/local_fuser.h"

namespace rc {
namespace navigation {
namespace ndt {

void LocalFuser::initialize(LocalFuserParameters parameters, Eigen::Affine2f initial_transform, Eigen::Affine3f initial_transform_radar_baselink) {
  // frames
  Eigen::Affine2f radar_to_baselink_2d;
  radar_to_baselink_2d.linear() = initial_transform_radar_baselink.linear().block<2,2>(0,0);
  radar_to_baselink_2d.translation() = initial_transform_radar_baselink.translation().block<2,1>(0,0);
  baselink_to_radar_2d_ = radar_to_baselink_2d.inverse();
  parameters_ = parameters;
  if (parameters_.clustering_method == "Grid") {
    _preprocessor.initialize(ClusteringType::Grid, parameters_.radar_preprocessor_parameters,  initial_transform_radar_baselink);
  } 
  else {
    std::cout << "ERROR: unsupported clustering type" << std::endl;
  }
  while (write_on_queue_ || read_on_queue_) {} // needed for raytracing
  write_on_queue_ = true;
  raytracing_queue_.clear();
  write_on_queue_ = false;
  
  // initialize components
  matcher_.initialize(parameters_.ndt_matcher_parameters); // ndt matcher
  _current_submap.initialize(parameters_.ndt_map_parameters, Eigen::Vector2f::Zero(), initial_transform); // each submap has its own frame
  _current_scan_map.initialize(parameters_.ndt_map_parameters, Eigen::Vector2f::Zero(), initial_transform); // sensor-centered map
  ogm_.initialize(parameters_.ogm_map_parameters, initial_transform); // ogm of all submaps
  scManager_.initialize(parameters_.scan_context_parameters);    // for loop closure
  std::map<int, std::pair<Eigen::Affine2f, std::shared_ptr<std::vector<int>>>> submap_count; // each submaps counters for ogm generation
  submap_count[0] = _current_submap.getSubmapCounts(); 
  ogm_.updateSubmaps(submap_count);
  current_transform_ = Sophus::SE2d(Eigen::Affine2d::Identity().matrix()); 
  current_global_transform_ = Sophus::SE2d(std::atan2(initial_transform.cast<double>().rotation()(1,0),initial_transform.cast<double>().rotation()(0,0)), initial_transform.cast<double>().translation());
  last_imu_bias_ = parameters_.ndt_matcher_parameters.initial_imu_bias;
  n_finished_submaps_ = 0;
}

void LocalFuser::initializeNewSubmap(Eigen::Affine2f initial_transform) {
  std::cout << "Initializing new Submap!" << std::endl;
  last_state_ = trajectory_.end()[-1]; // get correct state for next submap
  submaps_.insert({n_finished_submaps_, _current_submap}); // push last submap to completed submaps
  _last_submap_transformed = _current_submap; // for overlapping submaps
  Eigen::Affine2f old_submap_to_new_submap = Eigen::Affine2f(current_global_transform_.inverse().cast<float>().matrix()) * initial_transform;
  _last_submap_transformed.transformMap(old_submap_to_new_submap); // old submap in new submap frame
  _next_maps_to_insert.clear();
  _next_scans_to_insert.clear();
  _next_stamps_to_insert.clear();
  _next_max_to_insert.clear();      // reset buffers
  matcher_.resetMatcher();
  current_transform_ = Sophus::SE2d(Eigen::Affine2d::Identity().matrix());
  current_global_transform_ = Sophus::SE2d::fitToSE2(initial_transform.cast<double>().matrix());
  _current_submap.clear();
  _current_submap.initialize(parameters_.ndt_map_parameters, Eigen::Vector2f::Zero(), initial_transform); // each submap has its own frame
  trajectory_.clear();    // reset trajectory
  n_finished_submaps_++;
  map_window_.clear();    
  while (write_on_queue_ || read_on_queue_) {}
  write_on_queue_ = true;
  raytracing_queue_.clear();
  write_on_queue_ = false;
}

void LocalFuser::updateSubmaps(const std::map<int, Pose>& nodes, const std::map<int, int>& submap_idzs) {
  std::map<int, std::pair<Eigen::Affine2f, std::shared_ptr<std::vector<int>>>> submap_counts;
  for (int i=0; i<=n_finished_submaps_; i++) {
    int min_idx = 0; // we do not want to loop multiple times over the whole array
    for (int j=min_idx; j<nodes.size(); j++) {
      if (submap_idzs.at(j) == i) {  
        if (i < n_finished_submaps_) {
          submaps_.at(i).transformMapToOrigin(nodes.at(j).pose);  // submap positions are updated
          submap_counts[i] = submaps_.at(i).getSubmapCounts();    // simple grid for ogm generation
          min_idx = j+1;
          break;
        }
        else {
          _current_submap.transformMapToOrigin(nodes.at(j).pose);
          current_global_transform_ = nodes.at(j).pose;            // submap origin in global frame
          current_node_ = std::prev(nodes.end())->second;          // correct node for new submap
          submap_counts[i] = _current_submap.getSubmapCounts();
          ogm_.updateSubmaps(submap_counts);
          return;
        }
      }
    }
  }
}

void LocalFuser::processImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg != nullptr) {
    imu_msg_ = *msg;        // simply write message into variable
  } 
  else {
    ROS_WARN("Error with IMU msg");
  }
}

void LocalFuser::processScan(const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters, std::map<int, Pose>& nodes, std::map<int, std_msgs::Header>& stamps, std::vector<Constraint>& edges, std::map<int, int>& submap_idzs, std::mutex& nodes_mutex) {
  std::vector<std::vector<std::pair<double, double>>> polar_detections;
  std::vector<std::tuple<double, double, double>> peak_detections;
  _preprocessor.processScan(msg, clusters, polar_detections, peak_detections); // get clusters, polar representations and maximum intensity beams
  HierarchicalMap current_scan;
  current_scan.initialize(parameters_.ndt_map_parameters, Eigen::Vector2f::Zero(), Eigen::Affine2f::Identity());
  current_scan.addClusters(clusters, polar_detections, peak_detections); // create ndt of scan
  
  // do matching only if the submap is currently holding data
  if (!_current_submap.isEmpty()) {
    double roll, pitch, yaw;
    if (parameters_.ndt_matcher_parameters.use_imu) {
      // get relative orientation chnage based on IMU measurementsscan
      Eigen::Affine2f imu_diff = Eigen::Affine2f::Identity();
      tf2::Quaternion q_old_imu, q_new_imu, q_rel_imu;
      tf2::convert(last_imu_msg_.orientation, q_old_imu);
      tf2::convert(imu_msg_.orientation, q_new_imu);
      q_old_imu[3] = -q_old_imu[3];
      q_rel_imu = q_new_imu * q_old_imu;
      tf2::Matrix3x3 m(q_rel_imu);
      m.getRPY(roll, pitch, yaw);
    }
    else {
      yaw = 0;
    }
    // get initial guess
    matcher_.predictTransform(yaw, msg->header.stamp.toSec(), trajectory_);

    // get moving and fixed ndts
    Map fmap = _current_submap.getMap();
    Map mmap = current_scan.getMap();
    map_window_.push_back(mmap);
    std::deque<Map> f_maps;
    f_maps.push_back(fmap);
    if (trajectory_.size() < parameters_.submap_overlap && n_finished_submaps_ > 0) {
      Map old_fmap = _last_submap_transformed.getMap();
      f_maps.push_back(old_fmap);
    }
    // estimate transformation
    matcher_.estimateTransformCeres(current_transform_, trajectory_, yaw, msg->header.stamp.toSec(), f_maps, map_window_);
    
    // write both pose representations
    for (size_t i = 1; i <= std::min(parameters_.ndt_matcher_parameters.smoothing_steps, static_cast<int>(trajectory_.size())); i++) {
      if (parameters_.ndt_matcher_parameters.use_analytic_expressions_for_optimization ||
        !parameters_.ndt_matcher_parameters.optimize_on_manifold) {
        trajectory_.end()[-i].pose = Sophus::SE2d(trajectory_.end()[-i].rot, trajectory_.end()[-i].pos);
      }
      else {
        trajectory_.end()[-i].pos = trajectory_.end()[-i].pose.translation();
        trajectory_.end()[-i].rot = trajectory_.end()[-i].pose.log()(2);
      }
    }
    size_t trajectory_size = trajectory_.size();
    if (map_window_.size() >= parameters_.ndt_matcher_parameters.smoothing_steps) { // old scan dropped
      map_window_.pop_front();
    }
    if ((trajectory_size % parameters_.insertion_step) == 0) {  // keyframe, pushed on buffer
      _next_maps_to_insert.push_back(current_scan);
      _next_scans_to_insert.push_back(getTransformedScan());
      _next_stamps_to_insert.push_back(msg->header);
      _next_stamps_to_insert.back().frame_id = "map";
      _next_max_to_insert.push_back(peak_detections);
    }
    
    // if keyframe exits estimator
    if ((trajectory_size >= parameters_.insertion_delay + parameters_.insertion_step) && ((trajectory_size-parameters_.insertion_delay) % parameters_.insertion_step == 0)) {
      State X_smoothed = trajectory_.end()[-parameters_.insertion_delay-1]; 
      Sophus::SE2d smoothed_transform = X_smoothed.pose;
      if (parameters_.ndt_matcher_parameters.use_analytic_expressions_for_optimization ||
          !parameters_.ndt_matcher_parameters.optimize_on_manifold) {
        smoothed_transform = Sophus::SE2d(X_smoothed.rot, X_smoothed.pos);
      }
     
      // transform map to last estimate
      Map smoothed_map = _next_maps_to_insert.front().getMap();
      Map global_map =_current_submap.getMap();
      const Eigen::Affine2f smoothed_eigen_transform(smoothed_transform.cast<float>().matrix());
      scans_[current_node_id_] = _next_maps_to_insert.front(); // before transforming...
      _next_maps_to_insert.front().transformMapWithPointCloud(smoothed_eigen_transform);
      _last_merged_map = _next_maps_to_insert.front();

      // trigger raytracing
      if(parameters_.visualize_ogm) {
        while(read_on_queue_ || write_on_queue_) {}
        write_on_queue_ = true;
        for (auto& line: _next_max_to_insert.front()) {
          raytracing_queue_.emplace_back(std::make_pair(smoothed_eigen_transform * baselink_to_radar_2d_, line));
        }
        write_on_queue_ = false;
      }
      // merge into map
      _current_submap.mergeMapCell(_next_maps_to_insert.front());

      // add node and edge<<<<<<<<
      Sophus::SE2d total_transform = current_global_transform_ * smoothed_transform;
      Pose pose;
      pose.pose = total_transform;
      pose.pos = total_transform.translation();
      pose.rot = total_transform.log()(2);
      Constraint constraint;
      constraint.id_begin = current_node_id_-1;
      constraint.id_end = current_node_id_;
      constraint.trans = nodes.at(current_node_id_-1).pose.inverse() * pose.pose;
      pose.traversed_dist = nodes.at(current_node_id_-1).traversed_dist + constraint.trans.translation().norm();
      constraint.sqrt_information << 10,  0, 0,
                                      0, 10, 0,
                                      0,  0, 50;

      scManager_.makeAndSaveScancontextAndKeys(_next_scans_to_insert.front(), pose.pos, pose.traversed_dist);
    
      // append new graph nodes and constraints
      std::unique_lock<std::mutex> lock(nodes_mutex);
      nodes[current_node_id_] = pose;
      stamps[current_node_id_] = _next_stamps_to_insert.front();
      lock.unlock();
      submap_idzs[current_node_id_] = n_finished_submaps_;
      edges.push_back(constraint);
      _next_maps_to_insert.pop_front();
      _next_scans_to_insert.pop_front();
      _next_stamps_to_insert.pop_front();
      _next_max_to_insert.pop_front();
      current_node_ = pose;
      _next_maps_to_search_loop.push_back(current_node_id_);
      current_node_id_++;
    }
  }
  // if this is the first scan of the submap
  else {
    State initial_state;
    initial_state.pose = current_transform_;
    initial_state.pos = current_transform_.translation();
    initial_state.rot = current_transform_.log()(2);
    if (n_finished_submaps_ == 0){
      initial_state.lin_vel = Eigen::Vector2d::Zero();
      initial_state.rot_vel = 0.0;
      initial_state.lin_acc = Eigen::Vector2d::Zero();
      initial_state.imu_bias = last_imu_bias_;
    } 
    else {
      initial_state.lin_vel = last_state_.lin_vel;
      initial_state.rot_vel = last_state_.rot_vel;
      initial_state.lin_acc = last_state_.lin_acc;
      initial_state.imu_bias = last_state_.imu_bias;
    }
    initial_state.stamp = msg->header.stamp.toSec();
    trajectory_.push_back(initial_state);

    Pose initial_pose;
    initial_pose.pose = current_global_transform_;
    initial_pose.pos = initial_pose.pose.translation();
    initial_pose.rot = initial_pose.pose.log()(2);
    std::unique_lock<std::mutex> lock(nodes_mutex);
    nodes[current_node_id_] = initial_pose;
    lock.unlock();
    scans_[current_node_id_] = current_scan;
    stamps[current_node_id_] = msg->header;
    stamps[current_node_id_].frame_id = "map";
    submap_idzs[current_node_id_] = n_finished_submaps_;
    //if its not the first submap
    if (initial_node_set_) {
      Constraint initial_constraint;
      initial_constraint.id_begin = current_node_id_-1;
      initial_constraint.id_end = current_node_id_;
      initial_constraint.trans = nodes.at(current_node_id_-1).pose.inverse() * initial_pose.pose;
      initial_pose.traversed_dist = nodes.at(current_node_id_-1).traversed_dist + initial_constraint.trans.translation().norm();
      initial_constraint.sqrt_information << 10,  0, 0,
                                              0, 10, 0,
                                              0,  0, 50;
      edges.push_back(initial_constraint);
    }
    else {
      initial_pose.traversed_dist = 0;
    }
    current_submap_root_node_ = initial_pose;
    current_submap_root_node_id_ = current_node_id_;
    root_nodes_[n_finished_submaps_] = current_node_id_;
    nodes[current_node_id_] = initial_pose;
    current_node_ = initial_pose;
    current_node_id_++;
    initial_node_set_ = true;

    const Eigen::Affine2f current_eigen_transform(current_transform_.cast<float>().matrix());
    current_scan.transformMapWithPointCloud(current_eigen_transform);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud = getTransformedScan();
    scManager_.makeAndSaveScancontextAndKeys(current_cloud, initial_pose.pos, initial_pose.traversed_dist);
    if (parameters_.visualize_ogm) {
      while(read_on_queue_ || write_on_queue_) {}
      write_on_queue_ = true;
      for (auto& line: peak_detections) { 
        raytracing_queue_.emplace_back(std::make_pair(baselink_to_radar_2d_, line));
      }
      write_on_queue_ = false;
    }
    _current_submap.mergeMapCell(current_scan);
    _last_merged_map = current_scan;
  }
  _current_scan_map = current_scan;
  last_imu_msg_ = imu_msg_;

  n_scans++;
}

void LocalFuser::raytrace(std::mutex& map_mutex) {
  while (!raytracing_queue_.empty()) {
    // try raytracing only if no ogm is generated at the moment
    if (!write_on_queue_) {
      read_on_queue_ = true;
      const std::pair<Eigen::Affine2f, std::tuple<double, double, double>> line = raytracing_queue_.front();
      write_on_queue_ = true;
      read_on_queue_ = false;
      raytracing_queue_.pop_front();
      write_on_queue_ = false;
      std::unique_lock<std::mutex> mm(map_mutex);
      _current_submap.raytraceLine(line.first, line.second, 2 * std::get<1>(line.second));   
    }
  }
}

void LocalFuser::detectLoopClosures(std::map<int, Pose>& nodes, std::vector<Constraint>& edges, std::map<int, int>& submap_idzs, std::mutex& nodes_mutex) {// search for possible loop closures
  while (!_next_maps_to_search_loop.empty()) {  // for all query scans
    std::map<int, std::pair<int, double>> best_submap_matches;   // here, the best match for each submap is sotred
    const int current_node_idx = _next_maps_to_search_loop.front(); // id of query node
    if (parameters_.use_scan_context_as_loop_closure) {
      auto detectResult = scManager_.detectLoopClosureID(current_node_idx);  // get sc candidate
      int SCclosestHistoryFrameID = detectResult.first;
      if (SCclosestHistoryFrameID != -1 && submap_idzs.at(current_node_idx) != submap_idzs.at(SCclosestHistoryFrameID)) { // only targets in different submaps
        const int prev_node_idx = SCclosestHistoryFrameID;
        
        // start loop refinement
        Map f_loop_map = submaps_.at(submap_idzs.at(prev_node_idx)).getMap();
        Sophus::SE2d submap_transform = nodes.at(root_nodes_.at(submap_idzs.at(prev_node_idx))).pose; 
        Sophus::SE2d end_transform = nodes.at(prev_node_idx).pose;
        Map m_loop_map = scans_.at(current_node_idx).getMap();
        Sophus::SE2d estimated_difference = submap_transform.inverse() * end_transform * Sophus::SE2d(-detectResult.second, {0.0, 0.0});
        Map cs_m_loop_map = m_loop_map;
        double loop_closure_cost = matcher_.estimateLoopConstraint(estimated_difference, f_loop_map, m_loop_map, parameters_.loop_closure_gnc_steps, parameters_.use_intensity_in_loop_closure, parameters_.loop_closure_scale);
        
        // check with cs divergence
        cs_m_loop_map.transformMap(Eigen::Affine2f(estimated_difference.cast<float>().matrix()));
        double cs_divergence = f_loop_map.calculateCSDivergence(cs_m_loop_map);  
        if (cs_divergence < parameters_.loop_closure_max_cs_divergence) {
          Constraint loop_constraint;
          loop_constraint.id_end = current_node_idx;
          loop_constraint.id_begin = root_nodes_.at(submap_idzs.at(prev_node_idx));
          loop_constraint.trans = estimated_difference;

          loop_constraint.sqrt_information = parameters_.loop_closure_weight * parameters_.loop_sqrtI;
          edges.push_back(loop_constraint);
        } 
      }
    }
    else {
      // search for closest scan in each submap
      std::map<int, std::pair<int, double>> best_submap_matches;
      for (const auto& pose_i : nodes) { 
        if (submap_idzs.at(pose_i.first) != submap_idzs.at(current_node_idx) && submap_idzs.at(pose_i.first) != n_finished_submaps_) {
            //mathematically incorrect for manifold
          const double dist = std::sqrt((nodes.at(current_node_idx).pose.translation()-pose_i.second.pose.translation()).transpose() * pose_i.second.cov_pos_pos.inverse() * (nodes.at(current_node_idx).pose.translation()-pose_i.second.pose.translation()));
          if (dist<parameters_.max_data_association_mahalanobis_dist) {
            if (best_submap_matches.find(submap_idzs.at(pose_i.first)) == best_submap_matches.end() || dist < best_submap_matches.at(submap_idzs.at(pose_i.first)).second) {
              best_submap_matches[submap_idzs.at(pose_i.first)] = std::make_pair(pose_i.first, dist);
            }
          }
        }
      }
      
      double brute_force_cost = 1000.0;

      // compute possible loop closures
      for (const auto& match : best_submap_matches) { 
        Map f_loop_map = submaps_.at(match.first).getMap();
        Map m_loop_map = scans_.at(current_node_idx).getMap();

        Sophus::SE2d root_transform = nodes.at(root_nodes_.at(match.first)).pose; 
        Sophus::SE2d end_transform = nodes.at(current_node_idx).pose;

        Sophus::SE2d estimated_difference = root_transform.inverse() * end_transform;

        // search with csm
        if (parameters_.compute_dfs_loop_closure) {
          Eigen::Matrix3d current_cov = nodes.at(match.second.first).cov;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> cov_s;
          cov_s.compute(current_cov.block<2,2>(0,0));
          const double max_linear_eigenvalue = std::abs(cov_s.eigenvalues()(0));
          const double search_window_linear = parameters_.max_data_association_mahalanobis_dist * max_linear_eigenvalue;
          const double search_window_angular = std::min(2 * M_PI, parameters_.max_data_association_mahalanobis_dist * std::sqrt(current_cov(2,2)));
          std::cout << "searching loop closure with linear search window size " << search_window_linear << " and angular search window size " << search_window_angular * 180.0 / M_PI << "° \n"; 
          brute_force_cost = matcher_.estimateTransformGlobalBNB(estimated_difference, f_loop_map, m_loop_map, parameters_.use_intensity_in_loop_closure, parameters_.loop_closure_scale, search_window_linear, search_window_angular);
        } 
        else {
          brute_force_cost = 0;
        }
  
        // refine and check cs divergence
        Map cs_m_loop_map = m_loop_map;
        double loop_closure_cost = matcher_.estimateLoopConstraint(estimated_difference, f_loop_map, m_loop_map, parameters_.loop_closure_gnc_steps, parameters_.use_intensity_in_loop_closure, parameters_.loop_closure_scale);
        cs_m_loop_map.transformMap(Eigen::Affine2f(estimated_difference.cast<float>().matrix()));
        double cs_divergence = f_loop_map.calculateCSDivergence(cs_m_loop_map);

        std::cout << "loop cost: " << loop_closure_cost << std::endl;
        std::cout << "cs divergence: " << cs_divergence << std::endl;
        
        if (cs_divergence < parameters_.loop_closure_max_cs_divergence) {
          Constraint loop_constraint;
          loop_constraint.id_end = current_node_idx;
          loop_constraint.id_begin = root_nodes_.at(match.first);
          loop_constraint.trans = estimated_difference;

          loop_constraint.sqrt_information = parameters_.loop_closure_weight * parameters_.loop_sqrtI;
          edges.push_back(loop_constraint);
        }
      }
  
    }
    _next_maps_to_search_loop.pop_front(); // searched for query scan
  }
}

HierarchicalMap LocalFuser::getLastMergedMap() {
  return _last_merged_map;
}
void LocalFuser::getSubmap(HierarchicalMap& map) {
  map = _current_submap;
}
std::vector<State> LocalFuser::getTrajectory() {
  return trajectory_;
}

bool LocalFuser::submapComplete() {
  return trajectory_.size() >= parameters_.submap_size_poses;
}
}
}
}