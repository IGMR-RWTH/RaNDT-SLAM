#include <ndt_representation/ndt_master_map.h>

namespace rc {
namespace navigation {
namespace ndt {

void MasterMap::initialize(OGMMapParameters parameters, Eigen::Affine2f origin_in_global_frame) {
  parameters_ = parameters;
  origin_in_global_frame_ = origin_in_global_frame; 
  to_ogm_map_origin_.translation()(0) = -parameters.size_x * parameters.resolution / 2.;
  to_ogm_map_origin_.translation()(1) = -parameters.size_y * parameters.resolution / 2.;
  ogm_map_origin_ = origin_in_global_frame * to_ogm_map_origin_;  // root transform to ogm corner
  ogm_res_ = parameters.resolution;
  submap_ogm_res_ = ogm_res_;
  ogm_size_x_ = parameters_.size_x;
  ogm_size_y_ = parameters_.size_y;
  simple_grid_.reset(new std::vector<double>(ogm_size_x_ * ogm_size_y_, 0.));
}

std::shared_ptr<nav_msgs::OccupancyGrid> MasterMap::getOGM(std::mutex& ogm_mutex) {
  // first, to prevent aliasing we take 4 samples per cell
  std::vector<Eigen::Affine2f> in_cell_samples;
  Eigen::Affine2f sample_generator = Eigen::Affine2f::Identity();
  const double sampling_ratio = 0.25; // shift each sample by 0.25 to ensure evenly spaced sampling
  sample_generator.translation()(0) = -sampling_ratio * ogm_res_;
  sample_generator.translation()(1) = -sampling_ratio * ogm_res_;
  in_cell_samples.push_back(sample_generator);
  sample_generator.translation()(0) = -sampling_ratio * ogm_res_;
  sample_generator.translation()(1) = sampling_ratio * ogm_res_;
  in_cell_samples.push_back(sample_generator);
  sample_generator.translation()(0) = sampling_ratio * ogm_res_;
  sample_generator.translation()(1) = -sampling_ratio * ogm_res_;
  in_cell_samples.push_back(sample_generator);
  sample_generator.translation()(0) = sampling_ratio * ogm_res_;
  sample_generator.translation()(1) = sampling_ratio * ogm_res_;
  in_cell_samples.push_back(sample_generator);

  simple_grid_.reset(new std::vector<double>(ogm_size_x_ * ogm_size_y_, 0.));
  int max_update = submap_counts_.size()-1;
  #pragma omp parallel for //open mp for faster processing
  for (int i = 0; i <= max_update; i++) {
    std::unique_lock<std::mutex> ogm_lock(ogm_mutex, std::defer_lock); // lock only for most-recent ogm
    if (i == max_update) {
      ogm_lock.lock();
    }
    auto submap = std::make_pair(i, submap_counts_.at(i)); // current submap
    const Eigen::Affine2f submap_ogm_origin = ogm_map_origin_.inverse() * submap.second.first; // get origin of submap
    std::map<unsigned int, double> increments; // to prevent aliasing issues, we compare all transformed values before applying them to the global map
    for (int y = 0; y < parameters_.submap_size_y; y++) {
      for (int x = 0; x < parameters_.submap_size_x; x++) {
        const unsigned int submap_cell_index = y * parameters_.submap_size_x + x;
        if (submap.second.second->at(submap_cell_index) == 0) 
          continue;  // calculate only cells that have been updated
        Eigen::Affine2f in_submap_transform = Eigen::Affine2f::Identity();
        in_submap_transform.translation()(1) = y * submap_ogm_res_;
        in_submap_transform.translation()(0) = x * submap_ogm_res_;
        if (!submap.second.second || submap_cell_index >= submap.second.second->size()) {
          continue; // check out of bounds
        }
        for (const auto& sample: in_cell_samples) {
          Eigen::Affine2f global_cell_pose = submap_ogm_origin * in_submap_transform * sample; // pose of cell in global map frame
          const unsigned int ogm_grid_x = global_cell_pose.translation()(0) / ogm_res_;
          const unsigned int ogm_grid_y = global_cell_pose.translation()(1) / ogm_res_; 
          const unsigned int ogm_grid_cell_index = ogm_grid_y * ogm_size_x_ + ogm_grid_x; 
          if (std::abs(submap.second.second->at(submap_cell_index)) > std::abs(increments[ogm_grid_cell_index])) {
            increments[ogm_grid_cell_index] = static_cast<double>(submap.second.second->at(submap_cell_index)); // get increment
          }
          
        }
      }
    }
    for (const auto& update_index: increments) {
      if (update_index.first < simple_grid_->size()) {
        //#pragma omp critical
        if (update_index.second != 0)
          simple_grid_->at(update_index.first) += update_index.second; // update with values of map
      }
    }
  }
  std::shared_ptr<nav_msgs::OccupancyGrid> ogm = std::make_shared<nav_msgs::OccupancyGrid>(); // pointer for visualizer

  ogm->info.resolution = ogm_res_;
  ogm->info.width = ogm_size_x_;   // different resolution of ndt map and ogm map
  ogm->info.height = ogm_size_y_; 
  ogm->info.origin.position.x = ogm_map_origin_.translation()(0);
  ogm->info.origin.position.y = ogm_map_origin_.translation()(1);
  ogm->info.origin.position.z = 0.;

  const double theta = Sophus::SE2f::fitToSE2(origin_in_global_frame_.matrix()).log()(2);
  ogm->info.origin.orientation.w = std::cos(theta/2);
  ogm->info.origin.orientation.x = 0.;
  ogm->info.origin.orientation.y = 0.;
  ogm->info.origin.orientation.z = std::sin(theta/2);

  ogm->data.resize(ogm->info.width * ogm->info.height, -1);
  
  for (int i = 0; i < simple_grid_->size(); i++) {
    if (simple_grid_->at(i) != 0) {
      const double zeta = clamp(5 + 0.1 * static_cast<double>(simple_grid_->at(i)), 0., 10.);
      const double result = 100. * (-2 * std::pow(zeta/10., 3) + 3 * std::pow(zeta/10., 2));   // s function
      ogm->data.at(i) = static_cast<int>(result);
    }
  }

  return ogm;
}

}
}
}