#include <ndt_representation/ndt_hierarchical_map.h>

namespace rc {
namespace navigation {
namespace ndt {

void HierarchicalMap::clear(void) {
  Map cleared_map;
  ndt_map_ = cleared_map;
}

void HierarchicalMap::initialize(NDTMapParameters parameters, Eigen::Vector2f center, Eigen::Affine2f origin_in_global_frame) {
  parameters_ = parameters;
  ndt_map_.initialize(parameters_, center(0), center(1));       
  is_empty = true;
  origin_in_global_frame_ = origin_in_global_frame; 
  ogm_map_origin_.translation()(0) = -parameters.size_x * parameters.resolution / 2.;
  ogm_map_origin_.translation()(1) = -parameters.size_y * parameters.resolution / 2.;
  ogm_map_origin_.linear() = Eigen::Rotation2Df::Identity().toRotationMatrix();
  ogm_res_ = parameters.ogm_resolution;
  ogm_size_x_ = parameters_.size_x * parameters_.resolution / ogm_res_;
  ogm_size_y_ = parameters_.size_y * parameters_.resolution / ogm_res_;
  simple_grid_.reset(new std::vector<int>(ogm_size_x_ * ogm_size_y_, 0));
}

void HierarchicalMap::addClusters(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters, const std::vector<std::vector<std::pair<double, double>>>& angle_dists, const std::vector<std::tuple<double, double, double>>& max_detections) {
  for (size_t i = 0; i < clusters.size(); ++i) {
    ndt_map_.insertCluster(clusters[i], angle_dists[i]);  // pass through to ndt map
  }
  is_empty = false;
}

std::pair<Eigen::Affine2f, std::shared_ptr<std::vector<int>>> HierarchicalMap::getSubmapCounts() {
  const Eigen::Affine2f ogm_origin = origin_in_global_frame_ * ogm_map_origin_;
  return std::make_pair(ogm_origin, simple_grid_); // get simple grids and transformation for global ogm
}

std::shared_ptr<nav_msgs::OccupancyGrid> HierarchicalMap::getOGM() { // if one wants only most recent submaps OGM
  std::shared_ptr<nav_msgs::OccupancyGrid> ogm = std::make_shared<nav_msgs::OccupancyGrid>();

  ogm->info.resolution = ogm_res_;
  ogm->info.width = parameters_.size_x * parameters_.resolution / ogm_res_;   // different resolution of ndt map and ogm map
  ogm->info.height = parameters_.size_y * parameters_.resolution / ogm_res_;
  Sophus::SE2f to_ogm_origin = Sophus::SE2f::fitToSE2((origin_in_global_frame_ * ogm_map_origin_).matrix());
  ogm->info.origin.position.x = to_ogm_origin.translation()(0);
  ogm->info.origin.position.y = to_ogm_origin.translation()(1);
  ogm->info.origin.position.z = 0.;

  const double theta = Sophus::SE2f::fitToSE2(origin_in_global_frame_.matrix()).log()(2);
  ogm->info.origin.orientation.w = std::cos(theta/2);
  ogm->info.origin.orientation.x = 0.;
  ogm->info.origin.orientation.y = 0.;
  ogm->info.origin.orientation.z = std::sin(theta/2);

  ogm->data.reserve(ogm->info.width * ogm->info.height);
  
  for (int i = 0; i < simple_grid_->size(); i++) {
    const double zeta = clamp(2 + 0.1 * static_cast<double>(simple_grid_->at(i)), 0., 4.);
    const double result = 100. * (-2 * std::pow(zeta/4., 3) + 3 * std::pow(zeta/4., 2)); 
    ogm->data.push_back(static_cast<int>(result));
  }

  return ogm;
}

void HierarchicalMap::mergeMapCell(const HierarchicalMap& m_hmap) {
  Map m_map = m_hmap.getMap();
  ndt_map_.mergeMapCell(m_map); 
  is_empty = false;
}

void HierarchicalMap::transformMap(const Eigen::Affine2f& trans) {
  ndt_map_.transformMap(trans);
}

void HierarchicalMap::transformMapWithPointCloud(const Eigen::Affine2f& trans) {
  ndt_map_.transformMapWithPointCloud(trans);
}

void HierarchicalMap::transformMapToOrigin(const Sophus::SE2d& new_origin) {
  Eigen::Affine2f new_origin_eigen(new_origin.cast<float>().matrix());
  origin_in_global_frame_ = new_origin_eigen; // set map origin to different origin 
}

void HierarchicalMap::raytraceLine(const Eigen::Affine2f& from, const std::tuple<double,double,double>& line, int max_length) {
  const double phi = std::get<0>(line);
  const double r = std::get<1>(line);
  const double i = std::get<2>(line);
  const Eigen::Vector2f begin_pos = from.translation();
  const Eigen::Vector2f end_pos = (Sophus::SE2f::fitToSE2(from.matrix()) * Sophus::SE2f(phi, {std::cos(phi) * r, std::sin(phi) * r})).translation(); // calculate end point
  int dx = (end_pos(0)-begin_pos(0)) / ogm_res_; // grid coordinates of ray
  int dy = (end_pos(1)-begin_pos(1)) / ogm_res_;
  unsigned int x0 = (begin_pos(0) / ogm_res_ + ogm_size_x_ / 2);  // start grid coordingates
  unsigned int y0 = (begin_pos(1) / ogm_res_ + ogm_size_y_ / 2);
  
  unsigned int abs_dx = std::abs(dx); // split in absolute values and signs
  unsigned int abs_dy = std::abs(dy); 
  int offset_dx = sgn(dx);
  int offset_dy = sgn(dy) * ogm_size_x_;
 
  unsigned int offset = y0 * ogm_size_x_ + x0;
  double dist = std::hypot(dx, dy);
  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, (max_length / ogm_res_) / dist);

  // if x is dominant
  if (abs_dx >= abs_dy)
  {
    int error_y = abs_dx / 2;
    bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
  } 
  else {    
    // otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
  }
  int updated_count = 0;
  for (int i = 0; i < simple_grid_->size(); i++) {
    if (simple_grid_->at(i) != 0) {
      updated_count++; // update ogm
    }
  }
}

void HierarchicalMap::bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length) {
  unsigned int end = std::min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i) {
    if (offset < simple_grid_->size()) {
      simple_grid_->at(offset)--; // free spcae update
    }
    else {
      std::cout << "Warning: Try to raytrace out of map bounds!\n";
      return;
    }
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da) {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  if (offset < simple_grid_->size()) {
    simple_grid_->at(offset)+=2; // occupied update
  }
  else {
    std::cout << "Warning: Try to raytrace out of map bounds!\n";
  }
}

}
}
}