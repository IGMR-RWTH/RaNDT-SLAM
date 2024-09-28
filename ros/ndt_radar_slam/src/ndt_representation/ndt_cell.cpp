#include <ndt_representation/ndt_cell.h>

namespace rc {
namespace navigation {
namespace ndt {

void Cell::initialize(const int& min_points_per_cell, const NDTCellParameters& params){
  min_points_per_cell_ = min_points_per_cell;
  max_intensity_ = 0;
  params_ = params;
}

double Cell::evaluateAtPoint(const Eigen::Vector2f& pt) const { // evaluate for ndt
  const Eigen::Vector2f mean_diff = pt - new_mean_.block<2,1>(0,0);
  return 1 / (2 * M_PI * std::sqrt(new_cov_.block<2,1>(0,0).determinant())) * std::exp(-0.5 * mean_diff.transpose() * new_cov_.block<2,2>(0,0).inverse() * mean_diff);
}


void Cell::addPoint(const pcl::PointXYZI& point, const std::pair<double, double>& angle_dist){
  n_points_to_add_++;
  points_to_add_.push_back(point);
  polar_points_to_add_.push_back(angle_dist); 
}

bool Cell::addPointCloud(const pcl::PointCloud<pcl::PointXYZI>& point_cloud, const std::vector<std::pair<double, double>>& angle_dists){
  if (n_points_+ point_cloud.size()> min_points_per_cell_) {
    n_points_to_add_+= point_cloud.size();
    points_to_add_ += point_cloud;
    polar_points_to_add_.insert(polar_points_to_add_.end(), angle_dists.begin(), angle_dists.end());
    updateCell();  // use recursive update equation
    return true;
  }
  return false;
}

void Cell::updateCell(void) {
  if(n_points_ + n_points_to_add_ > min_points_per_cell_ && n_points_to_add_ > 0) {
    Eigen::Vector3f mean_to_add;
    Eigen::Matrix3f cov_to_add;
    Eigen::Matrix3f point_cov_to_add;
    mean_to_add.setZero(); 
    // first, accumulate all new measurements to calculate additional distributions parameters
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator point = points_to_add_.begin(); point != points_to_add_.end(); point++) {
      mean_to_add += Eigen::Vector3f{point->x, point->y, point->intensity}; 
      max_intensity_ = std::max(max_intensity_, static_cast<double>(point->intensity));
    }
    mean_to_add /= static_cast<double>(n_points_to_add_);
    cov_to_add.setZero();
    point_cov_to_add.setZero();
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator point = points_to_add_.begin(); point != points_to_add_.end(); point++) {
      Eigen::Vector3f new_pt;
      new_pt[0] = point->x - mean_to_add[0];
      new_pt[1] = point->y - mean_to_add[1];
      new_pt[2] = point->intensity - mean_to_add[2];
      cov_to_add(0,0) += (new_pt[0] * new_pt[0]);
      cov_to_add(1,1) += (new_pt[1] * new_pt[1]);
      cov_to_add(2,2) += (new_pt[2] * new_pt[2]);
      cov_to_add(0,1) += (new_pt[0] * new_pt[1]);
      cov_to_add(0,2) += (new_pt[0] * new_pt[2]);
      cov_to_add(1,2) += (new_pt[1] * new_pt[2]);
    }
    cov_to_add(1,0) = cov_to_add(0,1); // exploit symmetry
    cov_to_add(2,0) = cov_to_add(0,2); // exploit symmetry
    cov_to_add(2,1) = cov_to_add(1,2); // exploit symmetry
    cov_to_add /= static_cast<double>(n_points_to_add_);

    // for pndt calculate point covariances
    if (params_.use_pndt) {
      for (size_t i = 0; i < polar_points_to_add_.size(); i++) {
        const float a = polar_points_to_add_[i].first;
        const float r = polar_points_to_add_[i].second;
        Eigen::Matrix3f jacobian, probabilistic_cov, sensor_cov;
        sensor_cov = params_.beam_cov;
        jacobian << -r * std::sin(a), std::cos(a), 0,
                    r * std::cos(a), std::sin(a), 0,
                    0              , 0          , 1;
        probabilistic_cov = jacobian * sensor_cov * jacobian.transpose();
        point_cov_to_add = point_cov_to_add + probabilistic_cov;
      }
      point_cov_to_add /= static_cast<double>(n_points_to_add_);
      cov_to_add = cov_to_add + point_cov_to_add;
    }
    // recursive update formulation
    if (n_points_ > 0) {
      new_cov_ = (n_points_-1) * new_cov_ + (n_points_to_add_ - 1 ) * cov_to_add + (((n_points_ * n_points_to_add_)/(n_points_ + n_points_to_add_))*((new_mean_ - mean_to_add)*(new_mean_-mean_to_add).transpose()));
      new_mean_ = ((new_mean_ * n_points_) + (mean_to_add * n_points_to_add_)) / (n_points_ + n_points_to_add_);
      n_points_ += n_points_to_add_;
      new_cov_ /= (n_points_-1);
    }
    else {
      new_cov_ = cov_to_add;
      new_mean_ = mean_to_add;
      n_points_ = n_points_to_add_; 
    }
    points_ += points_to_add_;
    points_to_add_.clear();
    polar_points_.insert(polar_points_.end(), polar_points_to_add_.begin(), polar_points_to_add_.end());
    polar_points_to_add_.clear();
    n_points_to_add_ = 0;
    
    // regularization
    if (!params_.use_pndt) { // regularize ndt shape
      Eigen::Matrix2f pos_cov = new_cov_.block<2,2>(0,0);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> s(pos_cov);
      Eigen::Vector2f eig_values {s.eigenvalues().real()};
      Eigen::Matrix2f eig_vectors {s.eigenvectors().real()};
      eig_values(0) = std::max(eig_values(0), (float)0.001 * eig_values(1));
      Eigen::Matrix2f regularized_eigs;
      regularized_eigs << eig_values(0), 0, 0, eig_values(1);
      new_cov_.block<2,2>(0,0) = eig_vectors * regularized_eigs * eig_vectors.inverse(); 
      new_cov_(2,2) += 0.000001;
    }
  }
}

// transform cell parameters
void Cell::transformCell(const Eigen::Affine2f& trans) {
  Eigen::Affine3f trans_3d = Eigen::Affine3f::Identity();
  trans_3d.matrix().block<2,2>(0,0) = trans.linear();
  trans_3d.matrix().block<2,1>(0,3) = trans.translation();
  new_mean_ = trans_3d * new_mean_;
  new_cov_ = trans_3d.rotation() * new_cov_ * trans_3d.rotation().transpose();
}

// transform cell parameters and generating points
void Cell::transformCellWithPointCloud(const Eigen::Affine2f& trans) {
  Eigen::Affine3f trans_3d = Eigen::Affine3f::Identity();
  trans_3d.matrix().block<2,2>(0,0) = trans.linear();
  trans_3d.matrix().block<2,1>(0,3) = trans.translation();
  new_mean_ = trans_3d * new_mean_;
  new_cov_ = trans_3d.rotation() * new_cov_ * trans_3d.rotation().transpose();
  pcl::PointCloud<pcl::PointXYZI> transformed_points_;
  transformed_points_.reserve(points_.size());
  pcl::transformPointCloud(points_, transformed_points_,  trans_3d);
  points_ = transformed_points_;
}

void Cell::getMean(Eigen::Vector2f& mean) const{
  mean = new_mean_.block<2,1>(0,0);
}

void Cell::getCov(Eigen::Matrix2f& cov) const{
  cov = new_cov_.block<2,2>(0,0);
} 

void Cell::getIntensityMean(Eigen::Vector3f& mean) const{
  mean = new_mean_;
}

void Cell::getIntensityCov(Eigen::Matrix3f& cov) const{
  cov = new_cov_;
} 

void Cell::getMeanAndCov(Eigen::Vector2f& mean, Eigen::Matrix2f& cov) const {
  mean = new_mean_.block<2,1>(0,0);
  cov = new_cov_.block<2,2>(0,0);
}

void Cell::getIntensityMeanAndCov(Eigen::Vector3f& mean, Eigen::Matrix3f& cov) const {
  mean = new_mean_;
  cov = new_cov_;
}

// l2 distance between distributions
double Cell::mahalanobisSquared(const Cell& subtrahend) const {
  Eigen::Matrix2f summed_cov = subtrahend.getCov() + new_cov_.block<2,2>(0,0);
  Eigen::Vector2f mu = subtrahend.getMean() - new_mean_.block<2,1>(0,0);
  return mu.transpose() * summed_cov.inverse() * mu; 
}

// l2 distance between distributions 
double Cell::mahalanobisSquaredIntensity(const Cell& subtrahend) const {
  Eigen::Matrix3f summed_cov = subtrahend.getIntensityCov() + new_cov_;
  Eigen::Vector3f mu = subtrahend.getIntensityMean() - new_mean_;
  return mu.transpose() * summed_cov.inverse() * mu; 
}

size_t Cell::getNumCells() const {
  return n_points_;
}

void Cell::clearCell(void) {
  n_points_ = 0;
  points_.clear();
  new_mean_.setZero();
  new_cov_.setZero();
}

}
}
}

