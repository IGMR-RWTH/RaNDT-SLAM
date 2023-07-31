#ifndef TRAJECTORY_REPRESENTATION_H
#define TRAJECTORY_REPRESENTATION_H

#include <Eigen/Core>
#include <sophus/geometry.hpp>

namespace rc {
namespace navigation {
namespace ndt {

// State with velocity and acceleration for local motion estimation
struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Sophus::SE2d pose;
  Eigen::Vector2d pos;
  double rot;
  Eigen::Vector2d lin_vel;
  double rot_vel;
  Eigen::Vector2d lin_acc;
  double imu_bias;
  double stamp;
};

// Pose definition for usage in pose graph optimization with covariances
struct Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Sophus::SE2d pose;
  Eigen::Vector2d pos;
  double rot;
  Eigen::Matrix3d cov;
  Eigen::Matrix2d cov_pos_pos;
  Eigen::Vector2d cov_pos_rot;
  double cov_rot_rot;

  double traversed_dist;

  template <typename T>
  inline Eigen::Transform<T,2,Eigen::Affine> getEigenAffineTransform() const {
    Eigen::Transform<T,2,Eigen::Affine> result(pose.cast<T>().matrix());
    return result; 
  }
};

// constraint between two poses for pose graph
struct Constraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int id_begin;
  int id_end;
  Sophus::SE2d trans;

  Eigen::Matrix3d sqrt_information;
};

}
}
}

#endif