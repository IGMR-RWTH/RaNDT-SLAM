#ifndef CERES_RESIDUALS
#define CERES_RESIDUALS

#include "ceres_loss_functions.h"
#include "state_manifold.h"

#include <ndt_representation/ndt_map.h>
#include <ndt_slam/trajectory_representation.h>
#include <Eigen/Core>
#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/ceres_manifold.hpp>
#include <ceres/loss_function.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/sized_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

/** @brief predict the movement given the state in vector representation
    *  @param old_... old state
    *  @param raw_dt time between states
    *  @param new_... predicted state
    */
template<typename T>
inline void predict(const Eigen::Matrix<T,2,1>& old_pos,
                    const T&                    old_rot,
                    const Eigen::Matrix<T,2,1>& old_lin_vel,
                    const T&                    old_rot_vel,
                    const Eigen::Matrix<T,2,1>& old_lin_acc,
                    const T&                    raw_dt,
                    Eigen::Matrix<T,2,1>&       new_pos,
                    T&                          new_rot,
                    Eigen::Matrix<T,2,1>&       new_lin_vel,
                    T&                          new_rot_vel,
                    Eigen::Matrix<T,2,1>&       new_lin_acc) 
{
  const T dt = std::max(raw_dt, static_cast<T>(0.2)); // sometimes, the stamps are identical due to driver failure. Preven this.
  new_pos = old_pos;
  new_rot = old_rot;
  T rot = NormalizeAngle(old_rot + 0.5 * dt * old_rot_vel);
  new_rot += dt * old_rot_vel;
  new_rot = NormalizeAngle(new_rot);
  T sy = ceres::sin(rot);
  T cy = ceres::cos(rot);
  T delta_x = old_lin_vel(0) * dt + 0.5 * old_lin_acc(0) * dt * dt;
  T delta_y = old_lin_vel(1) * dt + 0.5 * old_lin_acc(1) * dt * dt;
  new_lin_vel = old_lin_vel;
  new_rot_vel = old_rot_vel;
  new_lin_acc = old_lin_acc;
  new_pos(0) += cy * delta_x - sy * delta_y;
  new_pos(1) += sy * delta_x + cy * delta_y;
  new_lin_vel(0) += dt * old_lin_acc(0);
  new_lin_vel(1) += dt * old_lin_acc(1);
}

/** @brief predict the movement given the state in Lie group representation
    *  @param old_... old state
    *  @param raw_dt time between states
    *  @param new_... predicted state
    */
template<typename T>
inline void predictSE2(const Sophus::SE2<T>&       old_pose,
                       const Eigen::Matrix<T,2,1>& old_lin_vel,
                       const T&                    old_rot_vel,
                       const Eigen::Matrix<T,2,1>& old_lin_acc,
                       const T&                    raw_dt,
                       Sophus::SE2<T>&             new_pose,
                       Eigen::Matrix<T,2,1>&       new_lin_vel,
                       T&                          new_rot_vel,
                       Eigen::Matrix<T,2,1>&       new_lin_acc) 
{
  const T dt = std::max(raw_dt, static_cast<T>(0.2));
  const Eigen::Matrix<T,3,1> screw = (Eigen::Matrix<T,3,1>() << old_lin_vel(0) * dt + 0.5 * dt * old_lin_acc(0), 
                                                                old_lin_vel(1) * dt + 0.5 * dt * old_lin_acc(1), 
                                                                old_rot_vel * dt).finished();
  new_pose = old_pose * Sophus::SE2<T>::exp(screw);
  new_lin_vel = old_lin_vel;
  new_rot_vel = old_rot_vel;
  new_lin_acc = old_lin_acc;
  new_lin_vel(0) += dt * old_lin_acc(0);
  new_lin_vel(1) += dt * old_lin_acc(1);
}

/** @brief predict the movement given the state in vector representation
    *  @param old_... old state
    *  @param raw_dt time between states
    *  @param new_... predicted state
    *  @param jacobians jacobians of the motion model
    */
inline void predict(const Eigen::Matrix<double,2,1>& old_pos,
                    const double&                    old_rot,
                    const Eigen::Matrix<double,2,1>& old_lin_vel,
                    const double&                    old_rot_vel,
                    const Eigen::Matrix<double,2,1>& old_lin_acc,
                    const double&                    raw_dt,
                    Eigen::Matrix<double,2,1>&       new_pos,
                    double&                          new_rot,
                    Eigen::Matrix<double,2,1>&       new_lin_vel,
                    double&                          new_rot_vel,
                    Eigen::Matrix<double,2,1>&       new_lin_acc,
                    double**                         jacobians) 
{
  const double dt = std::max(raw_dt, 0.2);
  new_pos = old_pos;
  new_rot = old_rot;
  new_lin_vel = old_lin_vel;
  new_rot_vel = old_rot_vel;
  new_lin_acc = old_lin_acc;
  new_rot += dt * old_rot_vel;
  new_rot = NormalizeAngle(new_rot);

  const double rot = NormalizeAngle(old_rot + 0.5 * dt * old_rot_vel);
  const double sy = ceres::sin(rot);
  const double cy = ceres::cos(rot);
  const double half_dt2 = 0.5 * dt * dt;
  const double delta_x = old_lin_vel(0) * dt + old_lin_acc(0) * half_dt2;
  const double delta_y = old_lin_vel(1) * dt + old_lin_acc(1) * half_dt2;
  const double delta_x_rot = cy * delta_x - sy * delta_y;
  const double delta_y_rot = sy * delta_x + cy * delta_y;
  const double delta_x_rot_dt = delta_x_rot * 0.5 * dt;
  const double delta_y_rot_dt = delta_y_rot * 0.5 * dt;

  new_pos(0) += delta_x_rot;
  new_pos(1) += delta_y_rot;
  new_lin_vel += dt * old_lin_acc;
  if (jacobians) {
    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double,8,2, Eigen::RowMajor>> jacobian(jacobians[0]);
      jacobian << 1, 0,
                  0, 1,
                  0, 0,
                  0, 0,
                  0, 0,
                  0, 0,
                  0, 0,
                  0, 0;
    }

    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double,8,1>> jacobian(jacobians[1]);
      jacobian << -delta_y_rot, 
                   delta_x_rot, 
                             1, 
                             0, 
                             0, 
                             0, 
                             0, 
                             0;
    }

    if (jacobians[2])
    {
      const double cy_dt = cy * dt;
      const double sy_dt = sy * dt;

      Eigen::Map<Eigen::Matrix<double,8,2, Eigen::RowMajor>> jacobian(jacobians[2]);
      
      jacobian << cy_dt, -sy_dt,
                  sy_dt,  cy_dt,
                      0, 0,
                      1, 0,
                      0, 1,
                      0, 0,
                      0, 0,
                      0, 0;
      
    }

    if (jacobians[3])
    {
      Eigen::Map<Eigen::Matrix<double,8,1>> jacobian(jacobians[3]);
      jacobian << -delta_y_rot_dt, 
                   delta_x_rot_dt, 
                               dt, 
                                0, 
                                0, 
                                1, 
                                0, 
                                0;
    }

    // Jacobian wrt acc_linear1
    if (jacobians[4])
    {
      const double cy_half_dt2 = cy * half_dt2;
      const double sy_half_dt2 = sy * half_dt2;

      Eigen::Map<Eigen::Matrix<double, 8, 2, Eigen::RowMajor>> jacobian(jacobians[4]);

      jacobian << cy_half_dt2, -sy_half_dt2,
                  sy_half_dt2,  cy_half_dt2,
                            0, 0,
                            dt, 0,
                            0, dt,
                            0, 0,
                            1, 0,
                            0, 1;
      
    }
  }  
}


struct NDTFrameToMapFactorResidualAnalytic : public ceres::SizedCostFunction<1,2,1> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual between two pairs of normal distributions
    *  @param m_mean mean of the moving normal distribution
    *  @param m_cov covariance of the moving normal distribution
    *  @param f_mean mean of the fixed normal distribution
    *  @param f_cov covariance of the fixed normal distribution
    */
  NDTFrameToMapFactorResidualAnalytic(const Eigen::Matrix<double,2,1>& m_mean, 
                                      const Eigen::Matrix<double,2,2>& m_cov, 
                                      const Eigen::Matrix<double,2,1>& f_mean, 
                                      const Eigen::Matrix<double,2,2>& f_cov):
                                        m_mean_(m_mean), 
                                        m_cov_(m_cov), 
                                        f_mean_(f_mean),
                                        f_cov_(f_cov) {}
  /** @brief evaluate cost
    *  @param parameters parameters[0][0:1] translation, parameters[1][0] rotation
    */
  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    const Eigen::Matrix<double,2,1> translation(parameters[0][0], parameters[0][1]);
    const Eigen::Matrix<double,2,2> rotation = Eigen::Rotation2D<double>(NormalizeAngle(parameters[1][0])).toRotationMatrix();
    const Eigen::Matrix<double,2,2> rotation_derivative = rotation * (Eigen::Matrix<double,2,2>() << 0.0, -1.0, 1.0, 0.0).finished();
    const Eigen::Matrix<double,2,1> mean_diff = rotation * m_mean_ + translation - f_mean_;
    const Eigen::Matrix<double,1,2> mean_diff_transposed = mean_diff.transpose();
    const Eigen::Matrix<double,2,2> cov_inv = (rotation * m_cov_ * rotation.transpose() + f_cov_).inverse();
    residuals[0] = mean_diff_transposed * cov_inv * mean_diff; 

    if (jacobians) {
      if (jacobians[0]) {
        jacobians[0][0] = 2.0 * mean_diff_transposed * cov_inv * Eigen::Vector2d::UnitX();
        jacobians[0][1] = 2.0 * mean_diff_transposed * cov_inv * Eigen::Vector2d::UnitY();
      }
      if (jacobians[1]) {
        jacobians[1][0] = 2.0 * mean_diff_transposed * cov_inv * rotation_derivative * m_mean_; 
        jacobians[1][0] += 2.0 * mean_diff_transposed * cov_inv * rotation * m_cov_ * rotation_derivative * cov_inv * mean_diff;
      }
    }
    return true;
  }

private:
  const Eigen::Matrix<double,2,1> m_mean_, f_mean_;
  const Eigen::Matrix<double,2,2> m_cov_, f_cov_;
};

struct NDTFrameToMapIntensityFactorResidualAnalytic : public ceres::SizedCostFunction<1,2,1> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual between two pairs of normal distributions
    *  @param m_mean mean of the moving normal distribution
    *  @param m_cov covariance of the moving normal distribution
    *  @param f_mean mean of the fixed normal distribution
    *  @param f_cov covariance of the fixed normal distribution
    */
  NDTFrameToMapIntensityFactorResidualAnalytic(const Eigen::Matrix<double,3,1>& m_mean, 
                                               const Eigen::Matrix<double,3,3>& m_cov, 
                                               const Eigen::Matrix<double,3,1>& f_mean, 
                                               const Eigen::Matrix<double,3,3>& f_cov):
                                                 m_mean_(m_mean), 
                                                 m_cov_(m_cov), 
                                                 f_mean_(f_mean),
                                                 f_cov_(f_cov) {}
  /** @brief evaluate cost
    *  @param parameters parameters[0][0:1] translation, parameters[1][0] rotation
    */                                               
  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    const Eigen::Matrix<double,3,1> translation(parameters[0][0], parameters[0][1], 0);
    const Eigen::Matrix<double,3,3> rotation(Eigen::AngleAxisd(NormalizeAngle(parameters[1][0]), Eigen::Vector3d(0.0,0.0,1.0)));//Eigen::Rotation2D<double>(NormalizeAngle(parameters[0][2])).toRotationMatrix();
    const Eigen::Matrix<double,3,3> rotation_derivative = rotation * (Eigen::Matrix<double,3,3>() << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    const Eigen::Matrix<double,3,1> mean_diff = rotation * m_mean_ + translation - f_mean_;
    const Eigen::Matrix<double,1,3> mean_diff_transposed = mean_diff.transpose();
    const Eigen::Matrix<double,3,3> cov_inv = (rotation * m_cov_ * rotation.transpose() + f_cov_).inverse();
    residuals[0] = mean_diff_transposed * cov_inv * mean_diff; 

    if (jacobians) {
      if (jacobians[0]) {
        jacobians[0][0] = 2.0 * mean_diff_transposed * cov_inv * Eigen::Vector3d::UnitX();
        jacobians[0][1] = 2.0 * mean_diff_transposed * cov_inv * Eigen::Vector3d::UnitY();
      }
      if (jacobians[1]) {
        jacobians[1][0] = 2.0 * mean_diff_transposed * cov_inv * rotation_derivative * m_mean_; 
        jacobians[1][0] += 2.0 * mean_diff_transposed * cov_inv * rotation * m_cov_ * rotation_derivative * cov_inv * mean_diff;
      }
    }
    return true;
  }

private:
  const Eigen::Matrix<double,3,1> m_mean_, f_mean_;
  const Eigen::Matrix<double,3,3> m_cov_, f_cov_;
};

struct RotationalResidual {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual of relative IMU measurement
    *  @param rot measured rotation difference by IMU
    *  @param weight rotation error weight
    *  @param dt time between measurements
    *  @param bias_weight weight of the IMU bias
    */
  RotationalResidual(const double& rot, const double& weight, const double& dt, const double& bias_weight): rot_(rot), weight_(weight), dt_(dt), bias_weight_(bias_weight) {}

  /** @brief evaluate residual
    *  @param rot_old old rotation angle
    *  @param rot_new new rotation angle
    *  @param bias_old old IMU bias
    *  @param bias_new new IMU bias
    *  @param residual vector of residuals
    */
  template <typename T>
  bool operator()(const T* const rot_old, const T* const rot_new, const T* const bias_old, const T* const bias_new, T* residuals) const {
    residuals[0] = weight_ * (rot_ - NormalizeAngle(rot_new[0] - rot_old[0] + bias_new[0] * dt_));
    residuals[1] = bias_weight_ * (bias_new[0] - bias_old[0]);    
    return true;
  }

  private:
    const double rot_;
    const double weight_;
    const double dt_;
    const double bias_weight_;
};

struct RotationalResidualSE2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual of relative IMU measurement
    *  @param rot measured rotation difference by IMU
    *  @param weight rotation error weight
    *  @param dt time between measurements
    *  @param bias_weight weight of the IMU bias
    */
  RotationalResidualSE2(const double& rot, const double& weight, const double& dt, const double& bias_weight): rot_(rot), weight_(weight), dt_(dt), bias_weight_(bias_weight) {}
  
  /** @brief evaluate residual
    *  @param pose_old old pose 
    *  @param pose_new new pose
    *  @param bias_old old IMU bias
    *  @param bias_new new IMU bias
    *  @param residual vector of residuals
    */
  template <typename T>
  bool operator()(const T* const pose_old, const T* const pose_new, const T* const bias_old, const T* const bias_new, T* residuals) const {
    const Eigen::Matrix<T,3,1> screw = (Eigen::Matrix<T,3,1>() << T(0.), T(0.), bias_new[0] * dt_).finished();
    Sophus::SE2<T> M_0 = Eigen::Map<const Sophus::SE2<T>>(pose_old);
    Sophus::SE2<T> M_1 = Eigen::Map<const Sophus::SE2<T>>(pose_new) * Sophus::SE2<T>::exp(screw);
    residuals[0] = weight_ * (rot_ - (M_0.inverse() * M_1).log()(2));
    residuals[1] = bias_weight_ * (bias_new[0] - bias_old[0]);    
    return true;
  }

  private:
    const double rot_;
    const double weight_;
    const double dt_;
    const double bias_weight_;
};

struct RotationalResidualAnalytic: ceres::SizedCostFunction<2,1,1,1,1> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual of relative IMU measurement
    *  @param rot measured rotation difference by IMU
    *  @param weight rotation error weight
    *  @param dt time between measurements
    *  @param bias_weight weight of the IMU bias
    */
  RotationalResidualAnalytic(const double& rot, const double& weight, const double& dt, const double& bias_weight): rot_(rot), weight_(weight), dt_(dt), bias_weight_(bias_weight) {}

  /** @brief evaluate residual
    *  @param parameters[0][0] old rotation angle
    *  @param parameters[1][0] new rotation angle
    *  @param parameters[2][0] old IMU bias
    *  @param parameters[3][0] new IMU bias
    */
  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    const double bias_old = parameters[2][0];
    const double bias_new = parameters[3][0];
    residuals[0] = weight_ * (rot_ - NormalizeAngle(parameters[1][0] - parameters[0][0] + bias_new * dt_));
    residuals[1] = bias_weight_ * (bias_new - bias_old);  
    if (jacobians) {
      if (jacobians[0]) {
        jacobians[0][0] = weight_;
        jacobians[0][1] = 0.0;
      }
      if (jacobians[1]) {
        jacobians[1][0] = -weight_;
        jacobians[1][1] = 0.0;
      }
      if (jacobians[2]) {
        jacobians[2][0] = 0.0;
        jacobians[2][1] = -bias_weight_;
      }
      if (jacobians[3]) {
        jacobians[3][0] = -weight_ * dt_;
        jacobians[3][1] = bias_weight_;
      }
    }  
    return true;
  }

  private:
    const double rot_;
    const double weight_;
    const double dt_;
    const double bias_weight_;
};

struct NDTFrameToMapFactorResidual {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual between two pairs of normal distributions
    *  @param m_mean mean of the moving normal distribution
    *  @param m_cov covariance of the moving normal distribution
    *  @param f_mean mean of the fixed normal distribution
    *  @param f_cov covariance of the fixed normal distribution
    */
  NDTFrameToMapFactorResidual(const Eigen::Matrix<double,2,1>& m_mean, 
              const Eigen::Matrix<double,2,2>& m_cov, 
              const Eigen::Matrix<double,2,1>& f_mean, 
              const Eigen::Matrix<double,2,2>& f_cov): 
                m_mean_(m_mean), 
                m_cov_(m_cov), 
                f_mean_(f_mean),
                f_cov_(f_cov) {}
  /** @brief evaluate cost
    *  @param parameters parameters[0][0:1] translation, parameters[1][0] rotation
    */  
  template <typename T>
  bool operator()(const T* const pos, const T* const rot, T* residuals) const {
    const Eigen::Matrix<T,2,1> translation(pos[0], pos[1]);
    const Eigen::Matrix<T,2,2> rotation = Eigen::Rotation2D<T>(NormalizeAngle(rot[0])).toRotationMatrix();
    residuals[0] = ceres::sqrt(T((rotation * m_mean_ + translation - f_mean_).transpose() * (rotation * m_cov_ * rotation.transpose() + f_cov_).inverse() * (rotation * m_mean_ + translation - f_mean_))); 
    return true;
  }

  private:
    const Eigen::Matrix<double,2,1> m_mean_, f_mean_;
    const Eigen::Matrix<double,2,2> m_cov_, f_cov_;
};


struct NDTFrameToMapFactorResidualSE2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual between two pairs of normal distributions
    *  @param m_mean mean of the moving normal distribution
    *  @param m_cov covariance of the moving normal distribution
    *  @param f_mean mean of the fixed normal distribution
    *  @param f_cov covariance of the fixed normal distribution
    */
  NDTFrameToMapFactorResidualSE2(const Eigen::Matrix<double,2,1>& m_mean, 
              const Eigen::Matrix<double,2,2>& m_cov, 
              const Eigen::Matrix<double,2,1>& f_mean, 
              const Eigen::Matrix<double,2,2>& f_cov): 
                m_mean_(m_mean), 
                m_cov_(m_cov), 
                f_mean_(f_mean),
                f_cov_(f_cov) {}
  
  /** @brief evaluate cost
    *  @param parameters pose Sophus::SE2 pose
    */  
  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    Sophus::SE2<T> M = Eigen::Map<const Sophus::SE2<T>>(pose);
    residuals[0] = ceres::sqrt(T((M * m_mean_ - f_mean_).transpose() * (M.rotationMatrix() * m_cov_ * M.rotationMatrix().transpose() + f_cov_).inverse() * (M * m_mean_ - f_mean_))); 
    return true;
  }

  private:
    const Eigen::Matrix<double,2,1> m_mean_, f_mean_;
    const Eigen::Matrix<double,2,2> m_cov_, f_cov_;
};

struct NDTFrameToMapIntensityFactorResidual {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual between two pairs of normal distributions
    *  @param m_mean mean of the moving normal distribution
    *  @param m_cov covariance of the moving normal distribution
    *  @param f_mean mean of the fixed normal distribution
    *  @param f_cov covariance of the fixed normal distribution
    */
  NDTFrameToMapIntensityFactorResidual(const Eigen::Matrix<double,3,1>& m_mean, 
              const Eigen::Matrix<double,3,3>& m_cov, 
              const Eigen::Matrix<double,3,1>& f_mean, 
              const Eigen::Matrix<double,3,3>& f_cov): 
                m_mean_(m_mean), 
                m_cov_(m_cov), 
                f_mean_(f_mean),
                f_cov_(f_cov) {}
  
  /** @brief evaluate cost
    *  @param pos position
    *  @param rot rotation
    */ 
  template <typename T>
  bool operator()(const T* const pos, const T* const rot, T* residuals) const {
    const Eigen::Matrix<T,3,1> translation(pos[0], pos[1], static_cast<T>(0.0));
    const Eigen::Matrix<T,3,3> rotation(Eigen::AngleAxis<T>(NormalizeAngle(rot[0]), Eigen::Vector3d(0.0,0.0,1.0).cast<T>()));
    residuals[0] = ceres::sqrt(T((rotation * m_mean_ + translation - f_mean_).transpose() * (rotation * m_cov_ * rotation.transpose() + f_cov_).inverse() * (rotation * m_mean_ + translation - f_mean_))); 
    return true;
  }

  private:
    const Eigen::Matrix<double,3,1> m_mean_, f_mean_;
    const Eigen::Matrix<double,3,3> m_cov_, f_cov_;
};

struct NDTFrameToMapIntensityFactorResidualSE2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief construct residual between two pairs of normal distributions
    *  @param m_mean mean of the moving normal distribution
    *  @param m_cov covariance of the moving normal distribution
    *  @param f_mean mean of the fixed normal distribution
    *  @param f_cov covariance of the fixed normal distribution
    */
  NDTFrameToMapIntensityFactorResidualSE2(const Eigen::Matrix<double,3,1>& m_mean, 
              const Eigen::Matrix<double,3,3>& m_cov, 
              const Eigen::Matrix<double,3,1>& f_mean, 
              const Eigen::Matrix<double,3,3>& f_cov): 
                m_mean_(m_mean), 
                m_cov_(m_cov), 
                f_mean_(f_mean),
                f_cov_(f_cov) {}
  
  /** @brief evaluate cost
    *  @param parameters pose Sophus::SE2 pose
    */ 
  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    Sophus::SE2<T> M = Eigen::Map<const Sophus::SE2<T>>(pose);
    const Eigen::Matrix<T,3,1> translation(M.translation()(0), M.translation()(1), static_cast<T>(0.0));
    const Eigen::Matrix<T,3,3> rotation(Eigen::AngleAxis<T>(M.so2().log(), Eigen::Vector3d(0.0,0.0,1.0).cast<T>()));
    residuals[0] = ceres::sqrt(T((rotation * m_mean_ + translation - f_mean_).transpose() * (rotation * m_cov_ * rotation.transpose() + f_cov_).inverse() * (rotation * m_mean_ + translation - f_mean_))); 
    return true;
  }

  private:
    const Eigen::Matrix<double,3,1> m_mean_, f_mean_;
    const Eigen::Matrix<double,3,3> m_cov_, f_cov_;
};

struct MotionModelFactor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief motion residual
    *  @param dt time between two states
    *  @param sqrtI square-root information matrix of the motion model
    */
  MotionModelFactor(const double& dt, const Eigen::Matrix<double,8,8>& sqrtI): dt_(dt), sqrtI_(sqrtI) {}
  template <typename T>
  bool operator()(const T* const old_pos,
                  const T* const old_rot,
                  const T* const old_lin_vel,
                  const T* const old_rot_vel,
                  const T* const old_lin_acc,
                  const T* const new_pos,
                  const T* const new_rot,
                  const T* const new_lin_vel,
                  const T* const new_rot_vel,
                  const T* const new_lin_acc,
                  T*             residuals) const {
                  
    const Eigen::Matrix<T,2,1> pos_0(old_pos[0], old_pos[1]);
    const T                    rot_0 = old_rot[0];
    const Eigen::Matrix<T,2,1> lin_vel_0(old_lin_vel[0], old_lin_vel[1]);
    const T                    rot_vel_0 = old_rot_vel[0];
    const Eigen::Matrix<T,2,1> lin_acc_0(old_lin_acc[0], old_lin_acc[1]);
    
    Eigen::Matrix<T,2,1> pos_1(new_pos[0], new_pos[1]);
    T                    rot_1 = new_rot[0];
    Eigen::Matrix<T,2,1> lin_vel_1(new_lin_vel[0], new_lin_vel[1]);
    T                    rot_vel_1 = new_rot_vel[0];
    Eigen::Matrix<T,2,1> lin_acc_1(new_lin_acc[0], new_lin_acc[1]);

    Eigen::Matrix<T,2,1> pos_pred;
    T                    rot_pred;
    Eigen::Matrix<T,2,1> lin_vel_pred;
    T                    rot_vel_pred;
    Eigen::Matrix<T,2,1> lin_acc_pred;

    predict(pos_0, 
            rot_0,
            lin_vel_0,
            rot_vel_0,
            lin_acc_0,
            T(dt_),
            pos_pred,
            rot_pred,
            lin_vel_pred,
            rot_vel_pred,
            lin_acc_pred);

    Eigen::Map<Eigen::Matrix<T, 8, 1>> residuals_map(residuals);
    residuals_map(0) = pos_1(0) - pos_pred(0);
    residuals_map(1) = pos_1(1) - pos_pred(1);
    residuals_map(2) = NormalizeAngle(rot_1 - rot_pred);
    residuals_map(3) = lin_vel_1(0) - lin_vel_pred(0);
    residuals_map(4) = lin_vel_1(1) - lin_vel_pred(1);
    residuals_map(5) = rot_vel_1    - rot_vel_pred;
    residuals_map(6) = lin_acc_1(0) - lin_acc_pred(0);
    residuals_map(7) = lin_acc_1(1) - lin_acc_pred(1);
    residuals_map.applyOnTheLeft(sqrtI_);
    return true;
  }
private:
  double dt_;
  Eigen::Matrix<double,8,8> sqrtI_;
};

struct MotionModelFactorSE2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief motion residual
    *  @param dt time between two states
    *  @param sqrtI square-root information matrix of the motion model
    */
  MotionModelFactorSE2(const double& dt, const Eigen::Matrix<double,8,8>& sqrtI): dt_(dt), sqrtI_(sqrtI) {}
  template <typename T>
  bool operator()(const T* const old_pose,
                  const T* const old_lin_vel,
                  const T* const old_rot_vel,
                  const T* const old_lin_acc,
                  const T* const new_pose,
                  const T* const new_lin_vel,
                  const T* const new_rot_vel,
                  const T* const new_lin_acc,
                  T*             residuals) const {
                  
    const Sophus::SE2<T>       pose_0 = Eigen::Map<const Sophus::SE2<T>>(old_pose);
    const Eigen::Matrix<T,2,1> lin_vel_0(old_lin_vel[0], old_lin_vel[1]);
    const T                    rot_vel_0 = old_rot_vel[0];
    const Eigen::Matrix<T,2,1> lin_acc_0(old_lin_acc[0], old_lin_acc[1]);
        
    Sophus::SE2<T>       pose_1 = Eigen::Map<const Sophus::SE2<T>>(new_pose);
    Eigen::Matrix<T,2,1> lin_vel_1(new_lin_vel[0], new_lin_vel[1]);
    T                    rot_vel_1 = new_rot_vel[0];
    Eigen::Matrix<T,2,1> lin_acc_1(new_lin_acc[0], new_lin_acc[1]);

    Sophus::SE2<T>       pose_pred;
    Eigen::Matrix<T,2,1> lin_vel_pred;
    T                    rot_vel_pred;
    Eigen::Matrix<T,2,1> lin_acc_pred;

    predictSE2(pose_0,
               lin_vel_0,
               rot_vel_0,
               lin_acc_0,
               T(dt_),
               pose_pred,
               lin_vel_pred,
               rot_vel_pred,
               lin_acc_pred);

    Eigen::Map<Eigen::Matrix<T, 8, 1>> residuals_map(residuals);
    residuals_map(0) = (pose_pred.inverse() * pose_1).log()(0);
    residuals_map(1) = (pose_pred.inverse() * pose_1).log()(1);
    residuals_map(2) = (pose_pred.inverse() * pose_1).log()(2);
    residuals_map(3) = lin_vel_1(0) - lin_vel_pred(0);
    residuals_map(4) = lin_vel_1(1) - lin_vel_pred(1);
    residuals_map(5) = rot_vel_1    - rot_vel_pred;
    residuals_map(6) = lin_acc_1(0) - lin_acc_pred(0);
    residuals_map(7) = lin_acc_1(1) - lin_acc_pred(1);
    residuals_map.applyOnTheLeft(sqrtI_);
    return true;
  }
private:
  double dt_;
  Eigen::Matrix<double,8,8> sqrtI_;
};

struct PriorFactor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief prior residual
    *  @param old_... state to which the prior applies 
    *  @param sqrtI square-root information matrix of the state
    */
  PriorFactor(const Eigen::Vector2d& old_pos, 
              const double& old_rot, 
              const Eigen::Vector2d& old_lin_vel,
              const double& old_rot_vel,
              const Eigen::Vector2d& old_lin_acc,
              const double& old_imu_bias,
              const Eigen::Matrix<double,9,9>& sqrtI): 
                old_pos_(old_pos),
                old_rot_(old_rot),
                old_lin_vel_(old_lin_vel),
                old_rot_vel_(old_rot_vel),
                old_lin_acc_(old_lin_acc),
                old_imu_bias_(old_imu_bias),
                sqrtI_(sqrtI) {}
  template <typename T>
  bool operator()(const T* const new_pos,
                  const T* const new_rot,
                  const T* const new_lin_vel,
                  const T* const new_rot_vel,
                  const T* const new_lin_acc,
                  const T* const new_imu_bias,
                  T*             residuals) const {
                      
    Eigen::Matrix<T,2,1> pos_1(new_pos[0], new_pos[1]);
    T                    rot_1 = new_rot[0];
    Eigen::Matrix<T,2,1> lin_vel_1(new_lin_vel[0], new_lin_vel[1]);
    T                    rot_vel_1 = new_rot_vel[0];
    Eigen::Matrix<T,2,1> lin_acc_1(new_lin_acc[0], new_lin_acc[1]);
    T                    imu_bias_1 = new_imu_bias[0];

    Eigen::Map<Eigen::Matrix<T, 9, 1>> residuals_map(residuals);
    residuals_map(0) = pos_1(0) - old_pos_(0);
    residuals_map(1) = pos_1(1) - old_pos_(1);
    residuals_map(2) = NormalizeAngle(rot_1 - old_rot_);
    residuals_map(3) = lin_vel_1(0) - old_lin_vel_(0);
    residuals_map(4) = lin_vel_1(1) - old_lin_vel_(1);
    residuals_map(5) = rot_vel_1    - old_rot_vel_;
    residuals_map(6) = lin_acc_1(0) - old_lin_acc_(0);
    residuals_map(7) = lin_acc_1(1) - old_lin_acc_(1);
    residuals_map(8) = imu_bias_1 - old_imu_bias_;
    residuals_map.applyOnTheLeft(sqrtI_);
    return true;
  }
private:
  Eigen::Vector2d old_pos_; 
  double old_rot_; 
  Eigen::Vector2d old_lin_vel_;
  double old_rot_vel_;
  Eigen::Vector2d old_lin_acc_;
  double old_imu_bias_;
  Eigen::Matrix<double,9,9> sqrtI_;
};

struct PriorFactorSE2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief prior residual
    *  @param old_... state to which the prior applies 
    *  @param sqrtI square-root information matrix of the state
    */
  PriorFactorSE2(const Sophus::SE2d& old_pose, 
                 const Eigen::Vector2d& old_lin_vel,
                 const double& old_rot_vel,
                 const Eigen::Vector2d& old_lin_acc,
                 const double& old_imu_bias,
                 const Eigen::Matrix<double,9,9>& sqrtI): 
                   old_pose_(old_pose), 
                   old_lin_vel_(old_lin_vel),
                   old_rot_vel_(old_rot_vel),
                   old_lin_acc_(old_lin_acc),
                   old_imu_bias_(old_imu_bias),
                   sqrtI_(sqrtI) {}
  template <typename T>
  bool operator()(const T* const new_pose,
                  const T* const new_lin_vel,
                  const T* const new_rot_vel,
                  const T* const new_lin_acc,
                  const T* const new_imu_bias,
                  T*             residuals) const {
        
    Sophus::SE2<T>       pose_1 = Eigen::Map<const Sophus::SE2<T>>(new_pose);
    Eigen::Matrix<T,2,1> lin_vel_1(new_lin_vel[0], new_lin_vel[1]);
    T                    rot_vel_1 = new_rot_vel[0];
    Eigen::Matrix<T,2,1> lin_acc_1(new_lin_acc[0], new_lin_acc[1]);
    T                    imu_bias_1 = new_imu_bias[0];

    Eigen::Map<Eigen::Matrix<T, 9, 1>> residuals_map(residuals);
    residuals_map(0) = (old_pose_.inverse() * pose_1).log()(0);
    residuals_map(1) = (old_pose_.inverse() * pose_1).log()(1);
    residuals_map(2) = (old_pose_.inverse() * pose_1).log()(2);
    residuals_map(3) = lin_vel_1(0) - old_lin_vel_(0);
    residuals_map(4) = lin_vel_1(1) - old_lin_vel_(1);
    residuals_map(5) = rot_vel_1    - old_rot_vel_;
    residuals_map(6) = lin_acc_1(0) - old_lin_acc_(0);
    residuals_map(7) = lin_acc_1(1) - old_lin_acc_(1);
    residuals_map(8) = imu_bias_1 - old_imu_bias_;
    residuals_map.applyOnTheLeft(sqrtI_);
    return true;
  }
private:
  Sophus::SE2d old_pose_;
  Eigen::Vector2d old_lin_vel_;
  double old_rot_vel_;
  Eigen::Vector2d old_lin_acc_;
  double old_imu_bias_;
  Eigen::Matrix<double,9,9> sqrtI_;
};

struct MotionModelFactorAnalytic: public ceres::SizedCostFunction<8,2,1,2,1,2,2,1,2,1,2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief prior residual
    *  @param dt time between two states
    *  @param sqrtI square-root information matrix of the motion model
    */
  MotionModelFactorAnalytic(const double& dt, const Eigen::Matrix<double,8,8>& sqrtI): dt_(dt), sqrtI_(sqrtI) {}
  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    const Eigen::Map<const Eigen::Matrix<double,2,1>> pos_0(parameters[0]);
    const Eigen::Map<const Eigen::Matrix<double,2,1>> lin_vel_0(parameters[2]);
    const Eigen::Map<const Eigen::Matrix<double,2,1>> lin_acc_0(parameters[4]);

    const Eigen::Map<const Eigen::Matrix<double,2,1>> pos_1(parameters[5]);
    const Eigen::Map<const Eigen::Matrix<double,2,1>> lin_vel_1(parameters[7]);
    const Eigen::Map<const Eigen::Matrix<double,2,1>> lin_acc_1(parameters[9]);
    

    Eigen::Matrix<double,2,1> pos_pred;
    double                    rot_pred;
    Eigen::Matrix<double,2,1> lin_vel_pred;
    double                    rot_vel_pred;
    Eigen::Matrix<double,2,1> lin_acc_pred;
    predict(pos_0, 
            parameters[1][0],
            lin_vel_0,
            parameters[3][0],
            lin_acc_0,
            dt_,
            pos_pred,
            rot_pred,
            lin_vel_pred,
            rot_vel_pred,
            lin_acc_pred,
            jacobians);

    Eigen::Map<Eigen::Matrix<double,8,1>> residuals_map(residuals);
    residuals_map(0) = pos_1(0)         - pos_pred(0);
    residuals_map(1) = pos_1(1)         - pos_pred(1);
    residuals_map(2) = parameters[6][0] - rot_pred;
    residuals_map(3) = lin_vel_1(0)     - lin_vel_pred(0);
    residuals_map(4) = lin_vel_1(1)     - lin_vel_pred(1);
    residuals_map(5) = parameters[8][0] - rot_vel_pred;
    residuals_map(6) = lin_acc_1(0)     - lin_acc_pred(0);
    residuals_map(7) = lin_acc_1(1)     - lin_acc_pred(1);
    residuals_map(2) = NormalizeAngle(residuals_map(2));
    residuals_map.applyOnTheLeft(sqrtI_);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double,8,2, Eigen::RowMajor>> jacobian(jacobians[0]);
        jacobian.applyOnTheLeft(-sqrtI_);
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double,8,1>> jacobian(jacobians[1]);
        jacobian.applyOnTheLeft(-sqrtI_);
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double,8,2, Eigen::RowMajor>> jacobian(jacobians[2]);
        jacobian.applyOnTheLeft(-sqrtI_);
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double,8,1>> jacobian(jacobians[3]);
        jacobian.applyOnTheLeft(-sqrtI_);
      }
      if (jacobians[4]) {
        Eigen::Map<Eigen::Matrix<double,8,2, Eigen::RowMajor>> jacobian(jacobians[4]);
        jacobian.applyOnTheLeft(-sqrtI_);
      }
      if (jacobians[5]) {
        Eigen::Map<Eigen::Matrix<double,8,2, Eigen::RowMajor>> jacobian(jacobians[5]);
        jacobian = sqrtI_.block<8, 2>(0, 0);
      }
      if (jacobians[6]) {
        Eigen::Map<Eigen::Matrix<double,8,1>> jacobian(jacobians[6]);
        jacobian = sqrtI_.col(2);
      }
      if (jacobians[7]) {
        Eigen::Map<Eigen::Matrix<double,8,2, Eigen::RowMajor>> jacobian(jacobians[7]);
        jacobian = sqrtI_.block<8, 2>(0, 3);
      }
      if (jacobians[8]) {
        Eigen::Map<Eigen::Matrix<double,8,1>> jacobian(jacobians[8]);
        jacobian = sqrtI_.col(5);
      }
      if (jacobians[9]) {
        Eigen::Map<Eigen::Matrix<double,8,2, Eigen::RowMajor>> jacobian(jacobians[9]);
        jacobian = sqrtI_.block<8, 2>(0, 6);
      }
    }
    return true;
  }
private:
  double dt_;
  Eigen::Matrix<double,8,8> sqrtI_;
};

#endif