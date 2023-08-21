#include "global_fuser/global_fuser.h"

namespace rc {
namespace navigation {
namespace ndt {

void GlobalFuser::initialize(GlobalFuserParameters parameters) {
  parameters_ = parameters;
  n_optimized_constraints_ = 0;
  n_optimized_poses_ = 0;
}

void GlobalFuser::optimizePoseGraph(std::map<int, Pose>& poses_ref, const std::vector<Constraint>& edges, std::mutex& poses_mutex, int max_update_index) {
  std::unique_lock<std::mutex> lock(poses_mutex);
  std::map<int, Pose> poses(poses_ref);
  lock.unlock();  // quickly unlock poses after copying
  ceres::LossFunction* loss;
  if (parameters_.use_robust_loss) {
    loss = new ceres::HuberLoss(parameters_.loss_function_scale);
  }
  else {
    loss = nullptr;
  }
  ceres::Problem problem;

  const int loop_closures = edges.size() + 1 - poses.size(); // number of loop closure constraints
  std::cout << "detected " << loop_closures << " loop closure constraints so far" << std::endl;

  // construct optimization
  for (const auto& constraint : edges) {
    if (constraint.id_begin+1 == constraint.id_end || constraint.id_end <= max_update_index) {
      auto pose_begin_iter = poses.find(constraint.id_begin);
      auto pose_end_iter = poses.find(constraint.id_end);
      problem.AddParameterBlock(pose_begin_iter->second.pos.data(), 2);
      problem.AddParameterBlock(&pose_begin_iter->second.rot, 1);
      problem.AddParameterBlock(pose_end_iter->second.pos.data(), 2);
      problem.AddParameterBlock(&pose_end_iter->second.rot, 1);
      ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(constraint.trans.translation(), constraint.trans.log()(2), constraint.sqrt_information);
      problem.AddResidualBlock(cost_function,
                              loss,
                              pose_begin_iter->second.pos.data(),
                              &pose_begin_iter->second.rot,
                              pose_end_iter->second.pos.data(),
                              &pose_end_iter->second.rot);
      
    }
  } 
  problem.SetParameterBlockConstant(poses.begin()->second.pos.data());
  problem.SetParameterBlockConstant(&poses.begin()->second.rot);

  // solve optimization
  ceres::Solver::Options options;
  options.max_num_iterations = 2000;
  options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.num_threads = std::thread::hardware_concurrency();
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.BriefReport() << '\n';

  // get covariance
  ceres::Covariance::Options cov_options;
  cov_options.num_threads = options.num_threads;
  problem.SetParameterBlockVariable(poses.begin()->second.pos.data());
  problem.SetParameterBlockVariable(&poses.begin()->second.rot);
  problem.SetParameterBlockConstant(std::prev(poses.end())->second.pos.data());
  problem.SetParameterBlockConstant(&std::prev(poses.end())->second.rot);

  ceres::Covariance covariance(cov_options);

  std::vector<std::pair<const double*, const double*>> covariance_blocks;

  for (int i = 0; i < poses.size(); i++) {
    covariance_blocks.push_back(std::make_pair(poses[i].pos.data(), poses[i].pos.data()));
    covariance_blocks.push_back(std::make_pair(poses[i].pos.data(), &poses[i].rot));
    covariance_blocks.push_back(std::make_pair(&poses[i].rot, &poses[i].rot));

  }
  covariance.Compute(covariance_blocks, &problem);
  for (int i = 0; i < poses.size(); i++) {
    covariance.GetCovarianceBlock(poses[i].pos.data(), poses[i].pos.data(), poses[i].cov_pos_pos.data());
    covariance.GetCovarianceBlock(poses[i].pos.data(), &poses[i].rot, poses[i].cov_pos_rot.data());
    covariance.GetCovarianceBlock(&poses[i].rot, &poses[i].rot, &poses[i].cov_rot_rot);
  }

  for (auto & pose: poses) {
    pose.second.pose = Sophus::SE2d(pose.second.rot, pose.second.pos);
    pose.second.cov.block<2,2>(0,0) = pose.second.cov_pos_pos;
    pose.second.cov.block<2,1>(0,2) = pose.second.cov_pos_rot;
    pose.second.cov.block<1,2>(2,0) = pose.second.cov_pos_rot.transpose();
    pose.second.cov(2,2) = pose.second.cov_rot_rot;
  }
  
  n_optimized_constraints_ = edges.size();
  n_optimized_poses_ = poses.size();
  
  // copy back to poses
  lock.lock(); 
  for (int i = 0; i < poses.size(); i++) {
    poses_ref.at(i) = poses.at(i);
  }
}
}
}
}