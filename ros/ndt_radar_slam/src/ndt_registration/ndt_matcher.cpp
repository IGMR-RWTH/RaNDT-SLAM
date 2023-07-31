#include "ndt_registration/ndt_matcher.h"

namespace rc {
namespace navigation {
namespace ndt {

void Matcher::initialize(NDTMatcherParameters parameters) {
  parameters_ = parameters;
  X_next_.pos = Eigen::Vector2d::Zero();
  X_next_.rot = 0;
  X_next_.pose = Sophus::SE2d(Eigen::Affine2d::Identity().matrix());
  X_next_.lin_vel = Eigen::Vector2d::Zero();
  X_next_.rot_vel = 0;
  X_next_.lin_acc = Eigen::Vector2d::Zero();
  X_next_.imu_bias = 0;
}

void Matcher::resetMatcher() {
  imu_constraints_.clear();
}

void Matcher::predictTransform(const double& initial_angle_guess, const double& stamp, std::vector<State>& trajectory) {
  if (!trajectory.empty()) {
    State last_state = trajectory.end()[-1];
    X_next_.stamp = stamp;
    last_state.lin_acc = Eigen::Vector2d::Zero();
    if (parameters_.use_analytic_expressions_for_optimization ||
        !parameters_.optimize_on_manifold) {
      predict(last_state.pos,
              last_state.rot,
              last_state.lin_vel,
              last_state.rot_vel,
              last_state.lin_acc,
              stamp - trajectory.end()[-1].stamp, 
              X_next_.pos,
              X_next_.rot,
              X_next_.lin_vel,
              X_next_.rot_vel,
              X_next_.lin_acc,
              nullptr);
      X_next_.pose = Sophus::SE2d(X_next_.rot, X_next_.pos); // both representations should hold the same information
    }
    else {
      predictSE2(last_state.pose,
                 last_state.lin_vel,
                 last_state.rot_vel,
                 last_state.lin_acc,
                 stamp - trajectory.end()[-1].stamp,
                 X_next_.pose,
                 X_next_.lin_vel,
                 X_next_.rot_vel,
                 X_next_.lin_acc);
      X_next_.pos = X_next_.pose.translation();  // both representations should hold the same information
      X_next_.rot = X_next_.pose.log()(2);
    }
    trajectory.push_back(X_next_);
    imu_constraints_.push_back(initial_angle_guess);
  }
}

ceres::ResidualBlockId Matcher::addMotionModelFactor(ceres::Problem& problem, State& X_0, State& X_1, const bool& use_analytic_residual, ceres::LossFunction* loss) {
  double dt = X_1.stamp - X_0.stamp;
  ceres::ResidualBlockId res_id;
  if (use_analytic_residual) {
    res_id = problem.AddResidualBlock(
      new MotionModelFactorAnalytic(dt, parameters_.covariance_scaling_factor * parameters_.motion_sqrtI),
      nullptr, 
      &X_0.pos[0],
      &X_0.rot,
      &X_0.lin_vel[0],
      &X_0.rot_vel,
      &X_0.lin_acc[0],
      &X_1.pos[0],
      &X_1.rot,
      &X_1.lin_vel[0],
      &X_1.rot_vel,
      &X_1.lin_acc[0]); 
  }
  else {
    if (!parameters_.optimize_on_manifold) {
      res_id = problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<MotionModelFactor,8,2,1,2,1,2,2,1,2,1,2>(
          new MotionModelFactor(dt, parameters_.covariance_scaling_factor * parameters_.motion_sqrtI)), // square root information is matrix times scaling factor
        nullptr, 
        &X_0.pos[0],
        &X_0.rot,
        &X_0.lin_vel[0],
        &X_0.rot_vel,
        &X_0.lin_acc[0],
        &X_1.pos[0],
        &X_1.rot,
        &X_1.lin_vel[0],
        &X_1.rot_vel,
        &X_1.lin_acc[0]);  
    }
    else {
      res_id = problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<MotionModelFactorSE2,8,4,2,1,2,4,2,1,2>(
          new MotionModelFactorSE2(dt, parameters_.covariance_scaling_factor * parameters_.motion_sqrtI)),
        nullptr, 
        X_0.pose.data(),
        &X_0.lin_vel[0],
        &X_0.rot_vel,
        &X_0.lin_acc[0],
        X_1.pose.data(),
        &X_1.lin_vel[0],
        &X_1.rot_vel,
        &X_1.lin_acc[0]);   
    }
  }
  return res_id;
}

// not used
ceres::ResidualBlockId Matcher::addPriorFactor(ceres::Problem& problem, State& X_0, State lp, ceres::LossFunction* loss, Eigen::Matrix<double,9,9>& prior_sqrt_information) {
  ceres::ResidualBlockId res_id;
  if (!parameters_.optimize_on_manifold) {
    res_id = problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<PriorFactor,9,2,1,2,1,2,1>(
        new PriorFactor(lp.pos, lp.rot, lp.lin_vel, lp.rot_vel, lp.lin_acc, lp.imu_bias, prior_sqrt_information)),
      nullptr, 
      &X_0.pos[0],
      &X_0.rot,
      &X_0.lin_vel[0],
      &X_0.rot_vel,
      &X_0.lin_acc[0], 
      &X_0.imu_bias); 
  }
  else {
    res_id = problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<PriorFactorSE2,9,4,2,1,2,1>(
        new PriorFactorSE2(lp.pose, lp.lin_vel, lp.rot_vel, lp.lin_acc, lp.imu_bias, prior_sqrt_information)),
      nullptr, 
      X_0.pose.data(),
      &X_0.lin_vel[0],
      &X_0.rot_vel,
      &X_0.lin_acc[0], 
      &X_0.imu_bias);  
  }
  if (!parameters_.use_imu) {
    problem.SetParameterBlockConstant(&X_0.imu_bias); // if no imu is used, hold the bias constant as well
  }
  return res_id;
}

ceres::ResidualBlockId Matcher::addImuFactor(ceres::Problem& problem, State& X_0, State& X_1, std::vector<State>& trajectory, const double& weight_imu, const double& imu_constraint, const bool& use_analytic_residual, ceres::LossFunction* loss) {
  double dt = X_1.stamp - X_0.stamp;
  ceres::ResidualBlockId res_id;
  if (!use_analytic_residual) {
    if (!parameters_.optimize_on_manifold) {
      res_id = problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RotationalResidual,2,1,1,1,1>(
          new RotationalResidual(imu_constraint, weight_imu, dt, parameters_.weight_imu_bias)),
        loss, 
        &X_0.rot, 
        &X_1.rot, 
        &X_0.imu_bias, 
        &X_1.imu_bias);
    }
    else {
      res_id = problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RotationalResidualSE2,2,4,4,1,1>(
          new RotationalResidualSE2(imu_constraint, weight_imu, dt, parameters_.weight_imu_bias)),
        loss,
        X_0.pose.data(),
        X_1.pose.data(),
        &X_0.imu_bias, 
        &X_1.imu_bias);
    }
  }
  else {
    res_id = problem.AddResidualBlock(
      new RotationalResidualAnalytic(imu_constraint, weight_imu, dt, parameters_.weight_imu_bias),
      loss, 
      &X_0.rot, 
      &X_1.rot, 
      &X_0.imu_bias, 
      &X_1.imu_bias);
  }
  return res_id;
}

std::vector<ceres::ResidualBlockId> Matcher::addNDTFactor(ceres::Problem& problem, 
                                                          State& X, 
                                                          const Sophus::SE2d initial_guess,
                                                          const Map& fixed_ndt, 
                                                          const Map& moving_ndt, 
                                                          const bool& use_intensity, 
                                                          const bool& use_analytic_residual,
                                                          // const ceres::Manifold* manifold,
                                                          ceres::LossFunction* loss, 
                                                          int n_neighbours) {
  Eigen::Vector2f m_mean, f_mean;                // distribution parameters for both pure geometric and intensity-augmented matching
  Eigen::Matrix2f m_cov, f_cov;
  Eigen::Vector3f m_i_mean, f_i_mean;
  Eigen::Matrix3f m_i_cov, f_i_cov;

  std::vector<ceres::ResidualBlockId> resIds;     // we need the residuals to determine the convexity of the problem
  
  for (size_t i = 0; i < moving_ndt.get_n_cells(); i++){
    if (use_intensity) {
      moving_ndt.getCellMeanAndCovariance(i, m_i_mean, m_i_cov);   // moving cell parameters

      std::vector<size_t> indizes;  // neares neighbor indizes 

      if (parameters_.lookup_mahalanobis) {              // distribution based lookup
        Cell query_cell = moving_ndt.getCells()[i]; 
        const Eigen::Affine2f affine_trans(initial_guess.cast<float>().matrix());
        query_cell.transformCell(affine_trans); 
        fixed_ndt.getClosestCells(query_cell, n_neighbours, indizes);
      }
      else {
        Eigen::Vector2f query_vector = initial_guess.cast<float>() * m_i_mean.block<2,1>(0,0);
        fixed_ndt.getClosestCells(query_vector, n_neighbours, indizes);
      }

      for (size_t j = 0; j < indizes.size(); j++){
        fixed_ndt.getCellMeanAndCovariance(indizes[j], f_i_mean, f_i_cov);          // fixed ndt parameters
        if (use_analytic_residual) {
          ceres::ResidualBlockId ndt_res = problem.AddResidualBlock(
            new NDTFrameToMapIntensityFactorResidualAnalytic(m_i_mean.cast<double>(), m_i_cov.cast<double>(), f_i_mean.cast<double>(), f_i_cov.cast<double>()),
            loss,
            &X.pos[0],
            &X.rot); 
          resIds.push_back(ndt_res);
        }
        else {
          if (parameters_.optimize_on_manifold) {
            ceres::ResidualBlockId ndt_res = problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<NDTFrameToMapIntensityFactorResidualSE2, 1, 4>(
                new NDTFrameToMapIntensityFactorResidualSE2(m_i_mean.cast<double>(), m_i_cov.cast<double>(), f_i_mean.cast<double>(), f_i_cov.cast<double>())),
              loss, 
              X.pose.data());
            resIds.push_back(ndt_res);
          }
          else {
            ceres::ResidualBlockId ndt_res = problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<NDTFrameToMapIntensityFactorResidual, 1, 2, 1>(
                new NDTFrameToMapIntensityFactorResidual(m_i_mean.cast<double>(), m_i_cov.cast<double>(), f_i_mean.cast<double>(), f_i_cov.cast<double>())),
              loss, 
              &X.pos[0],
              &X.rot);   
            resIds.push_back(ndt_res);         
          }
        }
      }   
    }
    else { // without intensity
      moving_ndt.getCellMeanAndCovariance(i, m_mean, m_cov);
      Eigen::Vector2f query_vector = initial_guess.cast<float>() * m_mean;

      std::vector<size_t> indizes;
      fixed_ndt.getClosestCells(query_vector, n_neighbours, indizes);   // pure geometric lookup

      for (size_t j = 0; j < indizes.size(); j++){
        fixed_ndt.getCellMeanAndCovariance(indizes[j], f_mean, f_cov);
        if (use_analytic_residual) {
          ceres::ResidualBlockId ndt_res = problem.AddResidualBlock(
            new NDTFrameToMapFactorResidualAnalytic(m_mean.cast<double>(), m_cov.cast<double>(), f_mean.cast<double>(), f_cov.cast<double>()),
            loss,
            &X.pos[0],
            &X.rot); 
          resIds.push_back(ndt_res);
        } 
        else {
          if (!parameters_.optimize_on_manifold) {
            ceres::ResidualBlockId ndt_res = problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<NDTFrameToMapFactorResidual, 1, 2, 1>(
                new NDTFrameToMapFactorResidual(m_mean.cast<double>(), m_cov.cast<double>(), f_mean.cast<double>(), f_cov.cast<double>())),
              loss, 
              &X.pos[0],
              &X.rot); 
            resIds.push_back(ndt_res);
          }
          else {
            ceres::ResidualBlockId ndt_res = problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<NDTFrameToMapFactorResidualSE2, 1, 4>(
                new NDTFrameToMapFactorResidualSE2(m_mean.cast<double>(), m_cov.cast<double>(), f_mean.cast<double>(), f_cov.cast<double>())),
              loss, 
              X.pose.data());
            resIds.push_back(ndt_res);
          }
        }
      }
    }
  }
  return resIds;  // ids of the ndt residuals
}

void Matcher::addMotionParameterBlock(ceres::Problem& problem, ceres::Manifold* manifold, State& X, const bool& set_constant) {
  if (manifold) {
   problem.AddParameterBlock(X.pose.data(), 4, manifold);
  }
  else {
    problem.AddParameterBlock(&X.pos[0], 2);
    problem.AddParameterBlock(&X.rot, 1);
  }
  problem.AddParameterBlock(&X.lin_vel[0], 2);
  problem.AddParameterBlock(&X.rot_vel, 1);
  problem.AddParameterBlock(&X.lin_acc[0], 2);
  if (parameters_.use_constant_velocity_model) {
    problem.SetParameterBlockConstant(&X.lin_acc[0]);        // is kept at 0 if use_constant_velocity_model
  }
  if (set_constant) {
    if (manifold) {
      problem.SetParameterBlockConstant(X.pose.data());
    }
    else {
      problem.SetParameterBlockConstant(&X.pos[0]);
      problem.SetParameterBlockConstant(&X.rot);
    }
  }
}

void Matcher::addImuParameterBlock(ceres::Problem& problem, State& X, const bool& set_constant) {
  problem.AddParameterBlock(&X.imu_bias, 1);
  if (set_constant) {
    problem.SetParameterBlockConstant(&X.imu_bias);
  }
}

void Matcher::estimateTransformCeres(Sophus::SE2d& trans, std::vector<State>& trajectory, const double& initial_angle_guess, const double& stamp, const std::deque<Map>& fixed_ndts, const std::deque<Map>& moving_ndts) {
  
  const int n_neighbours = parameters_.n_results_kd_lookup;
  const int gnc_steps = parameters_.gnc_steps;
  const double weight_imu = parameters_.weight_imu;
  const double loss_function_scale = parameters_.loss_function_scale;
  ceres::Problem problem;
  ceres::Manifold* manifold;
  if (!parameters_.optimize_on_manifold || parameters_.use_analytic_expressions_for_optimization) { 
    manifold = nullptr;
  }
  else {
    manifold = new Sophus::Manifold<Sophus::SE2>();
  }
  std::vector<ceres::ResidualBlockId> ndt_residuals;
  ceres::LossFunctionWrapper* current_loss = new ceres::LossFunctionWrapper(nullptr, ceres::TAKE_OWNERSHIP);  // we need this to dynamically adapt the scaöe

  double prior_translation[2] = {trans.translation()(0), trans.translation()(1)};  // to reject large transforms
  double prior_rotation = trans.log()(2);
  
  int n_cells = 0;
  size_t smoothing_steps_iter = std::min(trajectory.size()-1, static_cast<size_t>(parameters_.smoothing_steps));  // limit smoothing steps to trajectory length
  if(trajectory.size() > 1 ) {

    // add parameter blocks
    // motion parameters
    addMotionParameterBlock(problem, manifold, trajectory.end()[-smoothing_steps_iter-1], true);

    // imu bias parameters
    if (parameters_.use_imu) {
      addImuParameterBlock(problem, trajectory.end()[-smoothing_steps_iter-1], true);   
    }

    for (size_t i = smoothing_steps_iter; i > 0; i--) {
      addMotionParameterBlock(problem, manifold, trajectory.end()[-i], false);
      addMotionModelFactor(problem, trajectory.end()[-i-1], trajectory.end()[-i], parameters_.use_analytic_expressions_for_optimization, nullptr);

      if (parameters_.use_imu) {
        addImuFactor(problem, trajectory.end()[-i-1], trajectory.end()[-i], trajectory, weight_imu, imu_constraints_.end()[-i-1], parameters_.use_analytic_expressions_for_optimization, nullptr);
      }

      for (const auto& fixed_ndt: fixed_ndts) {
        std::vector<ceres::ResidualBlockId> residuals = addNDTFactor(problem, trajectory.end()[-i], trajectory.end()[-i].pose, fixed_ndt, moving_ndts.end()[-i], parameters_.use_intensity_as_dimension, parameters_.use_analytic_expressions_for_optimization, current_loss, parameters_.n_results_kd_lookup);
        ndt_residuals.insert(ndt_residuals.end(), residuals.begin(), residuals.end());
      }
      n_cells += moving_ndts.end()[-i].get_n_cells();  // keep track of all ndt residuals to scale them appropriately
    }
  }
  n_cells_ = n_cells;

  ceres::Solver::Options options;
  options.max_num_iterations = parameters_.max_iteration;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type=ceres::LEVENBERG_MARQUARDT;
  options.num_threads = std::thread::hardware_concurrency();     // enable mutlithreading

  options.check_gradients = false;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;

  ceres::Problem::EvaluateOptions eval_options;
  std::vector<double> raw_residuals, robust_residuals;
  eval_options.residual_blocks = ndt_residuals;
  eval_options.apply_loss_function = false;  // raw residuals needed to compute statistics
  problem.Evaluate(eval_options, nullptr, &raw_residuals, nullptr, nullptr);
  const double max_residual = *std::max_element(raw_residuals.begin(), raw_residuals.end());
  double gnc_mu = 2.0 * std::pow(max_residual, 2)/std::pow(parameters_.loss_function_scale, 2);    // see GNC paper
  gnc_mu = std::min(gnc_mu, std::pow(parameters_.gnc_control_parameter_divisor, parameters_.gnc_steps-1));  // limit maximum graduation steps
  do { 
    gnc_mu = std::max(gnc_mu, 1.0);
    ceres::LossFunction* scaled_loss = new ceres::ScaledLoss(new ceres::BarronLoss(loss_function_scale, parameters_.loss_function_convexity, gnc_mu), parameters_.ndt_weight/static_cast<double>(n_cells*n_neighbours), ceres::TAKE_OWNERSHIP);
    current_loss->Reset(scaled_loss, ceres::TAKE_OWNERSHIP);  // adjust loss function of ndt residuals
    ceres::Solve(options, &problem, &summary);                // solve optimization problem
    gnc_mu /= parameters_.gnc_control_parameter_divisor;      // reduce convextity
    total_optimization_time_ += summary.minimizer_time_in_seconds;
  } while (gnc_mu > 1.0/std::sqrt(parameters_.gnc_control_parameter_divisor));  // float comparison

  if (!parameters_.optimize_on_manifold || parameters_.use_analytic_expressions_for_optimization) {  // update estimate in both representations
    Sophus::SE2d last_transform(trajectory.end()[-1].rot, trajectory.end()[-1].pos);
    trajectory.end()[-1].pose = last_transform;
  }
  else {
    trajectory.end()[-1].pos = trajectory.end()[-1].pose.translation();
    trajectory.end()[-1].rot = trajectory.end()[-1].pose.log()(2);
  }

  double num_optimizations = static_cast<double>(trajectory.size());

  // reject if new estimate deviates too much from initial guess
  if((std::abs(trajectory.end()[-1].pose.translation()(0) - prior_translation[0]) > parameters_.pose_reject_translation || 
       std::abs(trajectory.end()[-1].pose.translation()(1) - prior_translation[1]) > parameters_.pose_reject_translation || 
       std::abs((trajectory.end()[-1].pose.so2().inverse() * Sophus::SO2d(prior_rotation)).log()) > parameters_.pose_reject_rotation)) {  
    std::cout << "Rejected new estimated transform!" << std::endl;
    trajectory.end()[-1].pos = trajectory.end()[-2].pos;
    trajectory.end()[-1].pose = trajectory.end()[-2].pose;
    trajectory.end()[-1].rot = trajectory.end()[-2].rot;
    trajectory.end()[-1].lin_vel = Eigen::Vector2d::Zero();
    trajectory.end()[-1].rot_vel = 0.0;
    trajectory.end()[-1].lin_acc = Eigen::Vector2d::Zero();
    trajectory.end()[-1].imu_bias = trajectory.end()[-2].imu_bias;
  }
  trans = trajectory.end()[-1].pose;  // return current transform
}

double Matcher::estimateLoopConstraint(Sophus::SE2d& trans, const Map& old_ndt, Map& new_ndt, int max_gnc_steps, bool use_intensity_as_dimension, double scale) {
  ceres::Problem problem;
  ceres::Manifold* manifold;
  if (!parameters_.optimize_on_manifold || parameters_.use_analytic_expressions_for_optimization) { 
    manifold = nullptr;
  }
  else {
    manifold = new Sophus::Manifold<Sophus::SE2>();
  }

  State two_representation_state; // both representations should hold the same representation
  if (parameters_.use_analytic_expressions_for_optimization || !parameters_.optimize_on_manifold) {
    two_representation_state.pos = trans.translation(); 
    two_representation_state.rot = trans.log()(2);
    problem.AddParameterBlock(&two_representation_state.pos[0], 2);
    problem.AddParameterBlock(&two_representation_state.rot, 1);
  }
  else {
    two_representation_state.pose = trans;
    problem.AddParameterBlock(trans.data(), 4, manifold);
  }

  ceres::LossFunctionWrapper* current_loss = new ceres::LossFunctionWrapper(nullptr, ceres::TAKE_OWNERSHIP); 
  std::vector<ceres::ResidualBlockId> ndt_residuals;
  
  // add all residuals
  ndt_residuals = addNDTFactor(problem, two_representation_state, trans, old_ndt, new_ndt, use_intensity_as_dimension, parameters_.use_analytic_expressions_for_optimization, current_loss, 4);
  
  if (ndt_residuals.size() == 0) {
    std::cout << "WARNING: NO RESIDUALS ADDED!" << std::endl;
  }
  ceres::Solver::Options options;
  options.max_num_iterations = parameters_.max_iteration;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type=ceres::LEVENBERG_MARQUARDT;
  options.num_threads = std::thread::hardware_concurrency();

  options.check_gradients = false;
  options.minimizer_progress_to_stdout = false;

  ceres::Problem::EvaluateOptions eval_options;
  ceres::Solver::Summary summary;
  std::vector<double> raw_residuals, robust_residuals;
  raw_residuals.reserve(ndt_residuals.size());
  eval_options.residual_blocks = ndt_residuals;
  eval_options.apply_loss_function = false;
  problem.Evaluate(eval_options, nullptr, &raw_residuals, nullptr, nullptr);

  const double max_residual = *std::max_element(raw_residuals.begin(), raw_residuals.end());
  double gnc_mu = 2.0 * std::pow(max_residual, 2)/std::pow(parameters_.loss_function_scale, 2);
  gnc_mu = std::min(gnc_mu, std::pow(parameters_.gnc_control_parameter_divisor, max_gnc_steps-1));
  do { 
    gnc_mu = std::max(gnc_mu, 1.0);
    ceres::LossFunction* scaled_loss = new ceres::ScaledLoss(new ceres::BarronLoss(scale, parameters_.loss_function_convexity, gnc_mu), 1, ceres::TAKE_OWNERSHIP);
    current_loss->Reset(scaled_loss, ceres::TAKE_OWNERSHIP);
    ceres::Solve(options, &problem, &summary);
    gnc_mu /= parameters_.gnc_control_parameter_divisor;
  } while (gnc_mu > 1.0/std::sqrt(parameters_.gnc_control_parameter_divisor)); // float comparison...

  if (parameters_.use_analytic_expressions_for_optimization || !parameters_.optimize_on_manifold) {
    const Sophus::SE2d return_trans(two_representation_state.rot, two_representation_state.pos);
    trans = return_trans;
  }
  else {
    trans = two_representation_state.pose;
  }  
  return summary.final_cost/summary.num_residual_blocks; // return cost divided by all cells
}

double Matcher::estimateTransformGlobalBNB(Sophus::SE2d& trans, const Map& fixed_ndt, Map& moving_ndt, bool use_intensity_as_dimension, double scale, double search_window_size_linear, double search_window_size_angular) {
  ceres::Problem problem;
  ceres::Manifold* manifold;
  if (!parameters_.optimize_on_manifold || parameters_.use_analytic_expressions_for_optimization) { 
    manifold = nullptr;
  }
  else {
    manifold = new Sophus::Manifold<Sophus::SE2>();
  }

  search_window_size_linear = std::min(search_window_size_linear, parameters_.csm_window_linear);
  search_window_size_angular = std::min(search_window_size_angular, parameters_.csm_window_angular);

  State two_representation_state;
  if (parameters_.use_analytic_expressions_for_optimization || !parameters_.optimize_on_manifold) {
    problem.AddParameterBlock(&two_representation_state.pos[0], 2);
    problem.AddParameterBlock(&two_representation_state.rot, 1);
  }
  else {
    problem.AddParameterBlock(trans.data(), 4, manifold);
  }

  ceres::LossFunction* current_loss = new ceres::BarronLoss(scale, parameters_.loss_function_convexity);
  std::vector<ceres::ResidualBlockId> ndt_residuals;

  ndt_residuals = addNDTFactor(problem, two_representation_state, trans, fixed_ndt, moving_ndt, use_intensity_as_dimension, parameters_.use_analytic_expressions_for_optimization, current_loss, 4);

  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = std::thread::hardware_concurrency();
  eval_options.residual_blocks = ndt_residuals;
  eval_options.apply_loss_function = true;
  double cost = 0.0;

  // construct search grid
  const double linear_step = parameters_.csm_linear_step;
  const double max_range = parameters_.csm_max_px_accurate_range;
  const double angular_step = std::acos(1-((linear_step * linear_step)/(2* max_range * max_range)));
  const double cost_threshold = parameters_.csm_cost_threshold; 
  size_t n_angular_steps = static_cast<size_t>(search_window_size_angular / angular_step);
  size_t n_linear_steps = static_cast<size_t>(search_window_size_linear / linear_step);
  size_t n_iter = parameters_.csm_n_iter;

  // already evaluated points
  std::vector<std::vector<float>> calculated_points;

  double initial_linear_step = std::pow(2, n_iter-1) * linear_step;
  double initial_angular_step = angular_step; 
  double min_cost = 100000.0;
  size_t n_initial_linear_samples = static_cast<size_t>(search_window_size_linear / initial_linear_step); 
  size_t n_initial_angular_samples = static_cast<size_t>(search_window_size_angular / initial_angular_step); 
  Sophus::SE2d best_trans;
  Sophus::SE2d current_trans;
  std::deque<std::tuple<Sophus::SE2d, double, size_t>> query_transforms;

  // evaluate coarsest resolution
  for (double tx = -search_window_size_linear/2.0; tx <= search_window_size_linear/2.0; tx += initial_linear_step) {
    for (double ty = -search_window_size_linear/2.0; ty <= search_window_size_linear/2.0; ty += initial_linear_step) {
      for (double a = -search_window_size_angular/2.0; a < search_window_size_angular/2.0; a += initial_angular_step) {
        current_trans = trans * Sophus::SE2d(a, {tx, ty});
        query_transforms.emplace_back(std::make_tuple(current_trans, 0.0, 1));
        std::vector<float> vec(current_trans.matrix().data(), current_trans.matrix().data() + current_trans.matrix().rows() * current_trans.matrix().cols());
        calculated_points.push_back(vec);
      }
    }
  }
  while(true) {
    two_representation_state.pose = std::get<0>(query_transforms.front());
    if (parameters_.use_analytic_expressions_for_optimization || !parameters_.optimize_on_manifold) {
      two_representation_state.pos = two_representation_state.pose.translation();
      two_representation_state.rot = two_representation_state.pose.log()(2);
    }
    const size_t current_level = std::get<2>(query_transforms.front());
    cost = 0.0;
    problem.Evaluate(eval_options, &cost, nullptr, nullptr, nullptr);  // calculate pose
    double sq_distance;
    if (parameters_.use_analytic_expressions_for_optimization || !parameters_.optimize_on_manifold) {
      sq_distance = (two_representation_state.pos - trans.translation()).squaredNorm() / (search_window_size_linear * search_window_size_linear); 
    }
    else {
      sq_distance = (two_representation_state.pose.translation() - trans.translation()).squaredNorm() / (search_window_size_linear * search_window_size_linear); 
    }
    double current_cost = cost/problem.NumResidualBlocks();    
    
    if (current_cost < cost_threshold) { // continue if below cost threshold
      if (current_cost < min_cost) {
        best_trans = std::get<0>(query_transforms.front());
        min_cost = current_cost;
      }
      if (current_level < n_iter) {      // and if recursion level not yet reached
        double current_linear_step = std::pow(2.0, current_level) * linear_step;
        double current_angular_step = angular_step; 
        for (double tx = -current_linear_step; tx <= current_linear_step; tx += current_linear_step) {
          for (double ty = -current_linear_step; ty <= current_linear_step; ty += current_linear_step) {
            for (double a = -current_angular_step; a <= current_angular_step; a += current_angular_step) {
              current_trans = Sophus::SE2d(a, {tx, ty});
              Sophus::SE2d sampled_transform = std::get<0>(query_transforms.front()) * current_trans;
              std::vector<float> vec(sampled_transform.matrix().data(), sampled_transform.matrix().data() + sampled_transform.matrix().rows() * sampled_transform.matrix().cols());
              if (std::find(calculated_points.begin(), calculated_points.end(), vec) == calculated_points.end()) {
                calculated_points.push_back(vec);
                query_transforms.emplace_back(std::make_tuple(sampled_transform, 0.0, current_level + 1));
              }
            }
          }
        }
      }
    }
    query_transforms.pop_front();
    if (query_transforms.empty()) {
      break;
    }
  }
  trans = best_trans;
  return min_cost;  // return best transform
}

}
}
}