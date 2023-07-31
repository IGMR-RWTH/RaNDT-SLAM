#ifndef NDT_MATCHER_H
#define NDT_MATCHER_H

#include "ceres_loss_functions.h"
#include "ceres_residuals.h"
#include "state_manifold.h"

#include <ndt_representation/ndt_map.h>
#include <ndt_slam/trajectory_representation.h>
#include "ndt_slam/ndt_slam_parameters.h"
#include <Eigen/Core>

#include <ceres/loss_function.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <sophus/ceres_manifold.hpp>

#include <chrono>
#include <thread>
#include <deque>

namespace rc{
namespace navigation {
namespace ndt {

struct MapTuple {
  public:
    Map _moving_ndt;
    Map _fixed_ndt;
};

/** @brief The NDT matcher 
 * 
 * The NDT matcher performs both local motion estimation and NDT matching for loop closure detection
 *
 * */
class Matcher {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief initialize the local_fuser
    *  @param parameters parameters of the matcher
    */
    void initialize(NDTMatcherParameters parameters); 

    /** @brief match NDTs for loop closure detection
    *  @param trans transformation from submap to new scan
    *  @param fixed_ndt submap
    *  @param moving_ndt new scan
    *  @param max_gnc_steps number of graduation steps in loop search
    *  @param use_intensity_as_dimension if true match intensity-augmented distributions
    *  @param scale scale of robust loss function
    */
    double estimateLoopConstraint(Sophus::SE2d& trans, const Map& fixed_ndt, Map& moving_ndt, int max_gnc_steps, bool use_intensity_as_dimension, double scale);

    /** @brief match NDTs with IMU and motion model for local motion estimation
    *  @param trans transformation from submap to new scan
    *  @param trajectory trajectory within submap
    *  @param initial_angle_guess measured IMU difference
    *  @param stamp time stamp of newest scan
    *  @param fixed_ndts window of submaps to match against
    *  @param moving_ndts window of scans to match against submaps
    */
    void estimateTransformCeres(Sophus::SE2d& trans, std::vector<State>& trajectory, const double& initial_angle_guess, const double& stamp, const std::deque<Map>& fixed_ndts, const std::deque<Map>& moving_ndts);

    
    /** @brief match NDTs with CSM
    *  @param trans transformation from submap to new scan
    *  @param fixed_ndt submap to match against
    *  @param moving_ndt scan to match against submap
    *  @param use_intensity_as_dimension if true match intensity-augmented distributions
    *  @param scale scale of robust loss function
    *  @param search_window_linear linear size of search window
    *  @param search_window_angular angular size of search window
    */
    double estimateTransformGlobalBNB(Sophus::SE2d& trans, const Map& fixed_ndt, Map& moving_ndt, bool use_intensity_as_dimension, double scale, double search_window_size_linear, double search_window_size_angular);

    /** @brief predict motion based on state history
    *  @param initial_angle_guess measured IMU rotation difference
    *  @param stamp time to estimate state
    *  @param trajectory past trajectory within submap
    */
    void predictTransform(const double& initial_angle_guess, const double& stamp, std::vector<State>& trajectory);

    void resetMatcher();

  protected:
    
    /** @brief add parameters needed for motion residual
    *  @param problem reference to ceres problem
    *  @param manifold pointer to ceres manifold
    *  @param X reference to newly added state
    *  @param set_constant if true, set the added state constant
    */
    void addMotionParameterBlock(ceres::Problem& problem, ceres::Manifold* manifold, State& X, const bool& set_constant);
    
    /** @brief add parameters needed for IMU residual 
    *  @param problem reference to ceres problem
    *  @param X reference to newly added state
    *  @param set_constant if true, set the added state constant
    */
    void addImuParameterBlock(ceres::Problem& problem, State& X, const bool& set_constant);

    /** @brief add motion model residual
    *  @param problem reference to ceres problem
    *  @param X_0 reference to old state
    *  @param X_1 reference to new state
    *  @param use_analytic_residual if true, use analytic residual
    *  @param loss pointer to possible robust loss function
    */
    ceres::ResidualBlockId addMotionModelFactor(ceres::Problem&        problem, 
                                                State&                 X_0, 
                                                State&                 X_1, 
                                                const bool&            use_analytic_residual,
                                                ceres::LossFunction*   loss);

    /** @brief add IMU residual
    *  @param problem reference to ceres problem
    *  @param X_0 reference to old state
    *  @param X_1 reference to new state
    *  @param trajectory trajectory within current submap
    *  @param weight_imu weight of the imu error
    *  @param imu_constraint measured IMU difference
    *  @param use_analytic_residual if true, use analytic residual
    *  @param loss pointer to possible robust loss function
    */
    ceres::ResidualBlockId addImuFactor(ceres::Problem&        problem, 
                                        State&                 X_0, 
                                        State&                 X_1, 
                                        std::vector<State>&    trajectory, 
                                        const double&          weight_imu, 
                                        const double&          imu_constraint, 
                                        const bool&            use_analytic_residual,
                                        ceres::LossFunction*   loss);

    /** @brief add NDT residual
    *  @param problem reference to ceres problem
    *  @param X reference to state
    *  @param initial_guess initial guess for estimate
    *  @param fixed_ndt ndt map of submap
    *  @param moving_ndt ndt map of scan
    *  @param use_intensity if true, use intensity information in matching
    *  @param use_analytic_residual if true, use analytic residual
    *  @param loss pointer to possible robust loss function
    *  @param n_neighbors number of neighbors used for each moving ndt cell
    */                                   
    std::vector<ceres::ResidualBlockId> addNDTFactor(ceres::Problem&        problem, 
                                                     State&                 X, 
                                                     const Sophus::SE2d     initial_guess,
                                                     const Map&             fixed_ndt, 
                                                     const Map&             moving_ndt, 
                                                     const bool&            use_intensity, 
                                                     const bool&            use_analytic_residual,
                                                     ceres::LossFunction*   loss,
                                                     int                    n_neighbours);

    /** @brief add prior residual
    *  @param problem reference to ceres problem
    *  @param X_0 reference to estimated state
    *  @param lp prior location of that state
    *  @param loss pointer to possible robust loss function
    *  @param prior_sqrt_information square root information of the prior 
    */
    ceres::ResidualBlockId addPriorFactor(ceres::Problem& problem, State& X_0, State lp, ceres::LossFunction* loss, Eigen::Matrix<double,9,9>& prior_sqrt_information);
    std::vector<double> imu_constraints_; /** vector of measured imu differences */
    State X_next_; /** next state */
    NDTMatcherParameters parameters_; /** parameters of the matcher */
    int n_cells_; /** number of cells */
    double total_optimization_time_; /** total time spent in optimization */
};

}
}
}

#endif