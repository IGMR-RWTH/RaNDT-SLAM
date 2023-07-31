/** @file
 *  @brief The global fuser handles the optimziation of the pose graph
 *
 *  The global fuser handles the optimziation of the pose graph. It also generates covariances for loop closure detection
 *
 *  @date 23.07.2023
 *
 */

#ifndef GLOBAL_FUSER_H
#define GLOBAL_FUSER_H

#include "ndt_slam/ndt_slam_parameters.h"
#include "ndt_slam/trajectory_representation.h"
#include "ndt_registration/ndt_matcher.h"
#include "pose_graph_2d_error_term.h"
#include <map>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

namespace rc {
namespace navigation {
namespace ndt {

/** @brief The Global Fuser Class handling the pose graph
 *
 *
 * */
class GlobalFuser {
  public:
    /** @brief Initialize the global fuser
    *  @param parameters global fuser parameters
    */
    void initialize(GlobalFuserParameters parameters);
    
    /** @brief add Poses as nodes to the pose graph
    *  @param poses pairs of key and poses
    */
    void addPoses(const std::map<int, Pose>& poses);
    /** @brief add constraints as edges of the pose graph
    *  @param edges vector of constraints
    */
    void addEdges(const std::vector<Constraint>& edges);
    /** @brief optimize the pose graph given constraints and poses
    *  @param poses_ref poses in the pose graph
    *  @param edges constraints in the pose graph
    *  @param poses_mutex mutex protecting the nodes of the pose graph
    *  @param max_update_index index of the last node of the most recent finished submap. For all following indices, only odometry constraints are considered
    */
    void optimizePoseGraph(std::map<int, Pose>& poses_ref, const std::vector<Constraint>& edges, std::mutex& poses_mutex, int max_update_index);

  private:
    int n_optimized_poses_; /** number of active poses */
    int n_optimized_constraints_; /** number of active constraints */
    GlobalFuserParameters parameters_; /** global fuser parameters */
};
}
}
}

#endif