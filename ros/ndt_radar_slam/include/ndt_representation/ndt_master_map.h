#ifndef NDT_MASTER_MAP_H
#define NDT_MASTER_MAP_H

#include "ndt_slam/ndt_slam_parameters.h"
#include "ndt_representation/ndt_hierarchical_map.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sophus/se2.hpp>
#include <functional>
#include <map>
#include <omp.h>
#define THREAD_NUM 4


namespace rc {
namespace navigation {
namespace ndt {

class MasterMap {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** @brief initialize master map
    *  @param parameters parameters of the master map
    *  @param origin_in_global_frame origin of the ogm in global map frame
    */
    void initialize(OGMMapParameters parameters, Eigen::Affine2f origin_in_global_frame);

    /** @brief update ogms based on new submap locations
    */
    inline void updateSubmaps(const std::map<int, std::pair<Eigen::Affine2f, std::shared_ptr<std::vector<int>>>>& updated_submaps) {
      submap_counts_ = updated_submaps;
    };

    /** @brief get closest cells of cell
    *  @param ogm_mutex mutex protecting the OGM
    *  @return shared pointer to OGM
    */
    std::shared_ptr<nav_msgs::OccupancyGrid> getOGM(std::mutex& ogm_mutex);
  private:
    OGMMapParameters parameters_; 
    Eigen::Affine2f origin_in_global_frame_, ogm_map_origin_;
    Eigen::Affine2f to_ogm_map_origin_ = Eigen::Affine2f::Identity(); 
    double submap_ogm_res_, ogm_res_, ogm_size_x_, ogm_size_y_; 
    std::map<int, std::pair<Eigen::Affine2f, std::shared_ptr<std::vector<int>>>> submap_counts_;
    std::shared_ptr<std::vector<double>> simple_grid_, simple_grid_one_delay_;
    
    template <typename T>
    T clamp(const T& n, const T& lower, const T& upper) {
      return std::max(lower, std::min(n, upper));
    }
};

}}}

#endif