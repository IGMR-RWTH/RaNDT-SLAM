#ifndef NDT_HIERARCHICAL_MAP_H
#define NDT_HIERARCHICAL_MAP_H

#include "ndt_representation/ndt_map.h"
#include "ndt_slam/ndt_slam_parameters.h"

#include <ndt_msgs/Covariance.h>
#include <ndt_msgs/Mean.h>
#include <ndt_msgs/NormalDistribution.h>
#include <ndt_msgs/NormalDistributions.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sophus/se2.hpp>

namespace rc {
namespace navigation {
namespace ndt {
class HierarchicalMap {
  public:
    //HierarchicalMap(Parameters parameters): parameters_(parameters){};
    /** @brief initialize new hierarchical map
    *  @param parameters parameters of the cell
    *  @param center should be 0
    *  @param origin_in_global_frame submap origin in global frame
    *  @param params ndt cell parameters
    */
    void initialize(NDTMapParameters parameters, Eigen::Vector2f center, Eigen::Affine2f origin_in_global_frame);
   
    /** @brief add clustered point clouds
    *  @param clusters vector of point clouds, each corresponding to a generated cluster
    *  @param angle_dists angle and range of points, needed for pNDT
    *  @param max_detections maximum intensity points per azimuth for OGM generation
    */
    void addClusters(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters, const std::vector<std::vector<std::pair<double, double>>>& angle_dists, const std::vector<std::tuple<double, double, double>>& max_detections);;
    
    /** @brief transform map
    *  @param trans transform to apply to map
    */
    void transformMap(const Eigen::Affine2f& trans);

    /** @brief transform map with point clouds in cells
    *  @param trans transform to apply to map
    */
    void transformMapWithPointCloud(const Eigen::Affine2f& trans);


    /** @brief transform map to new origin
    *  @param trans new origin of submap
    */
    void transformMapToOrigin(const Sophus::SE2d& new_origin);

    /** @brief get map
    *  @param map variable to write map into
    */
    inline Map getMap() const {
      return ndt_map_;
    }

    /** @brief merge two maps
    *  @param m_map map to merge into current map
    */
    void mergeMapCell(const HierarchicalMap& m_map);

    /** @brief get OGM of submap
    */ 
    std::shared_ptr<nav_msgs::OccupancyGrid> getOGM();

    /** @brief get OGM counter of submap
    */ 
    std::pair<Eigen::Affine2f, std::shared_ptr<std::vector<int>>> getSubmapCounts();
    
    /** @brief clear submap
    */ 
    void clear(void);

    /** @brief return true if submap is empty
    */ 
    inline bool isEmpty() const {
      return is_empty;
    }

    /** @brief get origin of the current submap
    */ 
    inline Eigen::Affine2f getOrigin() const {
      return origin_in_global_frame_;
    }
    
    /** @brief raytrace a ray of a sensor in OGM
    *  @param from origin of the sensor in OGM frame
    *  @param line line, characterized by angle, range, and intensity(?)
    *  @param max_length (?)
    */ 
    void raytraceLine(const Eigen::Affine2f& from, const std::tuple<double, double, double>& line, int max_length);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private: 

    /** @brief bresenham algorithm
    */ 
    void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length);
    NDTMapParameters parameters_; /** parameters of the map */
    Map ndt_map_; /** ndt map representation */
    bool is_empty = true; /** store if map is empty */
    Eigen::Affine2f origin_in_global_frame_, ogm_map_origin_; /** origins of the map in global frame */
    std::shared_ptr<nav_msgs::OccupancyGrid> grid_map_; /** OGM of the submap */
    std::shared_ptr<std::vector<int>> simple_grid_; /** OGM counter of the submap*/
    double ogm_res_; /** resolution of the ogm */
    unsigned int ogm_size_x_, ogm_size_y_; /** ogm sizes */
    
    /** @brief return sign
    */
    template <typename T> int sgn(T val) {
      return (T(0) < val) - (val < T(0));
    }

    /** @brief clamp value between lower and upper
    */
    template <typename T>
    T clamp(const T& n, const T& lower, const T& upper) {
      return std::max(lower, std::min(n, upper));
    }
};
}
}
}

#endif