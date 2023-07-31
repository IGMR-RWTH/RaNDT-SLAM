/** @file
 *  @brief The local fuser is the front-end of the SLAM system
 *
 *  The local fuser is the front-end of the SLAM system. It handles the sensor data, generates local motion estimates, and detects the loop closures.
 *
 *  @date 23.07.2023
 *
 */

#ifndef LOCAL_FUSER_H
#define LOCAL_FUSER_H

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include "radar_preprocessing/radar_preprocessor.h"
#include "ndt_representation/ndt_hierarchical_map.h"
#include <ndt_representation/ndt_master_map.h>
#include "ndt_slam/ndt_slam_parameters.h"
#include <ndt_slam/trajectory_representation.h>
#include "ndt_registration/ndt_matcher.h"
#include "Scancontext.h"

#include <chrono>

namespace rc {
namespace navigation {
namespace ndt {

/** @brief The Local Fuser Class is the front-end of the SLAM system
 *
 *
 * */
class LocalFuser {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** @brief initialize the local_fuser
    *  @param parameters parameters of the local fuser
    *  @param initial_transform initial transform from map frame to initial frame
    *  @param initial_transform_radar_baselink transform between radar sensor and base frame. Needed to carry out registration in robot-centric frame
    */
    void initialize(LocalFuserParameters parameters, Eigen::Affine2f initial_transform, Eigen::Affine3f initial_transform_radar_baselink);
    
    /** @brief initialize new submap
    *  @param intial_transform origin of new submap in map frame
    */
    void initializeNewSubmap(Eigen::Affine2f initial_transform);
    
    /** @brief process new IMU data
    *  @param msg new IMU message
    */
    void processImu(const sensor_msgs::Imu::ConstPtr& msg);
    
    /** @brief process incoming radar scans
    *  @param msg PointCloud2 message containing radar points
    *  @param clusters todo weg: clusters containing all points in NDT cell
    *  @param nodes poses in the pose graph
    *  @param stamps time stamps corresponding to the nodes in the pose graph
    *  @param edges edges in the pose graph
    *  @param submap_idzs map with the submaps corresponding to each pose graph node
    *  @param nodes_mutex mutex protecting the pose graph nodes
    */
    void processScan(const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters, std::map<int, Pose>& nodes, std::map<int, std_msgs::Header>& stamps, std::vector<Constraint>& edges, std::map<int, int>& submap_idzs, std::mutex& nodes_mutex);
    
    /** @brief check if submap is complete
    *  @return true if submap complete, otherwise false
    */
    bool submapComplete();
    
    /** @brief trigger raytracing queue
    *  @param map_mutex mutex protecting OGM
    */
    void raytrace(std::mutex& map_mutex);
    
    /** @brief get NDT representation of last radar Scan
    *  @param map reference to the NDT map in that the scan shall be stored
    */
    void getScanMap(HierarchicalMap& map);
    
    /** @brief get NDT representation of the last radar scan
    *  @return NDT map
    */
    HierarchicalMap getLastMergedMap();
    
    /** @brief get NDT representation of the current submap
    *  @return NDT map
    */
    void getSubmap(HierarchicalMap& map);
    
    /** @brief update submap origins according to pose graph
    *  @param nodes nodes of the pose graph
    *  @param submap_idzs map with the submaps corresponding to each pose graph node
    */
    void updateSubmaps(const std::map<int, Pose>& nodes, const std::map<int, int>& submap_idzs);
    
    /** @brief detect new loop closures for the last inserted poses
    *  @param nodes nodes of the pose graph
    *  @param edges edges of the pose graoh
    *  @param submap_idzs map with the submaps corresponding to each pose graph node
    *  @param nodes_mutex mutex protecting the pose graph nodes
    */
    void detectLoopClosures(std::map<int, Pose>& nodes, std::vector<Constraint>& edges, std::map<int, int>& submap_idzs, std::mutex& nodes_mutex);
    
    /** @brief get trajectory in current submap
    *  @return all poses in the current submap
    */
    std::vector<State> getTrajectory();
    
    /** @brief current transform of the robot in map frame
    *  @return transform
    */
    inline Eigen::Affine2f getTransform() const {
      const Eigen::Affine2f return_trans((current_global_transform_ * current_transform_).cast<float>().matrix());
      return return_trans;
    }

    /** @brief get origin of the current submap in map frame
    *  @return transform
    */
    inline Eigen::Affine2f getGlobalTransform() const {
      const Eigen::Affine2f return_trans(current_global_transform_.cast<float>().matrix());
      return return_trans;
    }

    /** @brief get OGM
    *  @return shared pointer to the OGM
    */
    inline std::shared_ptr<nav_msgs::OccupancyGrid> getOGM(std::mutex& ogm_mutex) {
      return ogm_.getOGM(ogm_mutex);
    };
    
    /** @brief get filtered scan
    *  @return filtered radar scan
    */
    inline pcl::PointCloud<pcl::PointXYZI>::Ptr getTransformedScan() {
      return _preprocessor.getDebugCloud();
    }

  private:
    RadarPreprocessor _preprocessor; /** radar preprocessor */
    LocalFuserParameters parameters_; /** parameters */
    HierarchicalMap _current_submap; /** current ndt submap */
    HierarchicalMap _current_scan_map; /** ndt map representing the last radar scan */
    HierarchicalMap _last_merged_map; /** ndt map that was last merged into submap */
    HierarchicalMap _last_submap_transformed; /** last submap, tranformed in current submap frame */ 
    MasterMap ogm_; /** ndt map */
    std::deque<HierarchicalMap> _next_maps_to_insert; /** queue of the next radar scan ndt maps to be inserted into the submap */
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> _next_scans_to_insert; /** queue of the next radar scan point clouds to be inserted into the submap */
    std::deque<std_msgs::Header> _next_stamps_to_insert; /** queue of the next radar scan time stamps to be inserted into the submap */
    std::deque<std::vector<std::tuple<double, double, double>>> _next_max_to_insert; /** queue of the next maximum intensity points per azimuth to be inserted into the submap */
    std::deque<int> _next_maps_to_search_loop; /** indizes of the next ndt maps to search possible loop closures */
    std::deque<Map> map_window_; /** window of submaps the new scan is matched to */
    std::deque<std::pair<Eigen::Affine2f, std::tuple<double,double,double>>, Eigen::aligned_allocator<std::pair<Eigen::Affine2f, std::tuple<double,double,double>>>> raytracing_queue_; /** queue of rays to be used in raytracing */
    sensor_msgs::Imu last_imu_msg_, imu_msg_; /** imu messages */
    Matcher matcher_; /** ndt matcher */
    Sophus::SE2d current_transform_; /** current robot location in submap frame */
    Sophus::SE2d current_global_transform_; /** origin of current submap in global frame */
    std::vector<State> trajectory_; /** history of states within submap */
    double last_imu_bias_; /** imu bias of the last state */
    double total_time_; /** complete time spent in this component*/
    int n_scans = 0; /** number of processed scans*/

    int current_submap_root_node_id_ = 0; /** id of the current submaps root node */
    int current_node_id_ = 0; /** id of the current node */
    Pose current_node_; /** pose of the current node */
    Pose current_submap_root_node_; /** pose of the root node of the current submap*/
    bool initial_node_set_ = false; /** if the first node is set */
    bool write_on_queue_ = false; /** protect raytracing queue */
    bool read_on_queue_ = false; /** protect raytracing queue */
    std::map<int, HierarchicalMap> submaps_; /** map of all submaps*/
    std::map<int, int> root_nodes_; /** map storing the root node ids of all submaps */
    int n_finished_submaps_ = 0; /** number of finished submaps */

    std::map<int, HierarchicalMap> scans_; /** ndt maps of scans associated to all nodes in the pose graph */
    State last_state_; /** state to keep information about velocity and biases */ 
    SCManager scManager_; /** scan context for loop closure */
    Eigen::Affine2f baselink_to_radar_2d_; /** transform between sensor and robot frame */
};
}
}
}

#endif