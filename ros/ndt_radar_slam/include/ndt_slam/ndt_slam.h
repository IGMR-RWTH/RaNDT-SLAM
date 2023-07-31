#ifndef NDT_SLAM_H
#define NDT_SLAM_H

#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ndt_msgs/ClusteredPointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/cache.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <local_fuser/local_fuser.h>
#include <global_fuser/global_fuser.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include "ndt_registration/ndt_matcher.h"
#include "ndt_slam_parameters.h"


namespace rc {
namespace navigation {
namespace ndt {
class NDTSlam {
  public:
    NDTSlam();
  private:
    std::mutex m;
    bool first_received = false;
    ros::NodeHandle _nh;

    // first thread
    ros::Subscriber _point_cloud_sub;

    // second thread
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    message_filters::TimeSequencer<sensor_msgs::Imu> seq;
    message_filters::Cache<sensor_msgs::Imu> cache;
    ros::Duration time_desync_imu_radar;

    // third thread
    ros::Timer visualization_timer;

    // fourth thread
    ros::Timer global_fuser_timer;

    //fifth thread
    ros::Timer search_loop_closure_timer;

    //sixth thread
    ros::Timer raytracing_timer;

    // publishers
    ros::Publisher _cluster_pub;
		ros::Publisher _ndt_pub;
		ros::Publisher _map_pub;
		ros::Publisher _odom_pub;
		ros::Publisher _aligned_ndt_pub;
		ros::Publisher _trajectory_pub;

    // tf handling
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // local and global fuser
    LocalFuser _local_fuser;
    GlobalFuser _global_fuser;

    // incoming messages
    nav_msgs::Odometry _odom_message;
    nav_msgs::Odometry _last_used_odom_message;
    sensor_msgs::Imu _imu_msg;
    sensor_msgs::Imu _last_used_imu_msg;
    std_msgs::Header _last_header;
    NDTSlamParameters parameters_;

    // callbacks
    void radarCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imuCb(const sensor_msgs::ImuConstPtr& msg);
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void raytrace(const ros::TimerEvent&);
    void visualizeMap(const ros::TimerEvent&);
    void optimizePoseGraph(const ros::TimerEvent&);
    void searchLoopClosure(const ros::TimerEvent&);

    void createVisualizationMsg(const Map& map, const std_msgs::Header& header, ndt_msgs::NormalDistributions& ndt);
    void readParameters();

    Map map_;
		Map previous_map_;

	  geometry_msgs::TransformStamped init_transform_tf;
    Eigen::Affine2f current_transform;
    geometry_msgs::TransformStamped init_radar_baselink_transform, init_imu_baselink_transform;
    Eigen::Affine3f initial_transform_radar_baselink;
		tf2_ros::TransformBroadcaster tfb;

    ros::Time _first_start_time;

    std::map<int, Pose> nodes_;
    std::map<int, std_msgs::Header> stamps_;
    std::mutex nodes_mutex_, ogm_mutex_;
    std::vector<Constraint> edges_;
    std::map<int, int> submap_idzs_;
};
}
}
}

#endif