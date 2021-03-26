#ifndef TRAJECTORY_BUILDER_INTERFACE_H
#define TRAJECTORY_BUILDER_INTERFACE_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

namespace cartographer_stripped {
namespace mapping {
class trajectory_builder_interface {
 public:
  /**
   * @brief Initialize the Trajectory Builder using the given ROS node handles.
   *
   * @param nh A Public ROS Node Handle.
   * @param nh_private A Private ROS Handle.
   * @return std::tuple<bool, std::string> Return success flag and message
   * string.
   */
  virtual std::tuple<bool, std::string> initialize(ros::NodeHandle& nh,
                                                   ros::NodeHandle& nh_private) = 0;

  /**
   * @brief Add a new IMU message to the Trajectory Builder.
   *
   * @param imu_msg
   */
  virtual void add_imu_data(const sensor_msgs::ImuConstPtr& imu_msg) = 0;

  /**
   * @brief Add a new Odometry message to the Trajectory Builder.
   *
   * @param odom_msg
   */
  virtual void add_odometry_data(const nav_msgs::OdometryConstPtr& odom_msg) = 0;

  /**
   * @brief Add a new PointCloud2 message to the Trajectory Builder;
   *
   * @param cloud_msg
   */
  virtual void add_pointcloud2_data(
      const sensor_msgs::PointCloud2ConstPtr& cloud_msg) = 0;

  /**
   * @brief Get the latest constructed PointCloud2 map pointer.
   *
   * @return sensor_msgs::PointCloud2Ptr
   */
  virtual sensor_msgs::PointCloud2Ptr get_map() = 0;

  /**
   * @brief Get the tracking frame object
   * 
   * @return const std::string& 
   */
  inline const std::string& get_tracking_frame() { return m_tracking_frame; }

  /**
   * @brief Get the map frame object
   * 
   * @return const std::string& 
   */
  inline const std::string& get_map_frame() { return m_map_frame; }

  /**
   * @brief Get the published frame object
   * 
   * @return const std::string& 
   */
  inline const std::string& get_published_frame() { return m_published_frame; }

  /**
   * @brief Get the odom frame object
   * 
   * @return const std::string& 
   */
  inline const std::string& get_odom_frame() { return m_odom_frame; }

  /**
   * @brief Get the lidar frame object
   * 
   * @return const std::string& 
   */
  inline const std::string& get_lidar_frame() { return m_lidar_frame; }

 protected:
  trajectory_builder_interface() {
    ROS_INFO("[trajectory_builder_interface] Constructor");
  }

  // Default frame name variables
  std::string m_tracking_frame  = "default_tracking_frame";
  std::string m_map_frame       = "default_map_frame";
  std::string m_published_frame = "default_published_frame";
  std::string m_odom_frame      = "default_odom_frame";
  std::string m_lidar_frame     = "default_lidar_frame";
};
}  // namespace mapping
}  // namespace cartographer_stripped

#endif /*TRAJECTORY_BUILDER_INTERFACE_H*/