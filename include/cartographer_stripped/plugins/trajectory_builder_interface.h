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
  virtual std::tuple<bool, std::string> initialize(
      ros::NodeHandle& nh, ros::NodeHandle& nh_private) = 0;

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
  virtual void add_pointcloud2_data(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) = 0;

  /**
   * @brief Get the latest constructed PointCloud2 map pointer.
   * 
   * @return sensor_msgs::PointCloud2Ptr 
   */
  virtual sensor_msgs::PointCloud2Ptr get_map() = 0;

 protected:
  trajectory_builder_interface() {
    ROS_INFO("[trajectory_builder_interface] Constructor");
  }
};
}  // namespace mapping
}  // namespace cartographer_stripped

#endif /*TRAJECTORY_BUILDER_INTERFACE_H*/