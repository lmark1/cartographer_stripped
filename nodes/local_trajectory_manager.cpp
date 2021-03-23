#include <cartographer_stripped/common/configuration_file_resolver.h>
#include <cartographer_stripped/common/lua_parameter_dictionary_test_helpers.h>
#include <cartographer_stripped/common/msg_conversion.h>
#include <cartographer_stripped/common/tf_bridge.h>
#include <cartographer_stripped/common/time_conversion.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_3d.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_options_3d.h>
#include <uav_ros_lib/param_util.hpp>

#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

namespace cartographer_stripped {

/**
 * @brief Local Trajectory Builder nodelet.
 *
 */
class LocalTrajectoryManager : public nodelet::Nodelet {
 public:
  void onInit() override;

 private:
  // Trajectory builder resources
  std::unique_ptr<mapping::LocalTrajectoryBuilder3D> m_trajectory_builder_ptr;
  std::unique_ptr<TfBridge> m_tf_bridge_ptr;

  // Default frame name variables
  std::string m_tracking_frame{"tracking_frame"};
  std::string m_map_frame{"map_frame"};
  std::string m_published_frame{"published_frame"};
  std::string m_odom_frame{"odom_frame"};
  std::string m_lidar_frame{"lidar_frame"};
};

void LocalTrajectoryManager::onInit() {
  ROS_INFO("[LocalTrajectoryManager] Hello from onInit()!");
}

}  // namespace cartographer_stripped

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cartographer_stripped::LocalTrajectoryManager,
                       nodelet::Nodelet)