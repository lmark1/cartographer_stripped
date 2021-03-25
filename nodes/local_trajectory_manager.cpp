#include <cartographer_stripped/plugins/trajectory_builder_interface.h>
#include <uav_ros_lib/param_util.hpp>

#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

using iBuilder_t = cartographer_stripped::mapping::trajectory_builder_interface;

namespace cartographer_stripped {


/**
 * @brief Local Trajectory Builder nodelet.
 *
 */
class LocalTrajectoryManager : public nodelet::Nodelet {
 public:
  void onInit() override;

 private:
  // Check if nodelet is initialized
  bool m_is_initialized = false;

  // Trajectory builder class loader
  std::unique_ptr<pluginlib::ClassLoader<iBuilder_t>> m_trajectory_builder_loader;
  boost::shared_ptr<iBuilder_t>                       m_trajectory_builder_ptr;
};

void LocalTrajectoryManager::onInit() {
  ROS_INFO("[LocalTrajectoryManager] onInit()!");

  // Load Private parameters
  auto& nh_private = getMTPrivateNodeHandle();
  auto& nh         = getMTNodeHandle();

  ros::Time::waitForValid();

  // Initialize the trajectory loader
  m_trajectory_builder_loader = std::make_unique<pluginlib::ClassLoader<iBuilder_t>>(
      "cartographer_stripped",
      "cartographer_stripped::mapping::trajectory_builder_interface");

  // Get the builder adress
  std::string builder_adress;
  param_util::getParamOrThrow(nh_private, "trajectory_builder_adress", builder_adress);

  // Load the plugin
  try {
    m_trajectory_builder_ptr =
        m_trajectory_builder_loader->createInstance(builder_adress);
  } catch (pluginlib::CreateClassException& ex1) {
    ROS_ERROR("[LocalTrajectoryManager]: CreateClassException '%s'",
              builder_adress.c_str());
    ROS_ERROR("[LocalTrajectoryManager]: Error: %s", ex1.what());
    ros::shutdown();
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("[LocalTrajectoryManager]: PluginlibException '%s'",
              builder_adress.c_str());
    ROS_ERROR("[LocalTrajectoryManager]: Error: %s", ex.what());
    ros::shutdown();
  }

  // Initialize the plugin
  auto [success, message] = m_trajectory_builder_ptr->initialize(nh, nh_private);
  if (!success) {
    ROS_FATAL(message.c_str());
    ROS_FATAL("[LocalTrajectoryManager] Failed to load the %s plugin.",
              builder_adress.c_str());
    ros::shutdown();
  }

  m_is_initialized = true;
}

}  // namespace cartographer_stripped

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cartographer_stripped::LocalTrajectoryManager, nodelet::Nodelet)