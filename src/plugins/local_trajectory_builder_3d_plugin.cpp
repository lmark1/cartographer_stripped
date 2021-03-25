#include <cartographer_stripped/common/configuration_file_resolver.h>
#include <cartographer_stripped/common/lua_parameter_dictionary_test_helpers.h>
#include <cartographer_stripped/common/msg_conversion.h>
#include <cartographer_stripped/common/tf_bridge.h>
#include <cartographer_stripped/common/time_conversion.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_3d.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_options_3d.h>
#include <cartographer_stripped/plugins/trajectory_builder_interface.h>
#include <uav_ros_lib/param_util.hpp>

namespace cartographer_stripped {
namespace mapping {
class LocalTrajectoryBuilder3DPlugin : public trajectory_builder_interface {
 public:
  LocalTrajectoryBuilder3DPlugin();
  std::tuple<bool, std::string> initialize(ros::NodeHandle& nh,
                                           ros::NodeHandle& nh_private) override;

  void add_imu_data(sensor_msgs::ImuConstPtr& imu_msg) override;
  void add_odometry_data(nav_msgs::OdometryConstPtr& odom_msg) override;
  void add_pointcloud2_data(sensor_msgs::PointCloud2ConstPtr& cloud_msg) override;

  sensor_msgs::PointCloud2Ptr get_map() override;

 private:
  /**
   * @brief Create a trajectory builder options object
   *
   * @param configuration_directory
   * @param configuration_basename
   * @return
   * cartographer_stripped::mapping::proto::LocalTrajectoryBuilderOptions3D
   */
  cartographer_stripped::mapping::proto::LocalTrajectoryBuilderOptions3D
  create_trajectory_builder_options(const std::string& configuration_directory,
                                    const std::string& configuration_basename);

  // Check if the plugin is initialized
  bool m_is_initialize = false;

  // Default frame name variables
  std::string m_tracking_frame  = "default_tracking_frame";
  std::string m_map_frame       = "default_map_frame";
  std::string m_published_frame = "default_published_frame";
  std::string m_odom_frame      = "default_odom_frame";
  std::string m_lidar_frame     = "default_lidar_frame";

  // Configuration parameters
  std::string m_configuration_basename  = "default_config.lua";
  std::string m_configuration_directory = "path/to/config";

  // Trajectory builder resources
  std::unique_ptr<LocalTrajectoryBuilder3D> m_trajectory_builder_ptr;
  std::unique_ptr<TfBridge>                 m_tf_bridge_ptr;
};

LocalTrajectoryBuilder3DPlugin::LocalTrajectoryBuilder3DPlugin() {
  ROS_INFO("[LocalTrajectoryBuilder3DPlugin] Constructor");
}

std::tuple<bool, std::string> LocalTrajectoryBuilder3DPlugin::initialize(
    ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  ROS_INFO("[LocalTrajectoryBuilder3DPlugin] Initialize");
  param_util::getParamOrThrow(nh_private, "configuration_basename",
                              m_configuration_basename);
  param_util::getParamOrThrow(nh_private, "configuration_directory",
                              m_configuration_directory);
  m_is_initialize = true;
  return {true, "LocalTrajectoryBuilder3DPlugin initialized succesfully."};
}

void LocalTrajectoryBuilder3DPlugin::add_imu_data(sensor_msgs::ImuConstPtr& imu_msg) {
  if (!m_is_initialize) {
    return;
  }

  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] add_imu_data");
}

void LocalTrajectoryBuilder3DPlugin::add_odometry_data(
    nav_msgs::OdometryConstPtr& odom_msg) {
  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] add_odometry_data");
}

void LocalTrajectoryBuilder3DPlugin::add_pointcloud2_data(
    sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] add_pointcloud2_data");
}

sensor_msgs::PointCloud2Ptr LocalTrajectoryBuilder3DPlugin::get_map() {
  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] get_map");
}

cartographer_stripped::mapping::proto::LocalTrajectoryBuilderOptions3D
LocalTrajectoryBuilder3DPlugin::create_trajectory_builder_options(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  auto file_resolver =
      std::make_unique<cartographer_stripped::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
  cartographer_stripped::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  m_tracking_frame  = lua_parameter_dictionary.GetString("tracking_frame");
  m_map_frame       = lua_parameter_dictionary.GetString("map_frame");
  m_published_frame = lua_parameter_dictionary.GetString("published_frame");
  m_odom_frame      = lua_parameter_dictionary.GetString("odom_frame");
  m_lidar_frame     = lua_parameter_dictionary.GetString("lidar_frame");

  return cartographer_stripped::mapping::CreateLocalTrajectoryBuilderOptions3D(
      lua_parameter_dictionary.GetDictionary("trajectory_builder_3d_options").get());
}

}  // namespace mapping
}  // namespace cartographer_stripped

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cartographer_stripped::mapping::LocalTrajectoryBuilder3DPlugin,
                       cartographer_stripped::mapping::trajectory_builder_interface)