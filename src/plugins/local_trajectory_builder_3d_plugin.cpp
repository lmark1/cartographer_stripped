#include <cartographer_stripped/common/configuration_file_resolver.h>
#include <cartographer_stripped/common/lua_parameter_dictionary_test_helpers.h>
#include <cartographer_stripped/common/msg_conversion.h>
#include <cartographer_stripped/common/tf_bridge.h>
#include <cartographer_stripped/common/time_conversion.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_3d.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_options_3d.h>
#include <cartographer_stripped/plugins/trajectory_builder_interface.h>
#include <uav_ros_lib/param_util.hpp>

#include <tf2_ros/transform_listener.h>
#include <mutex>

namespace cartographer_stripped {
namespace mapping {
class LocalTrajectoryBuilder3DPlugin : public trajectory_builder_interface {
 public:
  LocalTrajectoryBuilder3DPlugin();
  std::tuple<bool, std::string> initialize(ros::NodeHandle& nh,
                                           ros::NodeHandle& nh_private) override;

  void add_imu_data(const sensor_msgs::ImuConstPtr& imu_msg) override;
  void add_odometry_data(const nav_msgs::OdometryConstPtr& odom_msg) override;
  void add_pointcloud2_data(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) override;
  geometry_msgs::Transform    get_tracking_frame_transform() override;
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
  bool m_is_initialized = false;

  // Configuration parameters
  std::string m_configuration_basename  = "default_config.lua";
  std::string m_configuration_directory = "path/to/config";

  // Trajectory builder resources
  std::unique_ptr<LocalTrajectoryBuilder3D> m_trajectory_builder_ptr;
  std::mutex                                m_trajectory_builder_mutex;

  // TF bridge resources
  std::unique_ptr<TfBridge>  m_tf_bridge_ptr;
  tf2_ros::Buffer            m_tf2_buffer;
  tf2_ros::TransformListener m_tf2_listener;
};

LocalTrajectoryBuilder3DPlugin::LocalTrajectoryBuilder3DPlugin()
    : m_tf2_listener(m_tf2_buffer) {
  ROS_INFO("[LocalTrajectoryBuilder3DPlugin] Constructor");
}

std::tuple<bool, std::string> LocalTrajectoryBuilder3DPlugin::initialize(
    ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  ROS_INFO("[LocalTrajectoryBuilder3DPlugin] Initialize");
  param_util::getParamOrThrow(nh_private, "configuration_basename",
                              m_configuration_basename);
  param_util::getParamOrThrow(nh_private, "configuration_directory",
                              m_configuration_directory);

  m_trajectory_builder_ptr =
      std::make_unique<cartographer_stripped::mapping::LocalTrajectoryBuilder3D>(
          create_trajectory_builder_options(m_configuration_directory,
                                            m_configuration_basename),
          std::vector<std::string>{"lidar"});
  m_tf_bridge_ptr = std::make_unique<cartographer_stripped::TfBridge>(m_tracking_frame,
                                                                      0.1, &m_tf2_buffer);

  m_is_initialized = true;
  return {true, "LocalTrajectoryBuilder3DPlugin initialized succesfully."};
}

void LocalTrajectoryBuilder3DPlugin::add_imu_data(const sensor_msgs::ImuConstPtr& msg) {
  if (!m_is_initialized) {
    return;
  }

  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] add_imu_data");
  {
    std::scoped_lock lock(m_trajectory_builder_mutex);
    m_trajectory_builder_ptr->AddImuData(::cartographer_stripped::ToImuData(msg));
  }
}

void LocalTrajectoryBuilder3DPlugin::add_odometry_data(
    const nav_msgs::OdometryConstPtr& msg) {
  if (!m_is_initialized) {
    return;
  }

  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] add_odometry_data");
  {
    std::scoped_lock lock(m_trajectory_builder_mutex);
    m_trajectory_builder_ptr->AddOdometryData(
        ::cartographer_stripped::ToOdometryData(msg), true);
  }
}

void LocalTrajectoryBuilder3DPlugin::add_pointcloud2_data(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (!m_is_initialized) {
    return;
  }

  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] add_pointcloud2_data");
  auto [point_cloud, time] = ::cartographer_stripped::ToPointCloudWithIntensities(*msg);

  const auto sensor_to_tracking = m_tf_bridge_ptr->LookupToTracking(time, m_lidar_frame);
  if (sensor_to_tracking != nullptr) {
    std::scoped_lock lock(m_trajectory_builder_mutex);
    m_trajectory_builder_ptr->AddRangeData(
        "lidar", ::cartographer_stripped::sensor::TimedPointCloudData{
                     time, sensor_to_tracking->translation().cast<float>(),
                     ::cartographer_stripped::sensor::TransformTimedPointCloud(
                         point_cloud.points, sensor_to_tracking->cast<float>())});
  }
}

sensor_msgs::PointCloud2Ptr LocalTrajectoryBuilder3DPlugin::get_map() {
  if (!m_is_initialized) {
    return boost::make_shared<sensor_msgs::PointCloud2>();
  }

  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] get_map");

  std::shared_ptr<const Submap3D> active_submap;
  {
    std::scoped_lock lock(m_trajectory_builder_mutex);
    if (m_trajectory_builder_ptr->GetActiveSubmaps().submaps().empty()) {
      return boost::make_shared<sensor_msgs::PointCloud2>();
    }

    active_submap = m_trajectory_builder_ptr->GetActiveSubmaps().submaps().front();
  }

  const auto& high_res_grid = active_submap->high_resolution_hybrid_grid();
  Eigen::Transform<float, 3, Eigen::Affine> transform =
      Eigen::Translation3f(active_submap->local_pose().translation().x(),
                           active_submap->local_pose().translation().y(),
                           active_submap->local_pose().translation().z()) *
      Eigen::Quaternion<float>(active_submap->local_pose().rotation().w(),
                               active_submap->local_pose().rotation().x(),
                               active_submap->local_pose().rotation().y(),
                               active_submap->local_pose().rotation().z());

  auto cloud =
      ::cartographer_stripped::CreateCloudFromHybridGrid(high_res_grid, 0.7, transform);
  cloud.header.frame_id = m_map_frame;
  cloud.header.stamp    = ros::Time::now();
  return boost::make_shared<sensor_msgs::PointCloud2>(std::move(cloud));
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

geometry_msgs::Transform LocalTrajectoryBuilder3DPlugin::get_tracking_frame_transform() {
  transform::Rigid3d pose;

  {
    std::scoped_lock lock(m_trajectory_builder_mutex);
    pose = m_trajectory_builder_ptr->GetLatestPose();
  }

  geometry_msgs::Transform tf_pose;
  tf_pose.translation.x = pose.translation().x();
  tf_pose.translation.y = pose.translation().y();
  tf_pose.translation.z = pose.translation().z();
  tf_pose.rotation.x    = pose.rotation().x();
  tf_pose.rotation.y    = pose.rotation().y();
  tf_pose.rotation.z    = pose.rotation().z();
  tf_pose.rotation.w    = pose.rotation().w();
  return tf_pose;
}

}  // namespace mapping
}  // namespace cartographer_stripped

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cartographer_stripped::mapping::LocalTrajectoryBuilder3DPlugin,
                       cartographer_stripped::mapping::trajectory_builder_interface)