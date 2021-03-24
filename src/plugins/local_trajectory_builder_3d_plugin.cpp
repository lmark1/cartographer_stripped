#include <cartographer_stripped/plugins/trajectory_builder_interface.h>

namespace cartographer_stripped {
namespace mapping {
class LocalTrajectoryBuilder3DPlugin : public trajectory_builder_interface {
 public:
  LocalTrajectoryBuilder3DPlugin();
  std::tuple<bool, std::string> initialize(
      ros::NodeHandle& nh, ros::NodeHandle& nh_private) override;
  void add_imu_data(sensor_msgs::ImuConstPtr& imu_msg) override;
  void add_odometry_data(nav_msgs::OdometryConstPtr& odom_msg) override;
  void add_pointcloud2_data(
      sensor_msgs::PointCloud2ConstPtr& cloud_msg) override;
  sensor_msgs::PointCloud2Ptr get_map() override;

 private:
  bool m_is_initialize = false;
};

LocalTrajectoryBuilder3DPlugin::LocalTrajectoryBuilder3DPlugin() {
  ROS_INFO("[LocalTrajectoryBuilder3DPlugin] Constructor");
}

std::tuple<bool, std::string> LocalTrajectoryBuilder3DPlugin::initialize(
    ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  ROS_INFO("[LocalTrajectoryBuilder3DPlugin] Initialize");
  m_is_initialize = true;
  return {true, "LocalTrajectoryBuilder3DPlugin initialized succesfully."};
}

void LocalTrajectoryBuilder3DPlugin::add_imu_data(
    sensor_msgs::ImuConstPtr& imu_msg) {
  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] add_imu_data");
}

void LocalTrajectoryBuilder3DPlugin::add_odometry_data(
    nav_msgs::OdometryConstPtr& odom_msg) {
  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] add_odometry_data");
}

void LocalTrajectoryBuilder3DPlugin::add_pointcloud2_data(
    sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  ROS_INFO_THROTTLE(5.0,
                    "[LocalTrajectoryBuilder3DPlugin] add_pointcloud2_data");
}

sensor_msgs::PointCloud2Ptr LocalTrajectoryBuilder3DPlugin::get_map() {
  ROS_INFO_THROTTLE(5.0, "[LocalTrajectoryBuilder3DPlugin] get_map");
}
}  // namespace mapping
}  // namespace cartographer_stripped

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cartographer_stripped::mapping::LocalTrajectoryBuilder3DPlugin,
                       cartographer_stripped::mapping::trajectory_builder_interface)