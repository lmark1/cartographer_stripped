#include <cartographer_stripped/common/configuration_file_resolver.h>
#include <cartographer_stripped/common/lua_parameter_dictionary_test_helpers.h>
#include <cartographer_stripped/common/msg_conversion.h>
#include <cartographer_stripped/common/tf_bridge.h>
#include <cartographer_stripped/common/time_conversion.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_3d.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_options_3d.h>
#include <uav_ros_lib/param_util.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

std::unique_ptr<cartographer_stripped::mapping::LocalTrajectoryBuilder3D>
    trajectory_builder_ptr;
std::unique_ptr<cartographer_stripped::TfBridge> tf_bridge_ptr;
std::string tracking_frame{"tracking_frame"};
std::string map_frame{"map_frame"};
std::string published_frame{"published_frame"};
std::string odom_frame{"odom_frame"};
std::string lidar_frame{"lidar_frame"};

cartographer_stripped::mapping::proto::LocalTrajectoryBuilderOptions3D
CreateTrajectoryBuilderOptions3D(const std::string& configuration_directory,
                                 const std::string& configuration_basename) {
  auto file_resolver = std::make_unique<
      cartographer_stripped::common::ConfigurationFileResolver>(
      std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  cartographer_stripped::common::LuaParameterDictionary
      lua_parameter_dictionary(code, std::move(file_resolver));

  tracking_frame = lua_parameter_dictionary.GetString("tracking_frame");
  map_frame = lua_parameter_dictionary.GetString("map_frame");
  published_frame = lua_parameter_dictionary.GetString("published_frame");
  odom_frame = lua_parameter_dictionary.GetString("odom_frame");
  lidar_frame = lua_parameter_dictionary.GetString("lidar_frame");

  return cartographer_stripped::mapping::CreateLocalTrajectoryBuilderOptions3D(
      lua_parameter_dictionary.GetDictionary("trajectory_builder_3d_options")
          .get());
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg) {
  if (!trajectory_builder_ptr) {
    ROS_WARN_THROTTLE(
        1.0,
        "[local_trajectory_builder_node] Trajectory builder not initialized.");
    return;
  }

  trajectory_builder_ptr->AddImuData(
      {cartographer_stripped::FromRos(msg->header.stamp),
       Eigen::Vector3d{msg->linear_acceleration.x, msg->linear_acceleration.y,
                       msg->linear_acceleration.z},
       Eigen::Vector3d{msg->angular_velocity.x, msg->angular_velocity.y,
                       msg->angular_velocity.z}});
}

void odom_callback(const nav_msgs::OdometryConstPtr& msg) {
  if (!trajectory_builder_ptr) {
    ROS_WARN_THROTTLE(
        1.0,
        "[local_trajectory_builder_node] Trajectory builder not initialized.");
    return;
  }

  trajectory_builder_ptr->AddOdometryData(
      {cartographer_stripped::FromRos((msg->header.stamp)),
       cartographer_stripped::transform::Rigid3d(
           cartographer_stripped::transform::Rigid3d::Vector{
               msg->pose.pose.position.x, msg->pose.pose.position.y,
               msg->pose.pose.position.z},
           cartographer_stripped::transform::Rigid3d::Quaternion{
               msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
               msg->pose.pose.orientation.y, msg->pose.pose.orientation.z})});
}

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (!trajectory_builder_ptr) {
    ROS_WARN_THROTTLE(
        1.0,
        "[local_trajectory_builder_node] Trajectory builder not initialized.");
    return;
  }

  auto [point_cloud, time] =
      cartographer_stripped::ToPointCloudWithIntensities(*msg);

  const auto sensor_to_tracking =
      tf_bridge_ptr->LookupToTracking(time, lidar_frame);
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_ptr->AddRangeData(
        "lidar",
        cartographer_stripped::sensor::TimedPointCloudData{
            time, sensor_to_tracking->translation().cast<float>(),
            cartographer_stripped::sensor::TransformTimedPointCloud(
                point_cloud.points, sensor_to_tracking->cast<float>())});
  }
}

sensor_msgs::PointCloud2Ptr get_submap() {
  if (!trajectory_builder_ptr) {
    ROS_WARN_THROTTLE(
        1.0,
        "[local_trajectory_builder_node] Trajectory builder not initialized.");
    return boost::make_shared<sensor_msgs::PointCloud2>();
  }

  if (trajectory_builder_ptr->GetActiveSubmaps().submaps().empty()) {
    return boost::make_shared<sensor_msgs::PointCloud2>();
  }

  const auto active_submap =
      trajectory_builder_ptr->GetActiveSubmaps().submaps().front();
  const auto& high_res_grid = active_submap->high_resolution_hybrid_grid();

  Eigen::Transform<float, 3, Eigen::Affine> transform =
      Eigen::Translation3f(active_submap->local_pose().translation().x(),
                           active_submap->local_pose().translation().y(),
                           active_submap->local_pose().translation().z()) *
      Eigen::Quaternion<float>(active_submap->local_pose().rotation().w(),
                               active_submap->local_pose().rotation().x(),
                               active_submap->local_pose().rotation().y(),
                               active_submap->local_pose().rotation().z());

  auto cloud = cartographer_stripped::CreateCloudFromHybridGrid(high_res_grid,
                                                                0.7, transform);
  cloud.header.frame_id = map_frame;
  cloud.header.stamp = ros::Time::now();
  return boost::make_shared<sensor_msgs::PointCloud2>(std::move(cloud));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_trajectory_builder_node");

  ros::NodeHandle nh_private("~");
  std::string configuration_basename, configuration_directory;
  param_util::getParamOrThrow(nh_private, "configuration_basename",
                              configuration_basename);
  param_util::getParamOrThrow(nh_private, "configuration_directory",
                              configuration_directory);

  trajectory_builder_ptr = std::make_unique<
      cartographer_stripped::mapping::LocalTrajectoryBuilder3D>(
      CreateTrajectoryBuilderOptions3D(configuration_directory,
                                       configuration_basename),
      std::vector<std::string>{"lidar"});

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf(tf2_buffer);
  tf_bridge_ptr = std::make_unique<cartographer_stripped::TfBridge>(
     tracking_frame, 0, &tf2_buffer);

  ros::NodeHandle nh;
  auto imu_sub = nh.subscribe("imu", 1, &imu_callback);
  auto odom_sub = nh.subscribe("odometry", 1, &odom_callback);
  auto pointcloud_sub = nh.subscribe("pointcloud", 1, &pointcloud_callback);
  auto map_pub = nh.advertise<sensor_msgs::PointCloud2>("submap", 1);
  auto map_timer = nh.createTimer(ros::Duration(ros::Rate(10)),
                                  [&](const ros::TimerEvent& /*unused*/) {
                                    map_pub.publish(*get_submap());
                                  });

  ros::spin();
  return 0;
}