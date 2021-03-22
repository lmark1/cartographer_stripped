#include <cartographer_stripped/common/lua_parameter_dictionary_test_helpers.h>
#include <cartographer_stripped/common/msg_conversion.h>
#include <cartographer_stripped/common/tf_bridge.h>
#include <cartographer_stripped/common/time_conversion.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_3d.h>
#include <cartographer_stripped/mapping/local_trajectory_builder_options_3d.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

std::unique_ptr<cartographer_stripped::mapping::LocalTrajectoryBuilder3D>
    trajectory_builder_ptr;
std::unique_ptr<cartographer_stripped::TfBridge> tf_bridge_ptr;

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

cartographer_stripped::mapping::proto::LocalTrajectoryBuilderOptions3D
CreateTrajectoryBuilderOptions3D() {
  auto parameter_dictionary =
      cartographer_stripped::common::MakeDictionary(R"text(
        return {
          min_range = 0.5,
          max_range = 50.,
          num_accumulated_range_data = 1,
          voxel_filter_size = 0.2,

          high_resolution_adaptive_voxel_filter = {
            max_length = 0.7,
            min_num_points = 200,
            max_range = 50.,
          },

          low_resolution_adaptive_voxel_filter = {
            max_length = 0.7,
            min_num_points = 200,
            max_range = 50.,
          },

          use_online_correlative_scan_matching = false,
          real_time_correlative_scan_matcher = {
            linear_search_window = 0.2,
            angular_search_window = math.rad(1.),
            translation_delta_cost_weight = 1e-1,
            rotation_delta_cost_weight = 1.,
          },

          ceres_scan_matcher = {
            occupied_space_weight_0 = 5.,
            occupied_space_weight_1 = 20.,
            translation_weight = 0.1,
            translation_weight_z = 0.1,
            rotation_weight = 0.3,
            only_optimize_yaw = false,
            ceres_solver_options = {
              use_nonmonotonic_steps = true,
              max_num_iterations = 20,
              num_threads = 1,
            },
          },

          motion_filter = {
            max_time_seconds = 0.2,
            max_distance_meters = 0.02,
            max_angle_radians = 0.001,
          },

          imu_gravity_time_constant = 1.,
          rotational_histogram_size = 120,

          submaps = {
            high_resolution = 0.2,
            high_resolution_max_range = 50.,
            low_resolution = 0.5,
            num_range_data = 45000,
            range_data_inserter = {
              hit_probability = 0.7,
              miss_probability = 0.4,
              num_free_space_voxels = 0,
            },
          },
        }
        )text");
  return cartographer_stripped::mapping::CreateLocalTrajectoryBuilderOptions3D(
      parameter_dictionary.get());
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

  const auto sensor_to_tracking = tf_bridge_ptr->LookupToTracking(
      time, "red/velodyne");
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_ptr->AddRangeData(
        "lidar",
        cartographer_stripped::sensor::TimedPointCloudData{
            time, sensor_to_tracking->translation().cast<float>(),
            cartographer_stripped::sensor::TransformTimedPointCloud(
                point_cloud.points, sensor_to_tracking->cast<float>())});
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_trajectory_builder_node");

  trajectory_builder_ptr = std::make_unique<
      cartographer_stripped::mapping::LocalTrajectoryBuilder3D>(
      CreateTrajectoryBuilderOptions3D(), std::vector<std::string>{"lidar"});

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf(tf2_buffer);
  tf_bridge_ptr = std::make_unique<cartographer_stripped::TfBridge>(
      "red/base_link", 0, &tf2_buffer);

  ros::NodeHandle nh;
  auto imu_sub = nh.subscribe("imu", 1, &imu_callback);
  auto odom_sub = nh.subscribe("odometry", 1, &odom_callback);
  auto pointcloud_sub = nh.subscribe("pointcloud", 1, &pointcloud_callback);

  ros::spin();
  return 0;
}