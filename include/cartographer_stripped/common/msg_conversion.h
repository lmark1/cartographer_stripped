/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H

#include "cartographer_stripped/common/time.h"
#include "cartographer_stripped/mapping/hybrid_grid.h"
#include "cartographer_stripped/sensor/imu_data.h"
#include "cartographer_stripped/sensor/odometry_data.h"
#include "cartographer_stripped/sensor/point_cloud.h"
#include "cartographer_stripped/transform/rigid_transform.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_stripped {

sensor_msgs::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const cartographer_stripped::sensor::TimedPointCloud& point_cloud);

geometry_msgs::Transform ToGeometryMsgTransform(
    const cartographer_stripped::transform::Rigid3d& rigid3d);

geometry_msgs::Pose ToGeometryMsgPose(
    const cartographer_stripped::transform::Rigid3d& rigid3d);

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

sensor::OdometryData ToOdometryData(const nav_msgs::OdometryConstPtr& msg);

sensor::ImuData ToImuData(const sensor_msgs::ImuConstPtr& msg);

// Converts ROS message to point cloud. Returns the time when the last point
// was acquired (different from the ROS timestamp). Timing of points is given in
// the fourth component of each point relative to `Time`.
std::tuple<cartographer_stripped::sensor::PointCloudWithIntensities,
           cartographer_stripped::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::LaserScan& msg);

std::tuple<cartographer_stripped::sensor::PointCloudWithIntensities,
           cartographer_stripped::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::MultiEchoLaserScan& msg);

std::tuple<cartographer_stripped::sensor::PointCloudWithIntensities,
           cartographer_stripped::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::PointCloud2& msg);

cartographer_stripped::transform::Rigid3d ToRigid3d(
    const geometry_msgs::TransformStamped& transform);

cartographer_stripped::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);

// Converts from WGS84 (latitude, longitude, altitude) to ECEF.
Eigen::Vector3d LatLongAltToEcef(double latitude, double longitude, double altitude);

// Returns a transform that takes ECEF coordinates from nearby points to a local
// frame that has z pointing upwards.
cartographer_stripped::transform::Rigid3d ComputeLocalFrameFromLatLong(double latitude,
                                                                       double longitude,
                                                                       double altitude);

// Converts from a HybridGrid to a sensor_msgs::Pointcloud2
sensor_msgs::PointCloud2 CreateCloudFromHybridGrid(
    const mapping::HybridGrid& hybrid_grid, double min_probability,
    Eigen::Transform<float, 3, Eigen::Affine> transform);

}  // namespace cartographer_stripped

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H
