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

#include "cartographer_stripped/common/msg_conversion.h"

#include <cmath>

#include "cartographer_stripped/common/math.h"
#include "cartographer_stripped/common/port.h"
#include "cartographer_stripped/common/time.h"
#include "cartographer_stripped/common/time_conversion.h"
#include "cartographer_stripped/sensor/point_cloud.h"
#include "cartographer_stripped/transform/transform.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "glog/logging.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/point_cloud_conversion.h"

namespace {

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
struct PointXYZT {
  float x;
  float y;
  float z;
  float time;
};

struct PointXYZIT {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  float unused_padding[2];
};

}  // namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZT, (float, x, x)(float, y, y)(float, z, z)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, time, time))

namespace cartographer_stripped {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using sensor::PointCloudWithIntensities;
using transform::Rigid3d;

sensor_msgs::PointCloud2 PreparePointCloud2Message(const int64_t timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ToRos(common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}

// For sensor_msgs::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

// For sensor_msgs::MultiEchoLaserScan.
bool HasEcho(const sensor_msgs::LaserEcho& echo) {
  return !echo.echoes.empty();
}

float GetFirstEcho(const sensor_msgs::LaserEcho& echo) {
  return echo.echoes[0];
}

// For sensor_msgs::LaserScan and sensor_msgs::MultiEchoLaserScan.
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const sensor::TimedRangefinderPoint point{
            rotation * (first_echo * Eigen::Vector3f::UnitX()),
            i * msg.time_increment};
        point_cloud.points.push_back(point);
        if (msg.intensities.size() > 0) {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          const auto& echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }
  common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

bool PointCloud2HasField(const sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name) {
  for (const auto& field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const sensor::TimedPointCloud& point_cloud) {
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (const sensor::TimedRangefinderPoint& point : point_cloud) {
    stream.next(point.position.x());
    stream.next(point.position.y());
    stream.next(point.position.z());
    stream.next(kPointCloudComponentFourMagic);
  }
  return msg;
}

std::tuple<PointCloudWithIntensities, common::Time> ToPointCloudWithIntensities(
    const sensor_msgs::LaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<PointCloudWithIntensities, common::Time> ToPointCloudWithIntensities(
    const sensor_msgs::MultiEchoLaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

sensor_msgs::PointCloud2 CreateCloudFromHybridGrid(
    const cartographer_stripped::mapping::HybridGrid& hybrid_grid,
    double min_probability,
    Eigen::Transform<float, 3, Eigen::Affine> transform) {
  double resolution = hybrid_grid.resolution();
  sensor_msgs::PointCloud2 cloud;
  cloud.height = 1;  //"unstructured" point cloud
  cloud.width = 0;

  for (const auto& point : hybrid_grid) {
    if (point.second > 32767.0 * min_probability) {
      cloud.width++;
    }
  }

  cloud.is_dense = true;
  cloud.is_bigendian = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                1, sensor_msgs::PointField::FLOAT32,
                                "intensity", 1,
                                sensor_msgs::PointField::FLOAT32);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

  // for (int i = 0; i < hybrid_grid.values_size(); i++) {
  //   int value = hybrid_grid.values(i);
  //   if (value > 32767 * min_probability) {
  //     int x, y, z;
  //     x = hybrid_grid.x_indices(i);
  //     y = hybrid_grid.y_indices(i);
  //     z = hybrid_grid.z_indices(i);
  //     // transform the cell indices to an actual voxel center point
  //     Eigen::Vector3f point =
  //         transform * Eigen::Vector3f(x * resolution + resolution / 2,
  //                                     y * resolution + resolution / 2,
  //                                     z * resolution + resolution / 2);
  //     *iter_x = point.x();
  //     *iter_y = point.y();
  //     *iter_z = point.z();
  //     int prob_int = hybrid_grid.values(i);
  //     *iter_intensity = prob_int / 32767.0;  // 2^15
  //     ++iter_x;
  //     ++iter_y;
  //     ++iter_z;
  //     ++iter_intensity;
  //   }
  // }

  for (const auto& point : hybrid_grid) {
    int value = point.second;

    if (value <= 32767 * min_probability) {
      continue;
    }

    auto indices = point.first;
    Eigen::Vector3f cloud_point =
        transform * Eigen::Vector3f(indices.x() * resolution + resolution / 2,
                                    indices.y() * resolution + resolution / 2,
                                    indices.z() * resolution + resolution / 2);
    *iter_x = cloud_point.x();
    *iter_y = cloud_point.y();
    *iter_z = cloud_point.z();
    *iter_intensity = value / 32767.0;  // 2^15
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }

  return cloud;
}

std::tuple<PointCloudWithIntensities, common::Time> ToPointCloudWithIntensities(
    const sensor_msgs::PointCloud2& msg) {
  PointCloudWithIntensities point_cloud;
  // We check for intensity field here to avoid run-time warnings if we pass in
  // a PointCloud2 without intensity.
  if (PointCloud2HasField(msg, "intensity")) {
    if (PointCloud2HasField(msg, "time")) {
      pcl::PointCloud<PointXYZIT> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(point.intensity);
      }
    } else {
      pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
        point_cloud.intensities.push_back(point.intensity);
      }
    }
  } else {
    // If we don't have an intensity field, just copy XYZ and fill in 1.0f.
    if (PointCloud2HasField(msg, "time")) {
      pcl::PointCloud<PointXYZT> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(1.0f);
      }
    } else {
      pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
        point_cloud.intensities.push_back(1.0f);
      }
    }
  }
  common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
      CHECK_LE(point.time, 0.f)
          << "Encountered a point with a larger stamp than "
             "the last point in the cloud.";
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

geometry_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
  geometry_msgs::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = rigid3d.translation().z();
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
  geometry_msgs::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

geometry_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
  geometry_msgs::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(common::DegToRad(latitude));
  const double cos_phi = std::cos(common::DegToRad(latitude));
  const double sin_lambda = std::sin(common::DegToRad(longitude));
  const double cos_lambda = std::cos(common::DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

Rigid3d ComputeLocalFrameFromLatLong(const double latitude,
                                     const double longitude,
                                     const double altitude) {
  const Eigen::Vector3d translation =
      LatLongAltToEcef(latitude, longitude, altitude);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(common::DegToRad(longitude), Eigen::Vector3d::UnitZ()) *
      Eigen::Quaterniond(Eigen::AngleAxisd(
          M_PI * 0.5 - common::DegToRad(latitude), Eigen::Vector3d::UnitY())) *
      Eigen::Quaterniond(
          Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ()));
  return Rigid3d(translation, rotation).inverse();
}

}  // namespace cartographer_stripped
