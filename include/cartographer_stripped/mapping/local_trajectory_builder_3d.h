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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_

#include <chrono>
#include <memory>

#include "cartographer_stripped/common/time.h"
#include "cartographer_stripped/mapping/submap_3d.h"
#include "cartographer_stripped/mapping/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer_stripped/mapping/scan_matching/real_time_correlative_scan_matcher_3d.h"
#include "cartographer_stripped/mapping/motion_filter.h"
#include "cartographer_stripped/mapping/range_data_collator.h"
#include "cartographer_stripped/proto/local_trajectory_builder_options_3d.pb.h"
#include "cartographer_stripped/metrics/family_factory.h"
#include "cartographer_stripped/sensor/imu_data.h"
#include "cartographer_stripped/sensor/voxel_filter.h"
#include "cartographer_stripped/sensor/odometry_data.h"
#include "cartographer_stripped/sensor/range_data.h"
#include "cartographer_stripped/transform/rigid_transform.h"
#include "cartographer_stripped/mapping/pose_extrapolator.h"
#include "cartographer_stripped/mapping/trajectory_node.h"

namespace cartographer_stripped {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
class LocalTrajectoryBuilder3D {
 public:
  struct InsertionResult {
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps;
  };
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  explicit LocalTrajectoryBuilder3D(
      const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder3D();

  LocalTrajectoryBuilder3D(const LocalTrajectoryBuilder3D&) = delete;
  LocalTrajectoryBuilder3D& operator=(const LocalTrajectoryBuilder3D&) = delete;

  void AddImuData(const sensor::ImuData& imu_data);
  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'.  `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time,
      const sensor::RangeData& filtered_range_data_in_tracking,
      const std::optional<common::Duration>& sensor_duration);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& filtered_range_data_in_local,
      const sensor::RangeData& filtered_range_data_in_tracking,
      const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
      const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  // Scan matches using the two point clouds and returns the observed pose, or
  // nullptr on failure.
  std::unique_ptr<transform::Rigid3d> ScanMatch(
      const transform::Rigid3d& pose_prediction,
      const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
      const sensor::PointCloud& high_resolution_point_cloud_in_tracking);

  const mapping::proto::LocalTrajectoryBuilderOptions3D options_;
  mapping::ActiveSubmaps3D active_submaps_;

  mapping::MotionFilter motion_filter_;
  std::unique_ptr<scan_matching::RealTimeCorrelativeScanMatcher3D>
      real_time_correlative_scan_matcher_;
  std::unique_ptr<scan_matching::CeresScanMatcher3D> ceres_scan_matcher_;

  std::unique_ptr<mapping::PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;
  std::optional<std::chrono::steady_clock::time_point> last_wall_time_;

  std::optional<double> last_thread_cpu_time_seconds_;

  RangeDataCollator range_data_collator_;

  std::optional<common::Time> last_sensor_time_;
};

}  // namespace mapping
}  // namespace cartographer_stripped

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_
