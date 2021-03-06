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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer_stripped/common/ceres_solver_options.h"
#include "cartographer_stripped/proto/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer_stripped/common/lua_parameter_dictionary.h"
#include "cartographer_stripped/mapping/hybrid_grid.h"
#include "cartographer_stripped/sensor/point_cloud.h"
#include "cartographer_stripped/transform/rigid_transform.h"

namespace cartographer_stripped {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions3D CreateCeresScanMatcherOptions3D(
  common::LuaParameterDictionary* parameter_dictionary);

using PointCloudAndHybridGridPointers =
  std::pair<const sensor::PointCloud*, const HybridGrid*>;

// This scan matcher uses Ceres to align scans with an existing map.
class CeresScanMatcher3D
{
public:
  explicit CeresScanMatcher3D(const proto::CeresScanMatcherOptions3D& options);

  CeresScanMatcher3D(const CeresScanMatcher3D&) = delete;
  CeresScanMatcher3D& operator=(const CeresScanMatcher3D&) = delete;

  // Aligns 'point_clouds' within the 'hybrid_grids' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(
    const Eigen::Vector3d&                              target_translation,
    const transform::Rigid3d&                           initial_pose_estimate,
    const std::vector<PointCloudAndHybridGridPointers>& point_clouds_and_hybrid_grids,
    transform::Rigid3d*                                 pose_estimate,
    ceres::Solver::Summary*                             summary);

private:
  const proto::CeresScanMatcherOptions3D options_;
  ceres::Solver::Options                 ceres_solver_options_;
};

}// namespace scan_matching
}// namespace mapping
}// namespace cartographer_stripped

#endif// CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_
