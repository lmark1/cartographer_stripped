/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_IMU_DATA_H_
#define CARTOGRAPHER_SENSOR_IMU_DATA_H_

#include "Eigen/Core"
#include "cartographer_stripped/common/time.h"

namespace cartographer_stripped {
namespace sensor {

struct ImuData
{
  common::Time    time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;

  bool               contains_orientation = false;
  Eigen::Quaterniond orientation          = Eigen::Quaterniond::Identity();
};

}// namespace sensor
}// namespace cartographer_stripped

#endif// CARTOGRAPHER_SENSOR_IMU_DATA_H_
