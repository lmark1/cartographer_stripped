cmake_minimum_required(VERSION 2.8.3)
project(cartographer_stripped)

# NOTE: The following three lines are needed to force compile PCL with -std=c++17 see
# issue https://github.com/PointCloudLibrary/pcl/issues/2686
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS roscpp roslib tf2_ros nodelet pluginlib
                                        uav_ros_lib)
set(BOOST_COMPONENTS iostreams)

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Find LuaGoogle
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
find_package(LuaGoogle REQUIRED)

# googletest
enable_testing()
find_package(GMock REQUIRED)
set(GLOG_LIBRARY glog)

# Generate protobuf
file(GLOB_RECURSE ALL_PROTOS "include/cartographer_stripped/proto/*.proto")

# TODO(cschuet): Move proto compilation to separate function.
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --cpp_out ${PROJECT_BINARY_DIR} -I
            ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM)
endforeach()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES cartographer_stripped CATKIN_DEPENDS other_catkin_pkg
  # DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_BINARY_DIR}/include
  ${PROJECT_BINARY_DIR}
  ${LUA_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})

# NODES
add_executable(
  local_trajectory_builder_3d_node
  src/common/time_conversion.cc src/common/msg_conversion.cc src/common/tf_bridge.cc
  src/common/configuration_file_resolver.cc nodes/local_trajectory_builder_3d_node.cpp)
target_link_libraries(local_trajectory_builder_3d_node local_trajectory_builder_3d
                      ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# NODELETS
add_library(
  local_trajectory_manager
  src/common/time_conversion.cc src/common/msg_conversion.cc src/common/tf_bridge.cc
  src/common/configuration_file_resolver.cc nodes/local_trajectory_manager.cpp)
target_link_libraries(local_trajectory_manager local_trajectory_builder_3d
                      ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(local_trajectory_manager ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
# PLUGINS

add_library(
  local_trajectory_builder_3d_plugin
  src/common/time_conversion.cc src/common/msg_conversion.cc src/common/tf_bridge.cc
  src/common/configuration_file_resolver.cc
  src/plugins/local_trajectory_builder_3d_plugin.cpp)
target_link_libraries(local_trajectory_builder_3d_plugin local_trajectory_builder_3d
                      ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(local_trajectory_builder_3d_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
# LIBRARIES

add_library(
  real_time_correlative_scan_matcher
  src/mapping/scan_matching/real_time_correlative_scan_matcher_3d.cc
  src/mapping/scan_matching/real_time_correlative_scan_matcher_options.cc
  src/common/lua_parameter_dictionary.cc
  src/mapping/probability_values.cc
  src/transform/rigid_transform.cc
  src/sensor/point_cloud.cc
  ${ALL_PROTO_SRCS})

target_link_libraries(
  real_time_correlative_scan_matcher
  ${GTEST_BOTH_LIBRARIES}
  ${GMOCK_BOTH_LIBRARIES}
  ${GLOG_LIBRARY}
  ${PROTOBUF_LIBRARY}
  ${CERES_LIBRARIES}
  ${LUA_LIBRARIES})

add_library(
  ceres_scan_matcher
  src/mapping/scan_matching/ceres_scan_matcher_3d.cc
  src/mapping/scan_matching/ceres_pose.cc
  src/common/lua_parameter_dictionary.cc
  src/common/ceres_solver_options.cc
  src/mapping/probability_values.cc
  src/transform/rigid_transform.cc
  src/sensor/point_cloud.cc
  ${ALL_PROTO_SRCS})
target_link_libraries(
  ceres_scan_matcher
  ${GTEST_BOTH_LIBRARIES}
  ${GMOCK_BOTH_LIBRARIES}
  ${GLOG_LIBRARY}
  ${PROTOBUF_LIBRARY}
  ${CERES_LIBRARIES}
  ${LUA_LIBRARIES})

# local trajectory builder 3D
add_library(
  local_trajectory_builder_3d
  src/common/time.cc
  src/sensor/compressed_point_cloud.cc
  src/sensor/point_cloud.cc
  src/sensor/range_data.cc
  src/mapping/range_data_inserter_3d.cc
  src/common/lua_parameter_dictionary.cc
  src/mapping/probability_values.cc
  src/mapping/submap_3d.cc
  src/mapping/scan_matching/rotational_scan_matcher.cc
  src/sensor/voxel_filter.cc
  src/mapping/motion_filter.cc
  src/mapping/range_data_collator.cc
  src/mapping/imu_tracker.cc
  src/mapping/pose_extrapolator.cc
  src/mapping/local_trajectory_builder_3d.cc
  src/mapping/local_trajectory_builder_options_3d.cc
  src/metrics/counter.cc
  src/metrics/gauge.cc
  src/metrics/histogram.cc
  ${ALL_PROTO_SRCS})

target_link_libraries(
  local_trajectory_builder_3d
  ${GTEST_BOTH_LIBRARIES}
  ${GMOCK_BOTH_LIBRARIES}
  ${GLOG_LIBRARY}
  ${PROTOBUF_LIBRARY}
  ${CERES_LIBRARIES}
  ${LUA_LIBRARIES}
  real_time_correlative_scan_matcher
  ceres_scan_matcher)

# TESTS

# Local Trajectory builder test 3D
add_executable(local_trajectory_builder_3d_test test/local_trajectory_builder_3d_test.cc)

target_link_libraries(local_trajectory_builder_3d_test local_trajectory_builder_3d)

# Real time correlative scan matcher 3D test
add_executable(real_time_correlative_scan_matcher_3d_test
               test/real_time_correlative_scan_matcher_3d_test.cc)

target_link_libraries(real_time_correlative_scan_matcher_3d_test
                      real_time_correlative_scan_matcher)

# Ceres scan matcher 3D test
add_executable(ceres_scan_matcher_3d_test test/ceres_scan_matcher_3d_test.cc)
target_link_libraries(ceres_scan_matcher_3d_test ceres_scan_matcher)

# Hybrid Grid Test
add_executable(hybrid_grid_test test/hybrid_grid_test.cc
                                src/mapping/probability_values.cc)
target_link_libraries(hybrid_grid_test ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES}
                      ${GLOG_LIBRARY})

# Interpolated Grid Test
add_executable(interpolated_grid_test test/interpolated_grid_test.cc
                                      src/mapping/probability_values.cc)
target_link_libraries(interpolated_grid_test ${GTEST_BOTH_LIBRARIES}
                      ${GMOCK_BOTH_LIBRARIES} ${GLOG_LIBRARY})
