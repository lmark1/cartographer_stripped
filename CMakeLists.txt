cmake_minimum_required(VERSION 2.8.3)
project(cartographer_stripped)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)
set(BOOST_COMPONENTS iostreams)

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)

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
  message(WARNING ${ABS_FIL})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  message(WARNING ${ALL_PROTO_HDRS})
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
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
                    ${PROJECT_BINARY_DIR}/include ${PROJECT_BINARY_DIR} ${LUA_INCLUDE_DIR})

add_library(
  ceres_scan_matcher_3d src/mapping/scan_matching/ceres_scan_matcher_3d.cc
                        src/mapping/scan_matching/ceres_pose.cc ${ALL_PROTO_SRCS})
target_link_libraries(ceres_scan_matcher_3d ${GTEST_BOTH_LIBRARIES}
                      ${GMOCK_BOTH_LIBRARIES} ${GLOG_LIBRARY})
# TESTS

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
