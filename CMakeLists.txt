cmake_minimum_required(VERSION 2.8.3)
project(cartographer_stripped)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)
set(BOOST_COMPONENTS iostreams)

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)

# Find LuaGoogle
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
find_package(LuaGoogle REQUIRED)

# googletest
enable_testing()
find_package(GMock REQUIRED)
set(GLOG_LIBRARY glog)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES cartographer_stripped CATKIN_DEPENDS other_catkin_pkg
  # DEPENDS system_lib
)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
                    ${LUA_INCLUDE_DIR})

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
