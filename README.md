# Cartographer Stripped

## Introduction

A stripped down, bare-bones version of the following packages:

* [cartographer_ros](https://github.com/larics/cartographer_ros)
* [cartographer](https://github.com/larics/cartographer)

Used for eductaional puposes.

## TODO

- [x] Learn how Ceres scan matching works
- [x] Learn how to accumulate pointcloud scans
  - Pointcloud Scans are accumulated using LocalTrajectoryBuilder3D
  - Only one submap is active, holding the whole map
  - Still uses PoseExtrapolator under the hood - probably dont want that
- [ ] Adjust CeresScanMatcher parameters not to match along z-axis if there are no visible horizontal features in the current scan
- [ ] Figure out how to load .lua configuration files
- [ ] Build a map using \[Estimated odometry - Pointcloud\] message pairs
  - [ ] Adjust LocalTrajectoryBuilder3D to accept \[Odometry - Pointcloud\] pairs
  - [ ] Remove pose extrapolation from LocalTrajectoryBuilder3D
- [ ] NO global optimizations, loop closing etc. (... maybe some loop closing might be a good idea)
  - [ ] Loop closing and global optimization in sparse environments (?)
- [ ] NO protobuf! (... but actually some protobuf needed for parameters)
- [x] NO absl