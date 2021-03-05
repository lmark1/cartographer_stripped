# Cartographer Stripped

## Introduction

A stripped down, bare-bones version of the following packages:

* [cartographer_ros](https://github.com/larics/cartographer_ros)
* [cartographer](https://github.com/larics/cartographer)

Used for eductaional puposes.

## TODO

- [x] Learn how Ceres scan matching works
- [ ] Learn how to accumulate pointcloud scans
- [ ] Build a map using \[Estimated odometry - Pointcloud\] message pairs
- [ ] NO global optimizations, loop closing etc.
- [ ] NO protobuf! (...but actually some protobuf needed for parameters)
- [ ] NO absl