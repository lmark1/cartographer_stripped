include "trajectory_builder_3d_options.lua"

namespace = os.getenv("UAV_NAMESPACE")
options = {
  trajectory_builder_3d_options = TRAJECTORY_BUILDER_3D_OPTIONS,
  map_frame =  namespace.."/map",
  tracking_frame =  namespace.."/base_link",
  published_frame =  namespace.."/slam_link",
  odom_frame = namespace.."/base_link",
  lidar_frame = namespace.."/velodyne"
}

return options