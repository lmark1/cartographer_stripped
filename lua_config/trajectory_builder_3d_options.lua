TRAJECTORY_BUILDER_3D_OPTIONS = {
  min_range = 0.5,
  max_range = 50.,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.2,

  high_resolution_adaptive_voxel_filter = {
    max_length = 0.7,
    min_num_points = 200,
    max_range = 50.,
  },

  low_resolution_adaptive_voxel_filter = {
    max_length = 0.7,
    min_num_points = 200,
    max_range = 50.,
  },

  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.2,
    angular_search_window = math.rad(1.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1.,
  },

  ceres_scan_matcher = {
    occupied_space_weight_0 = 5.,
    occupied_space_weight_1 = 20.,
    translation_weight = 0.1,
    translation_weight_z = 0.1,
    rotation_weight = 0.3,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 0.2,
    max_distance_meters = 0.02,
    max_angle_radians = 0.001,
  },

  imu_gravity_time_constant = 1.,
  rotational_histogram_size = 120,

  submaps = {
    high_resolution = 0.2,
    high_resolution_max_range = 50.,
    low_resolution = 0.5,
    num_range_data = 45000,
    range_data_inserter = {
      hit_probability = 0.7,
      miss_probability = 0.4,
      num_free_space_voxels = 0,
    },
  },
}