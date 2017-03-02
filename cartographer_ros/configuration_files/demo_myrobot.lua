include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry_data = true,
  use_constant_odometry_variance = false,
  constant_odometry_translational_variance = 0.,
  constant_odometry_rotational_variance = 0.,
  use_horizontal_laser = true,
  use_horizontal_multi_echo_laser = false,
  horizontal_laser_min_range = 0.00999999977648,
  horizontal_laser_max_range = 41.,
  horizontal_laser_missing_echo_ray_length = 1.,
  num_lasers_3d = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = false

--SPARSE_POSE_GRAPH.constraint_builder.max_constraint_distance = 30.



return options
