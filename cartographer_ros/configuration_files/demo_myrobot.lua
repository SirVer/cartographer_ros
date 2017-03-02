include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  map_frame = "map",
  tracking_frame = "base_laser",
  published_frame = "base_laser",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 0,
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
