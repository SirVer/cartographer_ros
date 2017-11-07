include "map_builder.lua"
include "trajectory_builder.lua" 

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 4,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.05,
  submap_publish_period_sec = 0.4,
  pose_publish_period_sec = 0.04,
  trajectory_publish_period_sec = 0.04,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true -- as opposed to 3d
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 60 -- going to expect 10x4 per second

TRAJECTORY_BUILDER_2D.use_imu_data = false -- default true
TRAJECTORY_BUILDER_2D.scans_per_accumulation = 4
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 30.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100 -- size of the submap in m (we think)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2. -- these three weights are relative to each other
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.5 -- "ceres scan matcher" is used when "online" (aka, "real_time") scan matcher is disabled
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1. -- how much should the IMU rotational velocities matter relative to the laser endpoints? our IMU in simulation is lousy
-- enable online matcher and disable use_odometry to see what the map is supposed to look like
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- this uses the global aligner for the local submap building (also called "realtime" below)

return options
