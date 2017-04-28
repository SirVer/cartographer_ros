-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License. 
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  map_frame = "map",
  tracking_frame = "laser_link",
  published_frame = "laser_link",
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

TRAJECTORY_BUILDER_2D.laser_min_range = 0.3
TRAJECTORY_BUILDER_2D.laser_max_range = 100.
-- TRAJECTORY_BUILDER_2D.laser_missing_echo_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.
TRAJECTORY_BUILDER_2D.submaps.resolution = 0.1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150

SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.6
SPARSE_POSE_GRAPH.optimize_every_n_scans = 100
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 15
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30)

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 200
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e-5
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e-5

return options
