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
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  use_laser_scan = false,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}

TRAJECTORY_BUILDER_3D.scans_per_accumulation = 100

-- No point of trying to SLAM over the points on your car.
TRAJECTORY_BUILDER_3D.laser_min_range = 2.5
TRAJECTORY_BUILDER_3D.laser_max_range = 150

-- These were just my first guess: use more points for SLAMing and adapt a bit for the ranges that are bigger for cars.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 5.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 8.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 400.

-- The submaps felt pretty big - since the car moves faster, we want them to be
-- slightly smaller. You are also slamming at 10cm - which might be aggressive
-- for cars and for the quality of the laser. I increased 'high_resolution',
-- you might need to increase 'low_resolution'. Increasing the
-- '*num_iterations' in the various optimization problems also trades
-- performance/quality.
TRAJECTORY_BUILDER_3D.submaps.num_laser_fans = 70
TRAJECTORY_BUILDER_3D.submaps.high_resolution = .25
TRAJECTORY_BUILDER_3D.submaps.low_resolution = .60
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 30
MAP_BUILDER.sparse_pose_graph.optimization_problem.huber_scale = 5e2

-- Trying loop closing too often will cost CPU and not buy you a lot. There is
-- little point in trying more than once per submap.
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 40
MAP_BUILDER.sparse_pose_graph.constraint_builder.sampling_ratio = 0.03
MAP_BUILDER.sparse_pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 200
-- Reuse the coarser 3D voxel filter to speed up the computation of loop closure
-- constraints.
MAP_BUILDER.sparse_pose_graph.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter

MAP_BUILDER.sparse_pose_graph.constraint_builder.min_score = 0.5

-- Crazy search window to force loop closure to work. All other changes are probably not needed.
MAP_BUILDER.sparse_pose_graph.constraint_builder.max_constraint_distance = 250.
MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 250.
MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 30.
MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(60.)
MAP_BUILDER.sparse_pose_graph.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50

return options
