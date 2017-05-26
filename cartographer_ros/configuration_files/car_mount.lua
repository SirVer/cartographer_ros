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
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
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

TRAJECTORY_BUILDER_3D.scans_per_accumulation = 1
TRAJECTORY_BUILDER_3D.min_range = 2

NUM_SCANS_PER_SUBMAP = 120

--TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05 -- default 0.10, high res 0.01, mid 0.03
--TRAJECTORY_BUILDER_3D.submaps.low_resolution =  0.23 -- default 0.45, high res 0.15, mid 0.25
-- Long distances need lower resolution like high=0.5 low=2.5
TRAJECTORY_BUILDER_3D.submaps.num_range_data = NUM_SCANS_PER_SUBMAP --dec-/increase number of submaps

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 14.8e3 -- def:2e3
-- 3accumulation h0.03 l0.25: 
-- 3accumulation h0.10 l0.45: 
-- 1accumulation h0.03 l0.25: 
-- 1accumulation h0.10 l0.45: 14.8e3

--TRAJECTORY_BUILDER_3D.kalman_local_trajectory_builder.pose_tracker.imu_gravity_time_constant = 1e13 --def 1e9

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 20

SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 5e2
SPARSE_POSE_GRAPH.optimize_every_n_scans = NUM_SCANS_PER_SUBMAP --increases frequency of loop closure attempts
SPARSE_POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
SPARSE_POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
-- Reuse the coarser 3D voxel filter to speed up the computation of loop closure
-- constraints.
SPARSE_POSE_GRAPH.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.40

--SPARSE_POSE_GRAPH.max_num_final_iterations = 2000 --def 200

-- Crazy search window to force loop closure to work. All other changes are probably not needed.
SPARSE_POSE_GRAPH.constraint_builder.max_constraint_distance = 250.
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 50.
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 10.
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(20.)
--SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.83 --def0.77
SPARSE_POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50

return options
