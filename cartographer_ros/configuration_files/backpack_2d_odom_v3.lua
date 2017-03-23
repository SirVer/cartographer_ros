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
  -- sensor_bridge = {
  --   horizontal_laser_min_range = 0.02,
  --   horizontal_laser_max_range = 4.0,
  --   horizontal_laser_missing_echo_ray_length = 1.,
  --   constant_odometry_translational_variance = 0.1,
  --   constant_odometry_rotational_variance = 0.1,
  -- },
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Trust the constant velocity model more (this is putting more weight on odometry)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 70
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 300

TRAJECTORY_BUILDER_2D.laser_min_range = 0.02
TRAJECTORY_BUILDER_2D.laser_max_range = 4.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- This is sometimes useful for debugging.
-- TRAJECTORY_BUILDER_2D.submaps.laser_fan_inserter.insert_free_space = false
-- TRAJECTORY_BUILDER_2D.submaps.output_debug_images = true

-- Made submaps smaller and since the laser is so good made resolution smaller -> more features.
TRAJECTORY_BUILDER_2D.submaps.num_laser_fans = 35
TRAJECTORY_BUILDER_2D.submaps.resolution = 0.035
SPARSE_POSE_GRAPH.optimize_every_n_scans = 35

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- This was the actual tricky bit: the score histogram's showed that a lot of
-- really bogus loop closing constraint have been put into the graph, likely
-- because of the very similar looking long corridors. And they had high
-- scores. The real loop closure at the end was not caught though. Reducing the
-- search window got rid of some of the wrong constraints, reducing the score
-- made sure the correct loop closure was found too. The map was still terrible,
-- because the wrong loop constraint messed things up.
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.65
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.

-- Reducing this setting gives local SLAM more weight vs loop closure, which
-- was sufficient to get the whole system agree on the correct system.
SPARSE_POSE_GRAPH.constraint_builder.ceres_scan_matcher.covariance_scale = 1e-2
SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
