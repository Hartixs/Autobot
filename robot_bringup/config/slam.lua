-- Copyright 2016 The Cartographer Authors
-- Licensed under the Apache License, Version 2.0

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- TF Frame Setup (CRITICAL UPDATE FOR ESKF)
  map_frame = "map",
  tracking_frame = "base_footprint", -- Matches the root of your robot model
  published_frame = "odom",          -- Cartographer publishes the correction from map -> odom
  odom_frame = "odom",               -- Cartographer listens to the odom frame
  provide_odom_frame = false,        -- Changed to FALSE: Your ESKF provides odom now, not Cartographer!
  
  publish_frame_projected_to_2d = true,
  use_odometry = true,               -- Changed to TRUE: Listen to your ESKF's /odom topic
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5
TRAJECTORY_BUILDER_2D.use_imu_data = false -- Keep false! Let your ESKF handle the MPU6050 math.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

return options