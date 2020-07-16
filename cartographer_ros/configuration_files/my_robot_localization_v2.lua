include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_pose_extrapolator = on,
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
  rangefinder_sampling_ratio = 0.8,
  odometry_sampling_ratio = 0.6,
  fixed_frame_pose_sampling_ratio = 0.6,
  imu_sampling_ratio = 0.8,
  landmarks_sampling_ratio = 0.6,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- MAP_BUILDER.num_background_threads = 3

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 35.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 40.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 9.80
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.04
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 3
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 50
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range= 15.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads
-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds
-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data
-- TRAJECTORY_BUILDER_2D.submaps.resolution = 0.05


POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
-- POSE_GRAPH.optimization_problem.huber_scale = 100
-- POSE_GRAPH.constraint_builder.min_score = 0.7
-- POSE_GRAPH.max_num_final_iterations = 200
-- POSE_GRAPH.constraint_builder.max_constraint_distance
-- POSE_GRAPH.fast_correlative_scan_matcher.linear_search_window = 4.
-- POSE_GRAPH.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth
POSE_GRAPH.constraint_builder.min_score = 0.7
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher
-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight
-- POSE_GRAPH.matcher_translation_weight
-- POSE_GRAPH.matcher_rotation_weight
-- POSE_GRAPH.optimization_problem.*_weight
-- POSE_GRAPH.optimization_problem.ceres_solver_options
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.global_constraint_search_after_n_seconds = 3000.



return options
