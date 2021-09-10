include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  range_data_inserter = {
          insert_free_space = true,
          hit_probability = 0.65,
          miss_probability = 0.35,
          },
          map_resolution = 0.05
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 5

-- INPUT
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 20.0
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 20.0


-- LOCAL SLAM

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20

--------------CeresScanMatcher
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40

--------------RealTimeCorrelativeScanMatcher
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1


-- Global SLAM
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 15.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30)
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
POSE_GRAPH.constraint_builder.sampling_ratio = 0.01

POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options