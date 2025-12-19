-- Jarabot Cartographer 2D (ROS2 Humble) - Stable bringup config
-- Assumptions:
--  - /scan is sensor_msgs/LaserScan and header.frame_id == "base_link"  (너가 확인한 값)
--  - Simulator (or robot) provides TF: odom -> base_link  (있으면 use_odometry=true 권장)
--  - No IMU used (use_imu_data=false)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frames (TF)
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",

  -- If your simulator already publishes odom->base_link TF, keep this FALSE.
  -- If not, set provide_odom_frame=true and set use_odometry=false (Cartographer-only mode).
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,

  -- Inputs
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 0,

  -- IMPORTANT: define this key ONLY here (options), exactly once.
  num_subdivisions_per_laser_scan = 1,

  lookup_transform_timeout_sec = 0.2,

  -- Publish periods
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.05,

  -- Sampling ratios
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,

  -- We do NOT use IMU/landmarks in Jarabot sim now.
  imu_sampling_ratio = 0.0,
  landmarks_sampling_ratio = 0.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D settings
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Lidar range (너 시뮬 라이다 최대가 5~6m면 8.0 정도가 안전)
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- 안정성 튜닝(시뮬 학습용)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- Submap size (기본값과 비슷하게 유지)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- Loop closure scores
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

return options
