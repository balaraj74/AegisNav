"""Gazebo world + bridges + localization + local controller + AegisNav nodes.

`world:=rescue_building` (default) or `world:=corridor` selects a preset from config/worlds.yaml:
world file, maps, Nav2 parameters, lidar mount, spawn pose, speed and the core scenario name.
Verified in the aegisnav-jazzy container (docs/testing/verification_matrix.md).

Data path: gz DiffDrive odometry/tf -> /odom, /tf; gz lidar -> /scan; nav2 amcl (map->odom);
controller_server (Regulated Pure Pursuit + local costmap from /scan) -> /cmd_vel -> gz. AegisNav's
planner supplies the path to controller_server's FollowPath action; there is no Nav2 global planner.
"""
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory("aegisnav_bringup")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    name = LaunchConfiguration("world").perform(context)
    with open(os.path.join(share, "config", "worlds.yaml"), encoding="utf-8") as f:
        presets = yaml.safe_load(f)
    if name not in presets:
        raise RuntimeError(f"unknown world {name!r}; choose one of {sorted(presets)}")
    preset = presets[name]
    world = os.path.join(share, preset["world_sdf"])
    map_yaml = os.path.join(share, preset["planning_map"])  # planning grid
    fine_map_yaml = os.path.join(share, preset["localization_map"])  # 0.05 m, same geometry
    nav2_params = os.path.join(share, preset["nav2_params"])
    spawn_x, spawn_y = (float(v) for v in preset["spawn_xy"])
    lidar_x, lidar_y, lidar_z = (str(v) for v in preset["lidar_xyz"])
    amcl_pose = {"initial_pose.x": spawn_x, "initial_pose.y": spawn_y}
    rviz_config = os.path.join(share, "rviz", "aegisnav.rviz")
    aegisnav_launch = os.path.join(get_package_share_directory("aegisnav_ros"), "launch", "aegisnav_core.launch.py")
    sim = {"use_sim_time": True}

    gz_launch = os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": f"-r -v 3 --gui-config {os.path.join(share, 'config', 'gazebo_gui_recording.config')} {world}"}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    # Server only; sensors still render offscreen (EGL) for the gpu_lidar.
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": f"-r -s --headless-rendering -v 3 {world}"}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="aegisnav_gz_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/aegis_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/aegis_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/aegis_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/aegis_bot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # Services used only by the visualization node to draw forecast tiles.
            "/world/aegisnav_rescue_world/create@ros_gz_interfaces/srv/SpawnEntity",
            "/world/aegisnav_rescue_world/remove@ros_gz_interfaces/srv/DeleteEntity",
        ],
        remappings=[
            ("/model/aegis_bot/cmd_vel", "/cmd_vel"),
            ("/model/aegis_bot/odometry", "/odom"),
            ("/model/aegis_bot/tf", "/tf"),
            ("/model/aegis_bot/scan", "/scan"),
        ],
        parameters=[sim],
    )
    # base_footprint (DiffDrive child frame) -> base_link (SDF link); lidar mount from the preset.
    # The scan's header.frame_id is whatever gz sets (default: the scoped sensor name); override with lidar_frame
    # after checking `ros2 topic echo /scan --field header.frame_id --once`.
    tf_base = Node(
        package="tf2_ros", executable="static_transform_publisher", name="tf_footprint_to_base",
        arguments=["--frame-id", "base_footprint", "--child-frame-id", "base_link"], parameters=[sim],
    )
    tf_lidar = Node(
        package="tf2_ros", executable="static_transform_publisher", name="tf_base_to_lidar",
        arguments=["--x", lidar_x, "--y", lidar_y, "--z", lidar_z, "--frame-id", "base_link",
                   "--child-frame-id", LaunchConfiguration("lidar_frame")],
        parameters=[sim],
    )
    # /map (0.05 m) feeds AMCL, the local costmap and RViz; /aegisnav/planning_map (0.4 m)
    # feeds the AegisNav planner and forecaster. Both are generated from the core grid.
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server", output="screen",
        parameters=[{"yaml_filename": fine_map_yaml, "use_sim_time": True}],
    )
    planning_map_server = Node(
        package="nav2_map_server", executable="map_server", name="planning_map_server", output="screen",
        parameters=[{"yaml_filename": map_yaml, "topic_name": "/aegisnav/planning_map", "use_sim_time": True}],
    )
    amcl = Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen", parameters=[nav2_params, amcl_pose])
    controller = Node(
        package="nav2_controller", executable="controller_server", name="controller_server", output="screen",
        parameters=[nav2_params],
    )
    lifecycle_localization = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
        output="screen", parameters=[nav2_params],
    )
    lifecycle_navigation = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
        output="screen", parameters=[nav2_params],
    )
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aegisnav_launch),
        launch_arguments={
            "scenario": LaunchConfiguration("scenario"), "seed": LaunchConfiguration("seed"),
            "scripted_events": LaunchConfiguration("scripted_events"),
            "execute": LaunchConfiguration("execute"),
            "sensing": LaunchConfiguration("sensing"),
            "scenario_name": preset["scenario_name"],
            "nominal_speed_mps": str(preset["nominal_speed_mps"]),
            "start_snap_tolerance_m": str(preset["start_snap_tolerance_m"]),
            "wall_clearance_cells": str(preset["wall_clearance_cells"]),
            "wall_clearance_penalty_m": str(preset["wall_clearance_penalty_m"]),
            "straight_near_obstacles": str(preset["straight_near_obstacles"]).lower(),
            "inflate_obstacles_cells": str(preset["inflate_obstacles_cells"]),
            "hazard_standoff_cells": str(preset.get("hazard_standoff_cells", 0)),
            "triage": (str(preset.get("triage", False)).lower() if LaunchConfiguration("triage").perform(context) == "preset"
                       else LaunchConfiguration("triage").perform(context)),
            "ignition_seed": LaunchConfiguration("ignition_seed"), "ignition_count": LaunchConfiguration("ignition_count"),
            "console": LaunchConfiguration("console"), "visualization": LaunchConfiguration("visualization"),
            "baseline": LaunchConfiguration("baseline"), "record_path": LaunchConfiguration("record_path"),
            "max_mission_s": LaunchConfiguration("max_mission_s"), "slow_cycle_warn_s": LaunchConfiguration("slow_cycle_warn_s"),
        }.items(),
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="aegisnav_rviz", output="screen",
        arguments=["-d", rviz_config], parameters=[sim], condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    # Camera director for recordings: needs the Gazebo GUI, so it is never started headless.
    director = Node(
        package="aegisnav_ros", executable="aegisnav-director", name="aegisnav_director", output="screen",
        parameters=[sim],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("director"), "' == 'true' and '",
                                                LaunchConfiguration("headless"), "' != 'true'"])),
    )
    models = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models"))
    return [models, gazebo, gazebo_headless, bridge, tf_base, tf_lidar, map_server, planning_map_server, amcl,
            controller, lifecycle_localization, lifecycle_navigation, core, rviz, director]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="rescue_building", description="Preset in config/worlds.yaml."),
        DeclareLaunchArgument("headless", default_value="false", description="Run gz server only (no GUI)."),
        DeclareLaunchArgument("director", default_value="true", description="Event-driven Gazebo camera for recordings."),
        DeclareLaunchArgument("start_rviz", default_value="false", description="Open RViz overlays; may cost GPU memory."),
        DeclareLaunchArgument("scenario", default_value="true", description="Run the seeded scenario driver."),
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("scripted_events", default_value="true", description="false = sandbox (goal only)."),
        DeclareLaunchArgument("execute", default_value="true", description="Send AegisNav routes to FollowPath."),
        DeclareLaunchArgument("console", default_value="true", description="Operator console web server."),
        DeclareLaunchArgument("visualization", default_value="true", description="Gazebo hazard tiles + effects (display only)."),
        DeclareLaunchArgument("triage", default_value="preset", description="preset | true | false (ablation)."),
        DeclareLaunchArgument("ignition_seed", default_value="-1", description="Batch study: seeded random ignitions."),
        DeclareLaunchArgument("ignition_count", default_value="3"),
        DeclareLaunchArgument("baseline", default_value="false", description="Evaluation baseline planner."),
        DeclareLaunchArgument("record_path", default_value="", description="Batch study: mission metrics JSON path."),
        DeclareLaunchArgument("max_mission_s", default_value="900"),
        DeclareLaunchArgument("slow_cycle_warn_s", default_value="1.0"),
        DeclareLaunchArgument("sensing", default_value="false",
                              description="true = the robot discovers hazards with a simulated onboard sensor."),
        DeclareLaunchArgument("lidar_frame", default_value="aegis_bot/base_link/front_lidar",
                              description="frame_id of /scan; confirm on the running system."),
        OpaqueFunction(function=_setup),
    ])
