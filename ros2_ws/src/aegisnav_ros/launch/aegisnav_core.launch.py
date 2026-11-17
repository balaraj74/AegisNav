"""Launch the AegisNav forecast and decision/path adapters.

A map publisher, localization source, Nav2 controller (if path execution is
enabled), and goal publisher must be launched separately.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory("aegisnav_ros"), "config", "aegisnav.yaml")
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="false", description="Start the seeded scenario driver (goal, confidence, ignition)."),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="false only for tests without a /clock source."),
        DeclareLaunchArgument("execute", default_value="false", description="Send routes to Nav2 FollowPath (needs a running controller)."),
        DeclareLaunchArgument("pose_source", default_value="tf", description="tf | topic"),
        DeclareLaunchArgument("seed", default_value="7", description="Scenario seed."),
        DeclareLaunchArgument("scripted_events", default_value="true",
                              description="false = sandbox: scenario only sets the goal; user adds fires/confidence."),
        DeclareLaunchArgument("scenario_name", default_value="", description="Core scenario for mission geometry/hazards."),
        DeclareLaunchArgument("nominal_speed_mps", default_value="0.35", description="Planner speed model (m/s)."),
        DeclareLaunchArgument("inflate_obstacles_cells", default_value="0", description="Planner hard wall margin (cells)."),
        DeclareLaunchArgument("straight_near_obstacles", default_value="false", description="No diagonal moves near walls."),
        DeclareLaunchArgument("wall_clearance_penalty_m", default_value="0.3", description="Planner wall-clearance penalty (m)."),
        DeclareLaunchArgument("wall_clearance_cells", default_value="1", description="Planner wall-clearance radius (cells)."),
        DeclareLaunchArgument("start_snap_tolerance_m", default_value="0.25", description="Pose-to-free-cell snap (m)."),
        DeclareLaunchArgument("console", default_value="true", description="Operator console web server (port 8080)."),
        DeclareLaunchArgument("ignition_seed", default_value="-1", description="Batch study: seeded random ignitions (>= 0)."),
        DeclareLaunchArgument("ignition_count", default_value="3"),
        DeclareLaunchArgument("baseline", default_value="false", description="Evaluation baseline: hazard-blind shortest path."),
        DeclareLaunchArgument("record_path", default_value="", description="Batch study: write mission metrics JSON here."),
        DeclareLaunchArgument("max_mission_s", default_value="900", description="Recorder timeout on the mission clock (s)."),
        DeclareLaunchArgument("slow_cycle_warn_s", default_value="1.0", description="Log planner cycles slower than this (0 = all)."),
        DeclareLaunchArgument("triage", default_value="false",
                              description="Choose the next inspection point by lowest-exposure safe route."),
        Node(
            package="aegisnav_ros", executable="aegisnav-console", name="aegisnav_console", output="screen",
            parameters=[{"use_sim_time": sim_time, "scenario_name": LaunchConfiguration("scenario_name")}],
            condition=IfCondition(LaunchConfiguration("console")),
        ),
        DeclareLaunchArgument("visualization", default_value="true", description="Gazebo hazard tiles + RViz mission markers."),
        Node(
            package="aegisnav_ros", executable="aegisnav-visualization", name="aegisnav_visualization", output="screen",
            parameters=[params, {"use_sim_time": sim_time, "scenario_name": LaunchConfiguration("scenario_name"),
                                 # Gazebo shows what is really burning; with sensing that is the hidden truth.
                                 "forecast_topic": PythonExpression(["'/aegisnav/hazard_truth' if '",
                                                                     LaunchConfiguration("sensing"),
                                                                     "' == 'true' else '/aegisnav/hazard_forecast'"])}],
            condition=IfCondition(LaunchConfiguration("visualization")),
        ),
        Node(
            package="aegisnav_ros", executable="aegisnav-scenario", name="aegisnav_scenario", output="screen",
            parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool), "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                         "scripted_events": ParameterValue(LaunchConfiguration("scripted_events"), value_type=bool),
                         "scenario_name": ParameterValue(LaunchConfiguration("scenario_name"), value_type=str),
                         "triage": ParameterValue(LaunchConfiguration("triage"), value_type=bool),
                         "ignition_seed": ParameterValue(LaunchConfiguration("ignition_seed"), value_type=int),
                         "ignition_count": ParameterValue(LaunchConfiguration("ignition_count"), value_type=int),
                         "nominal_speed_mps": ParameterValue(LaunchConfiguration("nominal_speed_mps"), value_type=float),
                         "wall_clearance_cells": ParameterValue(LaunchConfiguration("wall_clearance_cells"), value_type=int),
                         "wall_clearance_penalty_m": ParameterValue(LaunchConfiguration("wall_clearance_penalty_m"), value_type=float),
                         "straight_near_obstacles": ParameterValue(LaunchConfiguration("straight_near_obstacles"), value_type=bool),
                         "inflate_obstacles_cells": ParameterValue(LaunchConfiguration("inflate_obstacles_cells"), value_type=int),
                         # With sensing, confidence is estimated by the sensor node instead of scripted.
                         "publish_confidence": ParameterValue(
                             PythonExpression(["'", LaunchConfiguration("sensing"), "' != 'true'"]), value_type=bool)}],
            condition=IfCondition(LaunchConfiguration("scenario")),
        ),
        DeclareLaunchArgument("sensing", default_value="false",
                              description="true = the robot only knows hazards its simulated onboard sensor has seen."),
        Node(
            package="aegisnav_ros", executable="aegisnav-hazard-forecast",
            name="aegisnav_hazard_forecast", output="screen",
            parameters=[params, {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                                 "scenario_name": LaunchConfiguration("scenario_name"),
                                 "source_mode": PythonExpression(["'belief' if '", LaunchConfiguration("sensing"),
                                                                  "' == 'true' else 'truth'"])}],
        ),
        # With sensing: the hidden true hazard field (drives the Gazebo display and the simulated sensor only).
        Node(
            package="aegisnav_ros", executable="aegisnav-hazard-forecast",
            name="aegisnav_hazard_truth", output="screen",
            parameters=[{"use_sim_time": sim_time, "scenario_name": LaunchConfiguration("scenario_name"),
                         "map_topic": "/aegisnav/planning_map", "forecast_topic": "/aegisnav/hazard_truth",
                         "source_mode": "truth", "publish_markers": False}],
            condition=IfCondition(LaunchConfiguration("sensing")),
        ),
        Node(
            package="aegisnav_ros", executable="aegisnav-hazard-sensor", name="aegisnav_hazard_sensor", output="screen",
            parameters=[{"use_sim_time": sim_time, "scenario_name": LaunchConfiguration("scenario_name"),
                         "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                         "estimate_confidence": ParameterValue(LaunchConfiguration("sensing"), value_type=bool)}],
            condition=IfCondition(LaunchConfiguration("sensing")),
        ),
        Node(
            package="aegisnav_ros", executable="aegisnav-risk-planner",
            name="aegisnav_risk_planner", output="screen",
            parameters=[params, {
                "execute_with_follow_path": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
                "pose_source": LaunchConfiguration("pose_source"),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "scenario_name": LaunchConfiguration("scenario_name"),
                "nominal_speed_mps": ParameterValue(LaunchConfiguration("nominal_speed_mps"), value_type=float),
                "start_snap_tolerance_m": ParameterValue(LaunchConfiguration("start_snap_tolerance_m"), value_type=float),
                "wall_clearance_cells": ParameterValue(LaunchConfiguration("wall_clearance_cells"), value_type=int),
                "wall_clearance_penalty_m": ParameterValue(LaunchConfiguration("wall_clearance_penalty_m"), value_type=float),
                "straight_near_obstacles": ParameterValue(LaunchConfiguration("straight_near_obstacles"), value_type=bool),
                "inflate_obstacles_cells": ParameterValue(LaunchConfiguration("inflate_obstacles_cells"), value_type=int),
                "baseline_distance_only": ParameterValue(LaunchConfiguration("baseline"), value_type=bool),
                "slow_cycle_warn_s": ParameterValue(LaunchConfiguration("slow_cycle_warn_s"), value_type=float),
            }],
        ),
        Node(
            package="aegisnav_ros", executable="aegisnav-mission-recorder", name="aegisnav_mission_recorder", output="screen",
            parameters=[{"use_sim_time": sim_time, "scenario_name": LaunchConfiguration("scenario_name"),
                         "output_path": LaunchConfiguration("record_path"),
                         "max_mission_s": ParameterValue(LaunchConfiguration("max_mission_s"), value_type=float),
                         "truth_topic": PythonExpression(["'/aegisnav/hazard_truth' if '", LaunchConfiguration("sensing"),
                                                          "' == 'true' else '/aegisnav/hazard_forecast'"])}],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("record_path"), "' != ''"])),
        ),
    ])
