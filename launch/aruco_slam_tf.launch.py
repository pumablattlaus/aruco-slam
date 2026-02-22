from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    launch_arguments = [
        DeclareLaunchArgument("filter", default_value="factorgraph"),
        DeclareLaunchArgument("base_frame", default_value="robot_base_footprint"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("marker_frame_regex", default_value=r"^aruco_marker_(\d+)_base$"),
        DeclareLaunchArgument("update_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("tf_timeout_sec", default_value="0.1"),
        DeclareLaunchArgument("camera_pose_topic", default_value="camera_pose"),
        DeclareLaunchArgument("save_trajectory", default_value="true"),
        DeclareLaunchArgument("save_map", default_value="false"),
        DeclareLaunchArgument("output_dir", default_value="~/.ros/aruco_slam"),
        DeclareLaunchArgument("trajectory_file", default_value=""),
        DeclareLaunchArgument("map_file", default_value=""),
        DeclareLaunchArgument("load_map", default_value="false"),
        DeclareLaunchArgument("load_calibration", default_value="false"),
        DeclareLaunchArgument("warn_interval_sec", default_value="5.0"),
    ]

    node_parameters = {
        "filter": LaunchConfiguration("filter"),
        "base_frame": LaunchConfiguration("base_frame"),
        "map_frame": LaunchConfiguration("map_frame"),
        "marker_frame_regex": LaunchConfiguration("marker_frame_regex"),
        "update_rate_hz": ParameterValue(LaunchConfiguration("update_rate_hz"), value_type=float),
        "tf_timeout_sec": ParameterValue(LaunchConfiguration("tf_timeout_sec"), value_type=float),
        "camera_pose_topic": LaunchConfiguration("camera_pose_topic"),
        "save_trajectory": ParameterValue(LaunchConfiguration("save_trajectory"), value_type=bool),
        "save_map": ParameterValue(LaunchConfiguration("save_map"), value_type=bool),
        "output_dir": LaunchConfiguration("output_dir"),
        "trajectory_file": LaunchConfiguration("trajectory_file"),
        "map_file": LaunchConfiguration("map_file"),
        "load_map": ParameterValue(LaunchConfiguration("load_map"), value_type=bool),
        "load_calibration": ParameterValue(LaunchConfiguration("load_calibration"), value_type=bool),
        "warn_interval_sec": ParameterValue(LaunchConfiguration("warn_interval_sec"), value_type=float),
    }

    return LaunchDescription(
        [
            *launch_arguments,
            Node(
                package="aruco_slam",
                executable="aruco_slam_tf",
                name="aruco_slam_tf",
                output="screen",
                parameters=[node_parameters],
            )
        ]
    )
