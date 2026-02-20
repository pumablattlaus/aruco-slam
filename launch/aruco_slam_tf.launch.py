from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="aruco_slam",
                executable="aruco_slam_tf",
                name="aruco_slam_tf",
                output="screen",
                parameters=[
                    {
                        "filter": "ekf",
                        "base_frame": "base_link",
                        "map_frame": "map",
                        "marker_frame_regex": r"^aruco_marker_(\d+)_base$",
                        "update_rate_hz": 30.0,
                        "tf_timeout_sec": 0.1,
                        "camera_pose_topic": "camera_pose",
                        "save_trajectory": True,
                        "save_map": False,
                        "output_dir": "~/.ros/aruco_slam",
                        "trajectory_file": "",
                        "map_file": "",
                        "load_map": False,
                        "load_calibration": False,
                    }
                ],
            )
        ]
    )
