"""ROS 2 node that feeds marker poses from TF into the SLAM filters."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, ExtrapolationException, LookupException, TimeoutException
from tf2_ros.transform_listener import TransformListener

from filters.ekf_with_rotations import EKF_Rotations
from filters.extended_kalman_filter import EKF
from outputs.trajectory_writer import TrajectoryWriter


def init_tracker(
    filter_type: str,
    initial_pose: np.ndarray,
    map_file: str | None,
    *,
    load_calibration: bool,
) -> object:
    """Initialize the tracker based on the filter type."""
    if filter_type == "ekf":
        return EKF(
            initial_pose,
            map_file,
            load_calibration=load_calibration,
            init_detector=False,
        )
    if filter_type == "ekf_rotations":
        return EKF_Rotations(
            initial_pose,
            map_file,
            load_calibration=load_calibration,
            init_detector=False,
        )
    if filter_type == "factorgraph":
        from filters.factor_graph import FactorGraph

        return FactorGraph(
            initial_pose,
            map_file,
            load_calibration=load_calibration,
            init_detector=False,
        )

    msg = f"Unknown filter type: {filter_type}"
    raise ValueError(msg)


class TfSlamNode(Node):
    """ROS 2 node for TF-based ArUco SLAM."""

    def __init__(self) -> None:
        super().__init__("aruco_slam_tf")

        self.declare_parameter("filter", "ekf")
        self.declare_parameter("base_frame", "robot_base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter(
            "marker_frame_regex",
            r"^aruco_marker_(\d+)_base$",
        )
        self.declare_parameter("update_rate_hz", 30.0)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("camera_pose_topic", "camera_pose")
        self.declare_parameter("save_trajectory", True)
        self.declare_parameter("save_map", False)
        self.declare_parameter("output_dir", "~/.ros/aruco_slam")
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("map_file", "")
        self.declare_parameter("load_map", False)
        self.declare_parameter("load_calibration", False)
        self.declare_parameter("warn_interval_sec", 5.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.marker_frame_regex = re.compile(
            self.get_parameter("marker_frame_regex").value,
        )

        output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)

        self.map_file = self._resolve_output_path(
            self.get_parameter("map_file").value,
            output_dir,
            "map.txt",
        )
        self.trajectory_file = self._resolve_output_path(
            self.get_parameter("trajectory_file").value,
            output_dir,
            "trajectory.txt",
        )

        load_map = bool(self.get_parameter("load_map").value)
        map_file = str(self.map_file) if load_map else None

        initial_pose = np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], dtype=float)
        self.get_logger().info(
            f"TfSlamNode init: module={Path(__file__).resolve()} "
            f"filter={self.get_parameter('filter').value} "
            f"base_frame={self.base_frame} map_frame={self.map_frame}",
        )
        self.get_logger().info("Initializing tracker backend...")
        self.tracker = init_tracker(
            self.get_parameter("filter").value,
            initial_pose,
            map_file,
            load_calibration=bool(self.get_parameter("load_calibration").value),
        )
        self.get_logger().info("Tracker backend initialized.")

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("camera_pose_topic").value,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.save_trajectory = bool(self.get_parameter("save_trajectory").value)
        self.save_map = bool(self.get_parameter("save_map").value)

        self.traj_writer = None
        if self.save_trajectory:
            self.traj_writer = TrajectoryWriter(str(self.trajectory_file))
            self.traj_writer.__enter__()

        update_rate = float(self.get_parameter("update_rate_hz").value)
        update_rate = update_rate if update_rate > 0.0 else 30.0
        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_sec").value))
        self.warn_interval = Duration(seconds=float(self.get_parameter("warn_interval_sec").value))
        self.last_no_frames_warn = None
        self.last_no_pose_warn = None
        self.last_tf_fail_warn = None
        self.timer = self.create_timer(1.0 / update_rate, self._on_timer)

    @staticmethod
    def _resolve_output_path(path_value: str, output_dir: Path, default_name: str) -> Path:
        if path_value:
            path = Path(path_value).expanduser()
            if not path.is_absolute():
                return output_dir / path
            return path
        return output_dir / default_name

    def _list_marker_frames(self) -> Iterable[tuple[str, int]]:
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
        except Exception:
            return []

        matches: list[tuple[str, int]] = []
        for line in frames_yaml.splitlines():
            if ":" not in line:
                continue
            frame = line.split(":", 1)[0].strip()
            if not frame:
                continue
            match = self.marker_frame_regex.match(frame)
            if match:
                matches.append((frame, int(match.group(1))))
        return matches

    def _maybe_warn(self, attr_name: str, message: str) -> None:
        now = self.get_clock().now()
        last_time = getattr(self, attr_name)
        if last_time is None or (now - last_time) >= self.warn_interval:
            self.get_logger().warn(message)
            setattr(self, attr_name, now)

    def _lookup_marker_pose(self, frame: str) -> np.ndarray | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame,
                rclpy.time.Time(),
                timeout=self.tf_timeout,
            )
        except (LookupException, ExtrapolationException, TimeoutException):
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        rotvec = Rotation.from_quat(
            [rotation.x, rotation.y, rotation.z, rotation.w],
        ).as_rotvec()

        return np.array(
            [translation.x, translation.y, translation.z, *rotvec],
            dtype=float,
        )

    def _on_timer(self) -> None:
        ids: list[int] = []
        poses: list[np.ndarray] = []
        frames = list(self._list_marker_frames())

        if not frames:
            self._maybe_warn(
                "last_no_frames_warn",
                f"No TF frames match regex '{self.marker_frame_regex.pattern}'.",
            )
            return

        failed = 0
        for frame, marker_id in frames:
            pose = self._lookup_marker_pose(frame)
            if pose is None:
                failed += 1
                continue
            ids.append(marker_id)
            poses.append(pose)

        if not ids:
            self._maybe_warn(
                "last_no_pose_warn",
                f"No marker poses available from TF (failed {failed}/{len(frames)} lookups).",
            )
            return

        if failed:
            self._maybe_warn(
                "last_tf_fail_warn",
                f"TF lookup failed for {failed}/{len(frames)} marker frames.",
            )

        self.tracker.observe(ids, poses)
        camera_pose, _ = self.tracker.get_poses()
        self._publish_camera_pose(camera_pose)

        if self.traj_writer is not None:
            now_ms = int(self.get_clock().now().nanoseconds / 1e6)
            self.traj_writer.write(now_ms, camera_pose)

    def _publish_camera_pose(self, camera_pose: np.ndarray) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = float(camera_pose[0])
        msg.pose.position.y = float(camera_pose[1])
        msg.pose.position.z = float(camera_pose[2])

        quat = camera_pose[3:7]
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])

        self.pose_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self.traj_writer is not None:
            self.traj_writer.__exit__(None, None, None)
            self.traj_writer = None

        if self.save_map:
            self.tracker.save_map(str(self.map_file))

        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TfSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
