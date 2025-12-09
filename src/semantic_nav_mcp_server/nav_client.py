# Copyright (c) 2025
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Navigation client for semantic navigation."""

import math
import time
import threading
from typing import Optional, Any, Callable

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class NavigationClient:
    """Client for Nav2 navigation operations."""

    _instance: Optional['NavigationClient'] = None
    _lock = threading.Lock()

    def __init__(self):
        """Initialize navigation client."""
        self._navigator: Optional[BasicNavigator] = None

    @classmethod
    def get_instance(cls) -> 'NavigationClient':
        """Get singleton instance of NavigationClient.

        Returns
        -------
        NavigationClient
            The singleton instance.
        """
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    @property
    def navigator(self) -> BasicNavigator:
        """Get or create the navigator instance.

        Returns
        -------
        BasicNavigator
            The Nav2 navigator instance.
        """
        if self._navigator is None:
            print("[DEBUG] Creating BasicNavigator instance...")
            self._navigator = BasicNavigator()
            print("[DEBUG] Navigator created!")
        return self._navigator

    def create_pose_stamped(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = 'map',
        use_zero_time: bool = False
    ) -> PoseStamped:
        """Create a PoseStamped message from coordinates.

        Parameters
        ----------
        x : float
            X coordinate.
        y : float
            Y coordinate.
        yaw : float
            Orientation in radians.
        frame_id : str
            Frame ID (default: 'map').
        use_zero_time : bool
            If True, use zero timestamp (latest transform available).
            Useful for base_link frame to avoid TF extrapolation issues.

        Returns
        -------
        PoseStamped
            The pose message.
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id

        # Use zero timestamp for base_link to get latest transform
        if use_zero_time:
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0
        else:
            pose.header.stamp = self.navigator.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)

        return pose

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        frame_id: str = 'map'
    ) -> str:
        """Navigate to a pose.

        Parameters
        ----------
        x : float
            X coordinate.
        y : float
            Y coordinate.
        yaw : float
            Orientation in radians.
        frame_id : str
            Frame ID (default: 'map').

        Returns
        -------
        str
            Result message.

        Note
        ----
        Progress updates are printed to stdout with [PROGRESS] prefix.
        """
        print(f"[DEBUG] navigate_to_pose called: x={x}, y={y}, yaw={yaw}, frame={frame_id}")

        # Create and send navigation goal
        pose = self.create_pose_stamped(x, y, yaw, frame_id)
        print(f"[DEBUG] Created pose: {pose.pose.position.x}, {pose.pose.position.y}")
        print(f"[DEBUG] Pose quaternion: w={pose.pose.orientation.w:.3f}, z={pose.pose.orientation.z:.3f}")

        try:
            print("[DEBUG] Sending navigation goal via goToPose()...")
            self.navigator.goToPose(pose)
            print("[DEBUG] Goal sent successfully!")
        except Exception as e:
            error_msg = f"Failed to send navigation goal: {str(e)}"
            print(f"[ERROR] {error_msg}")
            return error_msg

        # Calculate initial distance to goal for progress tracking
        initial_distance = None
        current_pose = self.get_robot_pose()
        if current_pose:
            dx = x - current_pose.pose.position.x
            dy = y - current_pose.pose.position.y
            initial_distance = math.sqrt(dx*dx + dy*dy)
            print(f"[DEBUG] Initial distance to goal: {initial_distance:.2f}m")

        # Monitor navigation progress
        self._monitor_navigation_progress(x, y, initial_distance)

        # Get final result
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            print("[PROGRESS] 100% - Navigation complete")
            return f"Successfully navigated to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})"
        elif result == TaskResult.CANCELED:
            return f"Navigation was canceled"
        elif result == TaskResult.FAILED:
            return f"Navigation failed to reach ({x:.2f}, {y:.2f})"
        else:
            return f"Navigation completed with unknown result: {result}"

    def get_robot_pose(self) -> Optional[PoseStamped]:
        """Get current robot pose.

        Returns
        -------
        Optional[PoseStamped]
            Current robot pose or None if unavailable.
        """
        try:
            return self.navigator.getCurrentPose()
        except Exception:
            return None

    def cancel_navigation(self) -> str:
        """Cancel current navigation task.

        Returns
        -------
        str
            Result message.
        """
        self.navigator.cancelTask()
        return "Navigation canceled"

    def navigate_relative(
        self,
        forward: float = 0.0,
        left: float = 0.0,
        yaw: float = 0.0
    ) -> str:
        """Navigate relative to the robot's current pose.

        Parameters
        ----------
        forward : float
            Distance to move forward in meters (negative = backward).
        left : float
            Distance to move left in meters (negative = right).
        yaw : float
            Rotation in radians (positive = counter-clockwise).

        Returns
        -------
        str
            Result message.

        Note
        ----
        Progress updates are printed to stdout with [PROGRESS] prefix.
        """
        print(f"[DEBUG] navigate_relative called: forward={forward}m, left={left}m, yaw={yaw}rad")

        # Get initial pose for progress calculation
        initial_pose = self.get_robot_pose()
        if not initial_pose:
            print("[WARNING] Could not get initial pose for progress tracking")

        # Create a pose in base_link frame with zero timestamp
        # Zero timestamp means "use latest available transform"
        # In base_link: x=forward, y=left
        pose = self.create_pose_stamped(forward, left, yaw, frame_id='base_link', use_zero_time=True)

        print(f"[DEBUG] Created relative pose in base_link frame")
        print(f"[DEBUG] Position: x={pose.pose.position.x}, y={pose.pose.position.y}")
        print(f"[DEBUG] Orientation: w={pose.pose.orientation.w:.3f}, z={pose.pose.orientation.z:.3f}")
        print(f"[DEBUG] Timestamp: {pose.header.stamp.sec}.{pose.header.stamp.nanosec}")

        try:
            print("[DEBUG] Sending relative navigation goal via goToPose()...")
            self.navigator.goToPose(pose)
            print("[DEBUG] Goal sent successfully!")
        except Exception as e:
            error_msg = f"Failed to send relative navigation goal: {str(e)}"
            print(f"[ERROR] {error_msg}")
            return error_msg

        # Calculate total expected distance (linear + angular)
        linear_distance = math.sqrt(forward*forward + left*left)
        angular_distance = abs(yaw)
        # Weight angular motion as equivalent to 0.5m per radian
        total_expected_distance = linear_distance + angular_distance * 0.5
        print(f"[DEBUG] Expected travel: {total_expected_distance:.2f}m equivalent")

        # Monitor relative navigation progress
        self._monitor_relative_navigation_progress(
            initial_pose, forward, left, yaw, total_expected_distance
        )

        # Get final result
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            print("[PROGRESS] 100% - Navigation complete")
            return f"Successfully moved: forward={forward:.2f}m, left={left:.2f}m, yaw={yaw:.2f}rad"
        elif result == TaskResult.CANCELED:
            return f"Relative navigation was canceled"
        elif result == TaskResult.FAILED:
            return f"Relative navigation failed"
        else:
            return f"Relative navigation completed with unknown result: {result}"

    def _monitor_navigation_progress(
        self,
        target_x: float,
        target_y: float,
        initial_distance: Optional[float]
    ) -> None:
        """Monitor navigation progress and provide updates.

        Parameters
        ----------
        target_x : float
            Target X coordinate.
        target_y : float
            Target Y coordinate.
        initial_distance : float, optional
            Initial distance to goal.
        """
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()

            # Provide progress updates every 10 iterations
            if i % 10 == 0:
                if feedback and hasattr(feedback, 'estimated_time_remaining'):
                    seconds = Duration.from_msg(
                        feedback.estimated_time_remaining
                    ).nanoseconds / 1e9
                    print(f"[PROGRESS] Estimated time of arrival: {seconds:.0f} seconds")
                else:
                    # Calculate progress percentage based on distance
                    progress_pct = 0
                    current_pose = self.get_robot_pose()
                    if current_pose and initial_distance and initial_distance > 0.01:
                        dx = target_x - current_pose.pose.position.x
                        dy = target_y - current_pose.pose.position.y
                        current_distance = math.sqrt(dx*dx + dy*dy)
                        progress_pct = int(100 * (1.0 - current_distance / initial_distance))
                        progress_pct = max(0, min(99, progress_pct))
                        print(f"[PROGRESS] {progress_pct}% - Navigation in progress...")
                    else:
                        print("[PROGRESS] Navigation in progress...")

    def _monitor_relative_navigation_progress(
        self,
        initial_pose: Optional[PoseStamped],
        forward: float,
        left: float,
        yaw: float,
        total_expected_distance: float
    ) -> None:
        """Monitor relative navigation progress and provide updates.

        Parameters
        ----------
        initial_pose : PoseStamped, optional
            Initial robot pose.
        forward : float
            Forward distance.
        left : float
            Left distance.
        yaw : float
            Rotation angle.
        total_expected_distance : float
            Total expected travel distance.
        """
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()

            # Provide progress updates every 10 iterations
            if i % 10 == 0:
                if feedback and hasattr(feedback, 'estimated_time_remaining'):
                    seconds = Duration.from_msg(
                        feedback.estimated_time_remaining
                    ).nanoseconds / 1e9
                    print(f"[PROGRESS] Estimated time of arrival: {seconds:.0f} seconds")
                else:
                    # Calculate progress percentage based on distance traveled
                    progress_pct = 0
                    current_pose = self.get_robot_pose()
                    if initial_pose and current_pose and total_expected_distance > 0.01:
                        # Calculate distance traveled
                        dx = current_pose.pose.position.x - initial_pose.pose.position.x
                        dy = current_pose.pose.position.y - initial_pose.pose.position.y
                        distance_traveled = math.sqrt(dx*dx + dy*dy)

                        # Calculate angular change
                        initial_yaw = 2 * math.atan2(
                            initial_pose.pose.orientation.z,
                            initial_pose.pose.orientation.w
                        )
                        current_yaw = 2 * math.atan2(
                            current_pose.pose.orientation.z,
                            current_pose.pose.orientation.w
                        )
                        angular_traveled = abs(current_yaw - initial_yaw)
                        # Normalize to [-pi, pi]
                        while angular_traveled > math.pi:
                            angular_traveled -= 2 * math.pi
                        angular_traveled = abs(angular_traveled)

                        # Combine linear and angular progress
                        total_traveled = distance_traveled + angular_traveled * 0.5
                        progress_pct = int(100 * total_traveled / total_expected_distance)
                        progress_pct = max(0, min(99, progress_pct))
                        print(f"[PROGRESS] {progress_pct}% - Navigation in progress...")
                    else:
                        print("[PROGRESS] Navigation in progress...")
