#!/usr/bin/env python3
"""Minimal navigation test - exactly like nav2_mcp_server."""

import rclpy
import math
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

# Initialize ROS2
print("Initializing ROS2...")
rclpy.init()

# Create BasicNavigator
print("Creating BasicNavigator...")
navigator = BasicNavigator()

# Create a pose
print("Creating pose for bedroom (0.0, -4.0)...")
pose = PoseStamped()
pose.header.frame_id = 'map'
pose.header.stamp = navigator.get_clock().now().to_msg()
pose.pose.position.x = 0.0
pose.pose.position.y = -4.0
pose.pose.position.z = 0.0

yaw = 3.14
pose.pose.orientation.w = math.cos(yaw / 2.0)
pose.pose.orientation.z = math.sin(yaw / 2.0)

print(f"Pose: x={pose.pose.position.x}, y={pose.pose.position.y}")
print(f"Quaternion: w={pose.pose.orientation.w:.3f}, z={pose.pose.orientation.z:.3f}")

# Send goal
print("\nSending goal...")
navigator.goToPose(pose)
print("Goal sent!")

print("\nCheck RViz - robot should be navigating!")
print("Press Ctrl+C to stop...")

# Keep alive
try:
    while True:
        import time
        time.sleep(1)
except KeyboardInterrupt:
    print("\nShutting down...")
    rclpy.shutdown()
