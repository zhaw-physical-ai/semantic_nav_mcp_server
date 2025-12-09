#!/usr/bin/env python3
"""Direct test of navigation without MCP."""

import sys
import os
import rclpy

# Setup environment
os.environ['SEMANTIC_NAV_WAYPOINTS_FILE'] = 'waypoints.yaml'
sys.path.insert(0, 'src')

from semantic_nav_mcp_server.waypoint_manager import WaypointManager
from semantic_nav_mcp_server.nav_client import NavigationClient

print("=" * 60)
print("DIRECT NAVIGATION TEST")
print("=" * 60)

# Initialize ROS2
print("\n1. Initializing ROS2...")
rclpy.init()
print("   ✓ ROS2 initialized")

# Load waypoints
print("\n2. Loading waypoints...")
manager = WaypointManager.get_instance('waypoints.yaml')
locations = manager.list_waypoints()
print(f"   ✓ Found locations: {locations}")

# Pick first location
if not locations:
    print("   ✗ No locations found!")
    sys.exit(1)

test_location = locations[0]
waypoint = manager.get_waypoint(test_location)
pos = waypoint['position']
ori = waypoint['orientation']

print(f"\n3. Testing navigation to '{test_location}'")
print(f"   Target: x={pos['x']}, y={pos['y']}, yaw={ori['yaw']}")

# Create navigation client and navigate
print("\n4. Creating NavigationClient...")
nav_client = NavigationClient.get_instance()

print("\n5. Calling navigate_to_pose()...")
result = nav_client.navigate_to_pose(pos['x'], pos['y'], ori['yaw'])

print("\n" + "=" * 60)
print("RESULT:")
print(result)
print("=" * 60)

# Cleanup
rclpy.shutdown()
