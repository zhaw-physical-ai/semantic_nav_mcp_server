#!/usr/bin/env python3
"""Quick test script to verify navigation setup."""

import sys
import os

# Add src to path
sys.path.insert(0, 'src')
os.environ['SEMANTIC_NAV_WAYPOINTS_FILE'] = 'waypoints.yaml'

from semantic_nav_mcp_server.waypoint_manager import WaypointManager
from semantic_nav_mcp_server.nav_client import NavigationClient

print("=" * 60)
print("SEMANTIC NAVIGATION DIAGNOSTIC TEST")
print("=" * 60)

# Test 1: Load waypoints
print("\n1. Testing Waypoint Manager...")
try:
    manager = WaypointManager.get_instance('waypoints.yaml')
    locations = manager.list_waypoints()
    print(f"   ✓ Found {len(locations)} locations: {locations}")

    if locations:
        first_loc = locations[0]
        waypoint = manager.get_waypoint(first_loc)
        pos = waypoint['position']
        ori = waypoint['orientation']
        print(f"   ✓ Sample location '{first_loc}':")
        print(f"     Position: x={pos['x']}, y={pos['y']}, z={pos['z']}")
        print(f"     Orientation: yaw={ori['yaw']}")
except Exception as e:
    print(f"   ✗ Error: {e}")
    sys.exit(1)

# Test 2: Initialize ROS2 and Nav client
print("\n2. Testing Navigation Client...")
try:
    nav_client = NavigationClient.get_instance()
    print("   ✓ NavigationClient initialized")
    print("   ✓ ROS2 initialized")
except Exception as e:
    print(f"   ✗ Error: {e}")
    sys.exit(1)

# Test 3: Check if we can get robot pose
print("\n3. Testing Robot Localization...")
try:
    pose = nav_client.get_robot_pose()
    if pose:
        print(f"   ✓ Robot pose available:")
        print(f"     Frame: {pose.header.frame_id}")
        print(f"     Position: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
    else:
        print("   ⚠ Robot pose not available (is localization running?)")
except Exception as e:
    print(f"   ⚠ Could not get robot pose: {e}")

# Test 4: Try creating a pose
print("\n4. Testing Pose Creation...")
try:
    test_pose = nav_client.create_pose_stamped(1.0, 2.0, 0.0)
    print(f"   ✓ Created test pose:")
    print(f"     Frame: {test_pose.header.frame_id}")
    print(f"     Position: x={test_pose.pose.position.x}, y={test_pose.pose.position.y}")
    print(f"     Orientation: w={test_pose.pose.orientation.w:.3f}, z={test_pose.pose.orientation.z:.3f}")
except Exception as e:
    print(f"   ✗ Error: {e}")
    sys.exit(1)

print("\n" + "=" * 60)
print("DIAGNOSTIC COMPLETE")
print("=" * 60)
print("\nTo test actual navigation:")
print("1. Make sure Nav2 is running: ros2 node list | grep nav")
print("2. Make sure robot is localized in RViz")
print("3. Try: navigate_to_location('home')  # or another location")
print("\nIf navigation still doesn't work, check:")
print("- Is the goal location within the map bounds?")
print("- Is the path to the goal free of obstacles?")
print("- Check Nav2 logs: ros2 topic echo /diagnostics")
