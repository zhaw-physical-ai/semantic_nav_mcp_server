#!/usr/bin/env python3
"""Test script for relative navigation."""

import rclpy
from semantic_nav_mcp_server.nav_client import NavigationClient


def main():
    """Test relative navigation."""
    # Initialize ROS2
    rclpy.init()

    # Get navigation client
    nav_client = NavigationClient.get_instance()

    print("Testing relative navigation...")
    print("\n=== Test 1: Move forward 2 meters ===")
    result = nav_client.navigate_relative(forward=2.0)
    print(f"Result: {result}")

    print("\n=== Test 2: Move backward 1 meter ===")
    result = nav_client.navigate_relative(forward=-1.0)
    print(f"Result: {result}")

    print("\n=== Test 3: Turn 90 degrees left (π/2 radians) ===")
    result = nav_client.navigate_relative(yaw=1.5708)
    print(f"Result: {result}")

    print("\n=== Test 4: Move left 0.5 meters ===")
    result = nav_client.navigate_relative(left=0.5)
    print(f"Result: {result}")

    print("\nAll tests completed!")

    # Cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    main()
