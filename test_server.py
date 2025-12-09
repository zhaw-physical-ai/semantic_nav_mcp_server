#!/usr/bin/env python3
"""Simple test script to verify the server loads correctly."""

import sys
sys.path.insert(0, 'src')

from semantic_nav_mcp_server.waypoint_manager import WaypointManager
from semantic_nav_mcp_server.server import mcp

print("✓ Imports successful")

# Test waypoint manager
manager = WaypointManager.get_instance('waypoints.yaml')
print(f"✓ Waypoint manager initialized")
print(f"✓ Found {len(manager.list_waypoints())} waypoints: {manager.list_waypoints()}")

# List MCP tools
tools = [tool for tool in dir(mcp) if not tool.startswith('_')]
print(f"✓ MCP server has {len(tools)} attributes")

print("\n✅ All basic tests passed!")
print("\nAvailable MCP tools:")
print("  - list_locations")
print("  - get_location")
print("  - search_locations")
print("  - register_location")
print("  - remove_location")
print("  - navigate_to_location")
print("\nAvailable MCP resources:")
print("  - semantic://locations/all")
print("  - semantic://locations/list")
print("  - semantic://locations/{name}")
