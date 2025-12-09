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

"""Semantic Navigation MCP Server.

This module provides the main MCP server implementation for semantic navigation
with named locations.
"""

import json
import os
from pathlib import Path
from typing import Annotated, Any, Dict, Optional

import anyio
from fastmcp import Context, FastMCP

from .nav_client import NavigationClient
from .waypoint_manager import WaypointManager

# Initialize MCP server
mcp = FastMCP("Semantic Navigation Server")

# Path to waypoints file
WAYPOINTS_FILE = os.environ.get(
    'SEMANTIC_NAV_WAYPOINTS_FILE',
    str(Path(__file__).parent.parent.parent / 'waypoints.yaml')
)


def get_waypoint_manager() -> WaypointManager:
    """Get the singleton WaypointManager instance.

    Returns
    -------
    WaypointManager
        The waypoint manager instance.
    """
    return WaypointManager.get_instance(WAYPOINTS_FILE)


def get_nav_client() -> NavigationClient:
    """Get the singleton NavigationClient instance.

    Returns
    -------
    NavigationClient
        The navigation client instance.
    """
    return NavigationClient.get_instance()


# ========================
# MCP Resources
# ========================

@mcp.resource("semantic://locations/all")
def get_all_locations() -> str:
    """Get all semantic locations with full details.

    Returns
    -------
    str
        JSON string with all location data.
    """
    manager = get_waypoint_manager()
    return json.dumps(manager.get_all_waypoints(), indent=2)


@mcp.resource("semantic://locations/list")
def list_location_names() -> str:
    """Get list of all location names.

    Returns
    -------
    str
        JSON string with list of location names.
    """
    manager = get_waypoint_manager()
    return json.dumps({"locations": manager.list_waypoints()}, indent=2)


@mcp.resource("semantic://locations/{name}")
def get_location_details(name: str) -> str:
    """Get details for a specific location.

    Parameters
    ----------
    name : str
        Name of the location.

    Returns
    -------
    str
        JSON string with location details.
    """
    manager = get_waypoint_manager()
    location = manager.get_waypoint(name)
    if location is None:
        return json.dumps({"error": f"Location '{name}' not found"})
    return json.dumps({name: location}, indent=2)


# ========================
# MCP Tools
# ========================

@mcp.tool(
    name='list_locations',
    description="""List all available semantic locations.

    Returns a list of location names that the robot can navigate to.

    Example usage:
    - show me all locations
    - what places can the robot go?
    - list available waypoints
    """,
    tags={'locations', 'list', 'waypoints', 'places'},
    annotations={
        'title': 'List Locations',
        'readOnlyHint': True,
        'openWorldHint': False
    },
)
async def list_locations(
    ctx: Annotated[Optional[Context], 'MCP context for logging'] = None,
) -> str:
    """List all available semantic locations."""
    return await anyio.to_thread.run_sync(_list_locations_sync, ctx)


@mcp.tool(
    name='get_location',
    description="""Get detailed information about a specific location.

    Returns the coordinates, orientation, and metadata for a named location.

    Example usage:
    - show me details for kitchen
    - what are the coordinates of the charging dock?
    - get info about home position
    """,
    tags={'location', 'details', 'info', 'coordinates'},
    annotations={
        'title': 'Get Location Details',
        'readOnlyHint': True,
        'openWorldHint': False
    },
)
async def get_location(
    name: Annotated[str, 'Name of the location'],
    ctx: Annotated[Optional[Context], 'MCP context for logging'] = None,
) -> str:
    """Get details about a specific location."""
    return await anyio.to_thread.run_sync(_get_location_sync, name, ctx)


@mcp.tool(
    name='search_locations',
    description="""Search for locations by name, zone, or metadata.

    Allows filtering locations by various criteria.

    Example usage:
    - search for locations in the kitchen zone
    - find all charging stations
    - show locations with public access
    """,
    tags={'search', 'filter', 'query', 'find'},
    annotations={
        'title': 'Search Locations',
        'readOnlyHint': True,
        'openWorldHint': False
    },
)
async def search_locations(
    query: Annotated[str, 'Text to search in name and description'] = '',
    zone: Annotated[str, 'Filter by zone'] = '',
    ctx: Annotated[Optional[Context], 'MCP context for logging'] = None,
) -> str:
    """Search for locations by criteria."""
    return await anyio.to_thread.run_sync(
        _search_locations_sync, query, zone, ctx
    )


@mcp.tool(
    name='register_location',
    description="""Register a new semantic location or update an existing one.

    Creates a named location with coordinates, orientation, and optional metadata.

    Example usage:
    - register a new location called office at (5, 2) facing north
    - save current position as waypoint_1
    - create location kitchen at x=2.5 y=3.0 yaw=1.57
    """,
    tags={'register', 'add', 'create', 'save', 'waypoint'},
    annotations={
        'title': 'Register Location',
        'readOnlyHint': False,
        'openWorldHint': False
    },
)
async def register_location(
    name: Annotated[str, 'Name of the location'],
    x: Annotated[float, 'X coordinate in map frame'],
    y: Annotated[float, 'Y coordinate in map frame'],
    yaw: Annotated[float, 'Orientation in radians (0=east, π/2=north)'] = 0.0,
    description: Annotated[str, 'Description of the location'] = '',
    zone: Annotated[str, 'Zone name'] = '',
    ctx: Annotated[Optional[Context], 'MCP context for logging'] = None,
) -> str:
    """Register a new semantic location."""
    return await anyio.to_thread.run_sync(
        _register_location_sync, name, x, y, yaw, description, zone, ctx
    )


@mcp.tool(
    name='remove_location',
    description="""Remove a semantic location from the database.

    Example usage:
    - remove location old_waypoint
    - delete the kitchen location
    """,
    tags={'remove', 'delete', 'unregister'},
    annotations={
        'title': 'Remove Location',
        'readOnlyHint': False,
        'openWorldHint': False
    },
)
async def remove_location(
    name: Annotated[str, 'Name of the location to remove'],
    ctx: Annotated[Optional[Context], 'MCP context for logging'] = None,
) -> str:
    """Remove a semantic location."""
    return await anyio.to_thread.run_sync(_remove_location_sync, name, ctx)


@mcp.tool(
    name='navigate_to_location',
    description="""Navigate the robot to a named semantic location.

    This is the main semantic navigation tool. Instead of specifying coordinates,
    you can use human-readable location names.

    Example usage:
    - go to the kitchen
    - navigate to charging_dock
    - take the robot to home position
    """,
    tags={'navigate', 'go to', 'move to', 'semantic navigation'},
    annotations={
        'title': 'Navigate to Location',
        'readOnlyHint': False,
        'openWorldHint': False
    },
)
async def navigate_to_location(
    name: Annotated[str, 'Name of the location to navigate to'],
    ctx: Annotated[Optional[Context], 'MCP context for logging'] = None,
) -> str:
    """Navigate to a named semantic location."""
    return await anyio.to_thread.run_sync(_navigate_to_location_sync, name, ctx)


@mcp.tool(
    name='navigate_relative',
    description="""Navigate the robot relative to its current position.

    Move the robot forward/backward, left/right, and rotate it relative to where
    it is currently facing. Uses Nav2 stack with base_link frame.

    Example usage:
    - move forward 2 meters
    - move backward 0.5 meters
    - move left 1 meter
    - turn 90 degrees right (use -1.57 radians)
    - turn 180 degrees (use 3.14 radians)
    - move forward 1m and turn 45 degrees left (use 0.785 radians)
    """,
    tags={'navigate', 'relative', 'move', 'turn', 'forward', 'backward'},
    annotations={
        'title': 'Navigate Relative',
        'readOnlyHint': False,
        'openWorldHint': False
    },
)
async def navigate_relative(
    forward: Annotated[float, 'Distance to move forward in meters (negative for backward)'] = 0.0,
    left: Annotated[float, 'Distance to move left in meters (negative for right)'] = 0.0,
    yaw: Annotated[float, 'Rotation in radians (positive counter-clockwise, negative clockwise)'] = 0.0,
    ctx: Annotated[Optional[Context], 'MCP context for logging'] = None,
) -> str:
    """Navigate relative to current robot pose."""
    return await anyio.to_thread.run_sync(_navigate_relative_sync, forward, left, yaw, ctx)


# ========================
# Synchronous Implementation Functions
# ========================

def _list_locations_sync(ctx: Optional[Context] = None) -> str:
    """List locations synchronously."""
    manager = get_waypoint_manager()
    locations = manager.list_waypoints()

    if not locations:
        return "No locations registered yet."

    result = {
        "count": len(locations),
        "locations": locations
    }
    return json.dumps(result, indent=2)


def _get_location_sync(name: str, ctx: Optional[Context] = None) -> str:
    """Get location details synchronously."""
    manager = get_waypoint_manager()
    location = manager.get_waypoint(name)

    if location is None:
        return json.dumps({
            "error": f"Location '{name}' not found",
            "available_locations": manager.list_waypoints()
        }, indent=2)

    pos = location['position']
    ori = location['orientation']
    metadata = location.get('metadata', {})

    result = {
        "name": name,
        "coordinates": {
            "x": pos['x'],
            "y": pos['y'],
            "yaw": ori['yaw']
        },
        "metadata": metadata
    }
    return json.dumps(result, indent=2)


def _search_locations_sync(
    query: str,
    zone: str,
    ctx: Optional[Context] = None
) -> str:
    """Search locations synchronously."""
    manager = get_waypoint_manager()

    # Build filter
    search_params: Dict[str, Any] = {}
    if query:
        search_params['query'] = query
    if zone:
        search_params['zone'] = zone

    results = manager.search_waypoints(**search_params)

    return json.dumps({
        "count": len(results),
        "locations": list(results.keys()),
        "details": results
    }, indent=2)


def _register_location_sync(
    name: str,
    x: float,
    y: float,
    yaw: float,
    description: str,
    zone: str,
    ctx: Optional[Context] = None
) -> str:
    """Register location synchronously."""
    manager = get_waypoint_manager()

    # Build metadata
    metadata = {}
    if description:
        metadata['description'] = description
    if zone:
        metadata['zone'] = zone

    # Check if updating existing location
    existing = manager.get_waypoint(name)
    is_update = existing is not None

    success = manager.add_waypoint(name, x, y, yaw, metadata)

    if success:
        action = "updated" if is_update else "registered"
        return json.dumps({
            "status": "success",
            "action": action,
            "location": name,
            "coordinates": {"x": x, "y": y, "yaw": yaw},
            "metadata": metadata
        }, indent=2)
    else:
        return json.dumps({
            "status": "error",
            "message": f"Failed to register location '{name}'"
        }, indent=2)


def _remove_location_sync(name: str, ctx: Optional[Context] = None) -> str:
    """Remove location synchronously."""
    manager = get_waypoint_manager()

    success = manager.remove_waypoint(name)

    if success:
        return json.dumps({
            "status": "success",
            "message": f"Location '{name}' removed successfully"
        }, indent=2)
    else:
        return json.dumps({
            "status": "error",
            "message": f"Location '{name}' not found",
            "available_locations": manager.list_waypoints()
        }, indent=2)


def _navigate_to_location_sync(name: str, ctx: Optional[Context] = None) -> str:
    """Navigate to location synchronously."""
    manager = get_waypoint_manager()
    nav_client = get_nav_client()

    # Get location
    location = manager.get_waypoint(name)
    if location is None:
        return json.dumps({
            "status": "error",
            "message": f"Location '{name}' not found",
            "available_locations": manager.list_waypoints()
        }, indent=2)

    # Extract coordinates
    pos = location['position']
    ori = location['orientation']
    x = pos['x']
    y = pos['y']
    yaw = ori['yaw']

    # Navigate
    try:
        result = nav_client.navigate_to_pose(x, y, yaw)
        return json.dumps({
            "status": "success",
            "location": name,
            "coordinates": {"x": x, "y": y, "yaw": yaw},
            "result": result
        }, indent=2)
    except Exception as e:
        return json.dumps({
            "status": "error",
            "location": name,
            "message": str(e)
        }, indent=2)


def _navigate_relative_sync(
    forward: float,
    left: float,
    yaw: float,
    ctx: Optional[Context] = None
) -> str:
    """Navigate relative to current pose synchronously."""
    nav_client = get_nav_client()

    # Ensure parameters are floats (in case they come as strings from MCP client)
    try:
        forward = float(forward)
        left = float(left)
        yaw = float(yaw)
    except (ValueError, TypeError) as e:
        return json.dumps({
            "status": "error",
            "message": f"Invalid parameter types: {str(e)}. Expected numeric values."
        }, indent=2)

    try:
        result = nav_client.navigate_relative(forward, left, yaw)
        return json.dumps({
            "status": "success",
            "movement": {
                "forward": forward,
                "left": left,
                "yaw": yaw
            },
            "result": result
        }, indent=2)
    except Exception as e:
        return json.dumps({
            "status": "error",
            "message": str(e),
            "movement": {
                "forward": forward,
                "left": left,
                "yaw": yaw
            }
        }, indent=2)
