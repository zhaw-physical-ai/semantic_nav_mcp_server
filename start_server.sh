#!/bin/bash
# Semantic Navigation MCP Server Startup Script

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source ROS 2 environment
# Adjust this path if your ROS 2 installation is elsewhere
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "Warning: ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}"
fi

# Source workspace if path is provided via environment variable
if [ -n "$ROS_WORKSPACE" ] && [ -f "$ROS_WORKSPACE/install/setup.bash" ]; then
    source "$ROS_WORKSPACE/install/setup.bash"
fi

# Set waypoints file location (defaults to waypoints.yaml in server directory)
export SEMANTIC_NAV_WAYPOINTS_FILE="${SEMANTIC_NAV_WAYPOINTS_FILE:-${SCRIPT_DIR}/waypoints.yaml}"

# Run the MCP server with system Python access
UV_SYSTEM_PYTHON=1 uv run --directory "$SCRIPT_DIR" semantic_nav_mcp_server
