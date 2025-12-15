#!/bin/bash
# Semantic Navigation MCP Server Startup Script (Remote/SSE Mode)
# This script starts the server with SSE transport for remote access

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

# Set host and port for SSE server
MCP_HOST="${MCP_HOST:-0.0.0.0}"
MCP_PORT="${MCP_PORT:-8000}"

echo "=========================================="
echo "Semantic Navigation MCP Server (Remote)"
echo "=========================================="
echo "Transport: SSE"
echo "Host: $MCP_HOST"
echo "Port: $MCP_PORT"
echo "Waypoints: $SEMANTIC_NAV_WAYPOINTS_FILE"
echo "=========================================="
echo ""
echo "Server will be accessible at: http://$MCP_HOST:$MCP_PORT"
echo "Press Ctrl+C to stop the server"
echo ""

# Run the MCP server with SSE transport for remote access
UV_SYSTEM_PYTHON=1 uv run --directory "$SCRIPT_DIR" semantic_nav_mcp_server --transport sse --host "$MCP_HOST" --port "$MCP_PORT"
