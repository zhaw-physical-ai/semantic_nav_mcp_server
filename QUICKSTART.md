# Quick Start Guide

Get your semantic navigation MCP server running in 5 minutes!

## Step 1: Install Prerequisites

```bash
# Install uv package manager (if not already installed)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Ensure ROS 2 and Nav2 are installed
# For Ubuntu 24.04:
# sudo apt install ros-jazzy-desktop ros-jazzy-navigation2
```

## Step 2: Setup the Server

```bash
# Navigate to where you want to install
cd ~/projects  # or your preferred location

# If cloning from git:
git clone <repository-url> semantic_nav_mcp_server
# OR if extracting from archive:
# tar -xzf semantic_nav_mcp_server.tar.gz

cd semantic_nav_mcp_server

# Install Python dependencies
UV_SYSTEM_PYTHON=1 uv sync

# Make startup script executable
chmod +x start_server.sh
```

## Step 3: Configure Your Locations

Edit `waypoints.yaml` to add your robot's named locations:

```yaml
waypoints:
  home:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {yaw: 0.0}
    metadata:
      description: "Home position"

  kitchen:
    position: {x: 2.5, y: 3.0, z: 0.0}
    orientation: {yaw: 1.57}  # facing north
    metadata:
      description: "Kitchen area"
      zone: "food_prep"
```

## Step 4: Configure Your MCP Client

### Option A: Local Access (Claude Desktop on same machine)

Find your config file location:
- **macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
- **Windows**: `%APPDATA%\Claude\claude_desktop_config.json`
- **Linux**: `~/.config/Claude/claude_desktop_config.json`

Add this configuration (replace paths with your actual paths):

```json
{
  "mcpServers": {
    "semantic_nav": {
      "command": "/home/youruser/projects/semantic_nav_mcp_server/start_server.sh",
      "env": {
        "ROS_DISTRO": "jazzy"
      }
    }
  }
}
```

**If you have a ROS workspace**, add the workspace path:

```json
{
  "mcpServers": {
    "semantic_nav": {
      "command": "/home/youruser/projects/semantic_nav_mcp_server/start_server.sh",
      "env": {
        "ROS_DISTRO": "jazzy",
        "ROS_WORKSPACE": "/home/youruser/colcon_ws"
      }
    }
  }
}
```

### Option B: Remote Access (Claude Desktop on different machine)

For remote access, you'll run the server with SSE transport and connect via HTTP.

**On the robot/server machine:**

```bash
# Start the server in remote mode
cd /home/youruser/projects/semantic_nav_mcp_server
./start_server_remote.sh
```

This starts the server on `http://0.0.0.0:8000` by default. You can customize with environment variables:

```bash
# Custom host and port
MCP_HOST=192.168.1.100 MCP_PORT=9000 ./start_server_remote.sh
```

**On the client machine (Claude Desktop):**

Add this to your Claude Desktop config:

```json
{
  "mcpServers": {
    "semantic_nav": {
      "url": "http://192.168.1.100:8000/sse",
      "transport": "sse"
    }
  }
}
```

Replace `192.168.1.100:8000` with your robot's IP address and port.

## Step 5: Test the Server

### Manual Test (Local Mode)

```bash
# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Run the server
./start_server.sh
```

You should see the MCP server start up. Press Ctrl+C to stop.

### Manual Test (Remote Mode)

```bash
# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Run the server in remote mode
./start_server_remote.sh
```

You should see output like:
```
==========================================
Semantic Navigation MCP Server (Remote)
==========================================
Transport: SSE
Host: 0.0.0.0
Port: 8000
Waypoints: /path/to/waypoints.yaml
==========================================

Server will be accessible at: http://0.0.0.0:8000
```

The server is now accessible remotely. Press Ctrl+C to stop.

### Test with Claude Desktop

1. Restart Claude Desktop
2. Start a new conversation
3. Try these commands:
   - "List all available locations"
   - "Show me the coordinates of the kitchen"
   - "Navigate to home"

## Common Issues

### Issue: "ROS 2 jazzy not found"
**Solution**: Either install ROS 2 Jazzy, or set `ROS_DISTRO` to your installed version (e.g., `"ROS_DISTRO": "humble"`).

### Issue: "Command not found: uv"
**Solution**: Install uv package manager: `curl -LsSf https://astral.sh/uv/install.sh | sh`

### Issue: "waypoints.yaml not found"
**Solution**: Ensure you're running from the server directory, or set `SEMANTIC_NAV_WAYPOINTS_FILE` environment variable.

### Issue: Navigation fails
**Solution**:
1. Ensure Nav2 is running: `ros2 node list | grep nav`
2. Check robot is localized
3. Verify map frame in `waypoints.yaml` matches your robot's configuration

## Next Steps

- Read the full [README.md](README.md) for advanced features
- Add custom metadata to your locations
- Use `register_location` tool to save new positions dynamically
- Search locations by zone with `search_locations`

## Example Conversation Flow

```
You: "What locations are available?"
Claude: *calls list_locations()*
Claude: "There are 2 locations: home, kitchen"

You: "Go to the kitchen"
Claude: *calls navigate_to_location(name="kitchen")*
Claude: "Successfully navigated to kitchen at coordinates (2.5, 3.0)"

You: "Register this position as 'office' at x=5, y=2, facing east"
Claude: *calls register_location(name="office", x=5.0, y=2.0, yaw=0.0)*
Claude: "Location 'office' has been registered successfully"
```

Happy navigating! 🤖
