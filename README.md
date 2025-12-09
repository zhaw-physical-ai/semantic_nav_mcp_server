# Semantic Navigation MCP Server

An MCP (Model Context Protocol) server for semantic robot navigation with named locations. This server allows LLMs to interact with robots using human-readable location names instead of raw coordinates.

## Features

- **Semantic Location Management**: Register, query, and navigate to named locations
- **MCP Resources**: Expose location data as queryable resources
- **MCP Tools**:
  - `list_locations` - List all available locations
  - `get_location` - Get details about a specific location
  - `search_locations` - Search locations by name, zone, or metadata
  - `register_location` - Create or update a named location
  - `remove_location` - Remove a location from the database
  - `navigate_to_location` - Navigate robot to a named location
- **YAML Configuration**: Easy-to-edit waypoints file
- **ROS2 Integration**: Direct integration with Nav2 navigation stack
- **Extensible Metadata**: Attach custom metadata to locations (zones, access levels, etc.)

## Architecture

```
┌─────────────┐
│     LLM     │  ← Orchestrates navigation with semantic names
└──────┬──────┘
       │
┌──────▼──────────────────┐
│ Semantic Nav MCP Server │  ← This server
│  - Query locations      │
│  - Register locations   │
│  - Semantic navigation  │
└──────┬──────────────────┘
       │
┌──────▼──────┐
│ Nav2 Stack  │  ← Direct ROS2 action calls
└─────────────┘
```

## Installation

### Prerequisites

- ROS 2 (Jazzy or compatible distribution)
- Nav2 installed and configured
- Python 3.12
- `uv` package manager ([installation instructions](https://docs.astral.sh/uv/))

### Setup

1. **Clone or download this repository** to your desired location:
   ```bash
   cd /path/to/your/projects
   git clone <repository-url> semantic_nav_mcp_server
   # OR: extract from archive, etc.
   ```

2. **Install dependencies** (using uv with system Python for ROS2):
   ```bash
   cd semantic_nav_mcp_server
   UV_SYSTEM_PYTHON=1 uv sync
   ```

3. **Make the startup script executable**:
   ```bash
   chmod +x start_server.sh
   ```

## Configuration

### Waypoints File

Edit `waypoints.yaml` to define your semantic locations:

```yaml
waypoints:
  kitchen:
    position:
      x: 2.5
      y: 3.0
      z: 0.0
    orientation:
      yaw: 1.57  # radians
    metadata:
      description: "Kitchen area"
      zone: "food_prep"
      access_level: "public"

  charging_dock:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      yaw: 0.0
    metadata:
      description: "Charging station"
      type: "charger"
      dock_id: "dock_01"

config:
  map_frame: "map"
  base_frame: "base_link"
  default_tolerance: 0.1
```

### Environment Variables

The startup script supports these environment variables:

- `ROS_DISTRO`: ROS 2 distribution name (default: `jazzy`)
- `ROS_WORKSPACE`: Path to your ROS workspace (optional, for sourcing `install/setup.bash`)
- `SEMANTIC_NAV_WAYPOINTS_FILE`: Path to waypoints YAML file (default: `waypoints.yaml` in server directory)

### MCP Client Configuration

To use this server with an MCP client (like Claude Desktop), add it to your MCP configuration:

**For Claude Desktop** (`~/Library/Application Support/Claude/claude_desktop_config.json` on macOS, or `%APPDATA%\Claude\claude_desktop_config.json` on Windows, or `~/.config/Claude/claude_desktop_config.json` on Linux):

```json
{
  "mcpServers": {
    "semantic_nav": {
      "command": "/absolute/path/to/semantic_nav_mcp_server/start_server.sh",
      "env": {
        "ROS_DISTRO": "jazzy",
        "ROS_WORKSPACE": "/absolute/path/to/your/colcon_ws"
      }
    }
  }
}
```

**Important**: Replace `/absolute/path/to/` with the actual absolute paths on your system.

**Example with actual paths**:
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

**Note**: The `ROS_WORKSPACE` environment variable is optional. Only include it if you have a ROS workspace that needs to be sourced.

## Usage

### Starting the Server Standalone

For testing or manual use:

```bash
./start_server.sh
```

Or with custom environment:

```bash
ROS_DISTRO=humble ROS_WORKSPACE=/path/to/workspace ./start_server.sh
```

### MCP Tools Usage

#### List all locations
```
LLM: "Show me all available locations"
→ Calls: list_locations()
→ Returns: JSON list of location names
```

#### Get location details
```
LLM: "What are the coordinates of the kitchen?"
→ Calls: get_location(name="kitchen")
→ Returns: {x: 2.5, y: 3.0, yaw: 1.57} + metadata
```

#### Register a new location
```
LLM: "Save current position as waypoint_office at x=5, y=2, facing north"
→ Calls: register_location(name="waypoint_office", x=5.0, y=2.0, yaw=1.57, description="Office")
→ Updates: waypoints.yaml
```

#### Navigate to a location
```
LLM: "Go to the kitchen"
→ Calls: navigate_to_location(name="kitchen")
→ Internally: Looks up coordinates, calls Nav2 goToPose()
→ Returns: Navigation result
```

### MCP Resources

Resources provide queryable data about locations:

- `semantic://locations/all` - All locations with full details
- `semantic://locations/list` - Just the location names
- `semantic://locations/{name}` - Specific location details

## Comparison with nav2_mcp_server

| Feature | nav2_mcp_server | semantic_nav_mcp_server |
|---------|----------------|------------------------|
| **Abstraction** | Low-level coordinates | High-level named locations |
| **Navigation** | `navigate_to_pose(x, y, yaw)` | `navigate_to_location("kitchen")` |
| **Location Storage** | None | YAML file with persistence |
| **Metadata** | None | Zones, descriptions, custom fields |
| **Use Case** | Direct coordinate control | Semantic, human-friendly navigation |

## Multi-Agent Architecture

This server demonstrates a **non-hierarchical multi-agent pattern**:

- **LLM** = Orchestrator that sees both servers
- **semantic_nav_mcp_server** = High-level semantic layer (this server)
- **nav2_mcp_server** = Low-level coordinate control (optional, for advanced use)

The semantic server **internally** calls Nav2 directly via ROS2 actions, not through MCP. This keeps the architecture simple while providing semantic abstraction.

### Why Not Call nav2_mcp_server?

MCP servers don't call each other - the LLM orchestrates. However, we bypass nav2_mcp_server entirely and go directly to Nav2 because:

1. Semantic navigation owns the full navigation decision
2. No need for LLM to see low-level coordinate translation
3. Simpler architecture with one less dependency
4. Better encapsulation of semantic logic

You can still use **both** servers simultaneously:
- **semantic_nav_mcp_server**: For named location navigation ("go to kitchen")
- **nav2_mcp_server**: For advanced features (waypoint following, docking, path planning)

## Development

### Project Structure

```
semantic_nav_mcp_server/
├── src/semantic_nav_mcp_server/
│   ├── __init__.py           # Package initialization
│   ├── __main__.py           # Entry point
│   ├── server.py             # MCP server with tools and resources
│   ├── waypoint_manager.py   # Location database management
│   └── nav_client.py         # Nav2 ROS2 client
├── waypoints.yaml            # Location configuration
├── pyproject.toml            # Python project config
├── start_server.sh           # Startup script
└── README.md                 # This file
```

### Adding Custom Metadata

You can extend locations with arbitrary metadata:

```yaml
waypoints:
  lab_bench_3:
    position: {x: 10.0, y: 5.0, z: 0.0}
    orientation: {yaw: 0.0}
    metadata:
      description: "Lab bench 3"
      zone: "laboratory"
      equipment: ["microscope", "centrifuge"]
      responsible_person: "Dr. Smith"
      max_dwell_time: 300  # seconds
```

Then search by metadata:
```python
search_locations(zone="laboratory")
```

## Troubleshooting

### Server won't start
- Ensure ROS2 Jazzy is sourced: `source /opt/ros/jazzy/setup.bash`
- Check Nav2 is available: `ros2 pkg list | grep nav2`
- Verify waypoints file exists and is valid YAML

### Navigation fails
- Ensure Nav2 is running: `ros2 node list | grep nav`
- Check robot localization is active
- Verify map frame matches waypoints config

### Location not found
- Use `list_locations` to see available locations
- Check spelling of location name (case-sensitive)
- Verify waypoints.yaml is loaded correctly

## Future Enhancements

- [ ] Zone-based navigation (avoid zones, speed limits)
- [ ] Polygon regions (not just point locations)
- [ ] Path constraints through semantic areas
- [ ] Integration with Open-RMF building maps
- [ ] Dynamic location learning from user commands
- [ ] Multi-robot semantic coordination

## License

Apache License 2.0

## Contributing

Contributions welcome! This server demonstrates emerging patterns for semantic robot navigation with LLMs.
