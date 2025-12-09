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

"""Tests for WaypointManager."""

import tempfile
from pathlib import Path

import pytest
import yaml


def create_test_waypoints_file():
    """Create a temporary waypoints file for testing."""
    waypoints_data = {
        'waypoints': {
            'test_location': {
                'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
                'orientation': {'yaw': 0.5},
                'metadata': {
                    'description': 'Test location',
                    'zone': 'test_zone'
                }
            }
        },
        'config': {
            'map_frame': 'map',
            'base_frame': 'base_link'
        }
    }

    temp_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False
    )
    yaml.dump(waypoints_data, temp_file, default_flow_style=False)
    temp_file.close()

    return temp_file.name


@pytest.fixture
def waypoints_file():
    """Fixture to provide a test waypoints file."""
    file_path = create_test_waypoints_file()
    yield file_path
    # Cleanup
    Path(file_path).unlink(missing_ok=True)


def test_load_waypoints(waypoints_file):
    """Test loading waypoints from file."""
    from semantic_nav_mcp_server.waypoint_manager import WaypointManager

    # Reset singleton for testing
    WaypointManager._instance = None

    manager = WaypointManager.get_instance(waypoints_file)

    assert 'test_location' in manager.waypoints
    assert manager.waypoints['test_location']['position']['x'] == 1.0


def test_list_waypoints(waypoints_file):
    """Test listing waypoint names."""
    from semantic_nav_mcp_server.waypoint_manager import WaypointManager

    WaypointManager._instance = None
    manager = WaypointManager.get_instance(waypoints_file)

    waypoints = manager.list_waypoints()
    assert 'test_location' in waypoints
    assert len(waypoints) == 1


def test_get_waypoint(waypoints_file):
    """Test getting a specific waypoint."""
    from semantic_nav_mcp_server.waypoint_manager import WaypointManager

    WaypointManager._instance = None
    manager = WaypointManager.get_instance(waypoints_file)

    waypoint = manager.get_waypoint('test_location')
    assert waypoint is not None
    assert waypoint['position']['x'] == 1.0
    assert waypoint['metadata']['zone'] == 'test_zone'

    # Test non-existent waypoint
    assert manager.get_waypoint('nonexistent') is None


def test_add_waypoint(waypoints_file):
    """Test adding a new waypoint."""
    from semantic_nav_mcp_server.waypoint_manager import WaypointManager

    WaypointManager._instance = None
    manager = WaypointManager.get_instance(waypoints_file)

    result = manager.add_waypoint(
        name='new_location',
        x=5.0,
        y=10.0,
        yaw=1.57,
        metadata={'description': 'New test location', 'zone': 'new_zone'}
    )

    assert result is True
    assert 'new_location' in manager.waypoints

    waypoint = manager.get_waypoint('new_location')
    assert waypoint['position']['x'] == 5.0
    assert waypoint['position']['y'] == 10.0
    assert waypoint['orientation']['yaw'] == 1.57
    assert waypoint['metadata']['description'] == 'New test location'


def test_remove_waypoint(waypoints_file):
    """Test removing a waypoint."""
    from semantic_nav_mcp_server.waypoint_manager import WaypointManager

    WaypointManager._instance = None
    manager = WaypointManager.get_instance(waypoints_file)

    # Remove existing waypoint
    result = manager.remove_waypoint('test_location')
    assert result is True
    assert 'test_location' not in manager.waypoints

    # Try to remove non-existent waypoint
    result = manager.remove_waypoint('nonexistent')
    assert result is False


def test_search_waypoints(waypoints_file):
    """Test searching waypoints."""
    from semantic_nav_mcp_server.waypoint_manager import WaypointManager

    WaypointManager._instance = None
    manager = WaypointManager.get_instance(waypoints_file)

    # Add more waypoints for testing
    manager.add_waypoint(
        'kitchen', 2.0, 3.0, 0.0,
        metadata={'description': 'Kitchen area', 'zone': 'food_prep'}
    )
    manager.add_waypoint(
        'office', 5.0, 2.0, 0.0,
        metadata={'description': 'Office space', 'zone': 'work'}
    )

    # Search by query
    results = manager.search_waypoints(query='kitchen')
    assert 'kitchen' in results
    assert len(results) == 1

    # Search by zone
    results = manager.search_waypoints(zone='test_zone')
    assert 'test_location' in results

    # Search with no matches
    results = manager.search_waypoints(query='nonexistent')
    assert len(results) == 0


def test_get_config(waypoints_file):
    """Test getting configuration values."""
    from semantic_nav_mcp_server.waypoint_manager import WaypointManager

    WaypointManager._instance = None
    manager = WaypointManager.get_instance(waypoints_file)

    assert manager.get_config('map_frame') == 'map'
    assert manager.get_config('base_frame') == 'base_link'
    assert manager.get_config('nonexistent', 'default') == 'default'
