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

"""Waypoint management for semantic navigation."""

import json
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


class WaypointManager:
    """Manages semantic waypoints and their storage."""

    _instance: Optional['WaypointManager'] = None
    _lock = threading.Lock()

    def __init__(self, waypoints_file: str):
        """Initialize waypoint manager.

        Parameters
        ----------
        waypoints_file : str
            Path to the waypoints YAML file.
        """
        self.waypoints_file = Path(waypoints_file)
        self.waypoints: Dict[str, Dict[str, Any]] = {}
        self.config: Dict[str, Any] = {}
        self._load_waypoints()

    @classmethod
    def get_instance(cls, waypoints_file: Optional[str] = None) -> 'WaypointManager':
        """Get singleton instance of WaypointManager.

        Parameters
        ----------
        waypoints_file : Optional[str]
            Path to waypoints file (required on first call).

        Returns
        -------
        WaypointManager
            The singleton instance.
        """
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    if waypoints_file is None:
                        raise ValueError(
                            "waypoints_file required for first initialization"
                        )
                    cls._instance = cls(waypoints_file)
        return cls._instance

    def _load_waypoints(self) -> None:
        """Load waypoints from YAML file."""
        if not self.waypoints_file.exists():
            raise FileNotFoundError(
                f"Waypoints file not found: {self.waypoints_file}"
            )

        with open(self.waypoints_file, 'r') as f:
            data = yaml.safe_load(f)

        self.waypoints = data.get('waypoints', {})
        self.config = data.get('config', {})

    def _save_waypoints(self) -> None:
        """Save waypoints to YAML file."""
        data = {
            'waypoints': self.waypoints,
            'config': self.config
        }

        with open(self.waypoints_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    def list_waypoints(self) -> List[str]:
        """Get list of all waypoint names.

        Returns
        -------
        List[str]
            List of waypoint names.
        """
        return sorted(self.waypoints.keys())

    def get_waypoint(self, name: str) -> Optional[Dict[str, Any]]:
        """Get waypoint data by name.

        Parameters
        ----------
        name : str
            Name of the waypoint.

        Returns
        -------
        Optional[Dict[str, Any]]
            Waypoint data or None if not found.
        """
        return self.waypoints.get(name)

    def get_all_waypoints(self) -> Dict[str, Dict[str, Any]]:
        """Get all waypoints.

        Returns
        -------
        Dict[str, Dict[str, Any]]
            Dictionary of all waypoints.
        """
        return self.waypoints.copy()

    def add_waypoint(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float,
        metadata: Optional[Dict[str, Any]] = None
    ) -> bool:
        """Add or update a waypoint.

        Parameters
        ----------
        name : str
            Name of the waypoint.
        x : float
            X coordinate in map frame.
        y : float
            Y coordinate in map frame.
        yaw : float
            Orientation in radians.
        metadata : Optional[Dict[str, Any]]
            Additional metadata for the waypoint.

        Returns
        -------
        bool
            True if waypoint was added/updated successfully.
        """
        waypoint_data = {
            'position': {
                'x': float(x),
                'y': float(y),
                'z': 0.0
            },
            'orientation': {
                'yaw': float(yaw)
            },
            'metadata': metadata or {}
        }

        self.waypoints[name] = waypoint_data
        self._save_waypoints()
        return True

    def remove_waypoint(self, name: str) -> bool:
        """Remove a waypoint.

        Parameters
        ----------
        name : str
            Name of the waypoint to remove.

        Returns
        -------
        bool
            True if waypoint was removed, False if not found.
        """
        if name in self.waypoints:
            del self.waypoints[name]
            self._save_waypoints()
            return True
        return False

    def search_waypoints(
        self,
        query: Optional[str] = None,
        zone: Optional[str] = None,
        metadata_filter: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Dict[str, Any]]:
        """Search waypoints by criteria.

        Parameters
        ----------
        query : Optional[str]
            Text to search in name and description.
        zone : Optional[str]
            Filter by zone metadata.
        metadata_filter : Optional[Dict[str, Any]]
            Filter by specific metadata key-value pairs.

        Returns
        -------
        Dict[str, Dict[str, Any]]
            Dictionary of matching waypoints.
        """
        results = {}

        for name, data in self.waypoints.items():
            # Text search
            if query:
                query_lower = query.lower()
                if query_lower not in name.lower():
                    desc = data.get('metadata', {}).get('description', '')
                    if query_lower not in desc.lower():
                        continue

            # Zone filter
            if zone:
                if data.get('metadata', {}).get('zone') != zone:
                    continue

            # Metadata filter
            if metadata_filter:
                metadata = data.get('metadata', {})
                if not all(
                    metadata.get(k) == v for k, v in metadata_filter.items()
                ):
                    continue

            results[name] = data

        return results

    def get_config(self, key: str, default: Any = None) -> Any:
        """Get configuration value.

        Parameters
        ----------
        key : str
            Configuration key.
        default : Any
            Default value if key not found.

        Returns
        -------
        Any
            Configuration value.
        """
        return self.config.get(key, default)
