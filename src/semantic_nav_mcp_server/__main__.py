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

"""Main entry point for Semantic Navigation MCP Server."""

import asyncio
import rclpy

from .server import mcp


async def main_async() -> None:
    """Run the Semantic Navigation MCP Server (async)."""
    # Initialize ROS 2
    rclpy.init()

    try:
        # Run the MCP server using async transport
        await mcp.run_async(transport='stdio')
    finally:
        # Cleanup ROS 2
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    """Entry point wrapper."""
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
