# Copyright (C) 2024 Robotec.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import base64
import json
import logging
import time
from typing import Any, Dict, Type, cast

import cv2
import numpy as np
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from langchain_core.pydantic_v1 import BaseModel, Field
from langchain_core.tools import BaseTool
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion

from rai.tools.ros.deprecated import SingleMessageGrabber
from rai.tools.utils import TF2TransformFetcher

from .native import TopicInput

logger = logging.getLogger(__name__)


class AddDescribedWaypointToDatabaseToolInput(BaseModel):
    """Input for the add described waypoint to database tool."""

    x: float = Field(..., description="X coordinate of the waypoint")
    y: float = Field(..., description="Y coordinate of the waypoint")
    z: float = Field(0.0, description="Z coordinate of the waypoint")
    text: str = Field(
        ...,
        description="Text to display on the waypoint (very short, one or two words)",
    )


class AddDescribedWaypointToDatabaseTool(BaseTool):
    """Add described waypoint to the database tool."""

    name = "AddDescribedWaypointToDatabaseTool"
    description: str = (
        "A tool for adding a described waypoint to the database for later use. "
    )

    args_schema: Type[AddDescribedWaypointToDatabaseToolInput] = (
        AddDescribedWaypointToDatabaseToolInput
    )

    map_database: str = ""

    def _run(self, x: float, y: float, z: float = 0.0, text: str = ""):
        try:
            self.update_map_database(x, y, z, text)
        except FileNotFoundError:
            logger.warn(f"Database {self.map_database} not found.")
        return {"content": "Waypoint added successfully"}

    def update_map_database(
        self,
        x: float,
        y: float,
        z: float,
        text: str,
        frame_id: str = "map",
        child_frame_id: str = "base_link",
    ):
        with open(self.map_database, "r") as file:
            map_database = json.load(file)

        data: Dict[str, Any] = {
            "header": {"frame_id": frame_id, "stamp": time.time()},
            "child_frame_id": child_frame_id,
            "transform": {
                "translation": {"x": x, "y": y, "z": z},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "text": text,
        }
        map_database.append(data)

        with open(self.map_database, "w") as file:
            json.dump(map_database, file, indent=2)


class GetOccupancyGridTool(BaseTool):
    """Get the current map as an image with the robot's position marked on it (red dot)."""

    name: str = "GetOccupancyGridTool"
    description: str = (
        "A tool for getting the current map as an image with the robot's position marked on it."
    )

    args_schema: Type[TopicInput] = TopicInput

    image_width: int = 1500
    debug: bool = False

    def _postprocess_msg(self, map_msg: OccupancyGrid, transform: TransformStamped):
        width = cast(int, map_msg.info.width)
        height = cast(int, map_msg.info.height)
        resolution = cast(float, map_msg.info.resolution)
        origin_position = cast(Point, map_msg.info.origin.position)
        origin_orientation = cast(Quaternion, map_msg.info.origin.orientation)

        data = np.array(map_msg.data).reshape((height, width))

        # Convert the OccupancyGrid values to grayscale image (0-255)
        # the final image shape is (self.image_width, self.image_width), scale to fit
        scale = self.image_width / max(width, height)
        width = int(width * scale)
        height = int(height * scale)
        data = cv2.resize(data, (width, height), interpolation=cv2.INTER_NEAREST)
        resolution = resolution / scale
        image = np.zeros_like(data, dtype=np.uint8)
        image[data == -1] = 127  # Unknown space
        image[data == 0] = 255  # Free space
        image[data > 0] = 0  # Occupied space

        # Calculate robot's position in the image
        robot_x = cast(
            float, (transform.transform.translation.x - origin_position.x) / resolution
        )

        robot_y = cast(
            float, (transform.transform.translation.y - origin_position.y) / resolution
        )

        _, _, yaw = euler_from_quaternion(
            [
                origin_orientation.x,  # type: ignore
                origin_orientation.y,  # type: ignore
                origin_orientation.z,  # type: ignore
                origin_orientation.w,  # type: ignore
            ]
        )
        # Rotate the robot's position based on the yaw angle
        rotated_x = robot_x * np.cos(yaw) - robot_y * np.sin(yaw)
        rotated_y = robot_x * np.sin(yaw) + robot_y * np.cos(yaw)
        robot_x = int(rotated_x)
        robot_y = int(rotated_y)

        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        # Draw the robot's position as an arrow
        if 0 <= robot_x < width and 0 <= robot_y < height:
            _, _, yaw = euler_from_quaternion(
                [
                    transform.transform.rotation.x,  # type: ignore
                    transform.transform.rotation.y,  # type: ignore
                    transform.transform.rotation.z,  # type: ignore
                    transform.transform.rotation.w,  # type: ignore
                ]
            )
            arrow_length = 100
            arrow_end_x = int(robot_x + arrow_length * np.cos(yaw))
            arrow_end_y = int(robot_y + arrow_length * np.sin(yaw))
            cv2.arrowedLine(
                image, (robot_x, robot_y), (arrow_end_x, arrow_end_y), (0, 0, 255), 5
            )

        image = cv2.flip(image, 1)

        step_size_m: float = 2.0  # Step size for grid lines in meters, adjust as needed
        step_size_pixels = int(step_size_m / resolution)
        # print(step_size_pixels, scale)
        for x in range(0, width, step_size_pixels):
            cv2.line(
                img=image,
                pt1=(x, 0),
                pt2=(x, height),
                color=(200, 200, 200),
                thickness=1,
            )
            cv2.putText(
                img=image,
                text=f"{x * resolution + origin_position.x:.1f}",
                org=(x, 30),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1.0,
                color=(0, 0, 0),
                thickness=1,
                lineType=cv2.LINE_AA,
            )
        for y in range(0, height, step_size_pixels):
            cv2.line(
                img=image,
                pt1=(0, y),
                pt2=(width, y),
                color=(200, 200, 200),
                thickness=1,
            )
            cv2.putText(
                img=image,
                text=f"{y * resolution + origin_position.y:.1f}",
                org=(15, y + 35),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1.0,
                color=(0, 0, 0),
                thickness=1,
                lineType=cv2.LINE_AA,
            )
        # Encode into PNG base64
        _, buffer = cv2.imencode(".png", image)
        return image

        if self.debug:
            cv2.imwrite("map.png", image)
        return base64.b64encode(buffer.tobytes()).decode("utf-8")

    def _run(self, topic_name: str):
        """Gets the current map from the specified topic."""
        map_grabber = SingleMessageGrabber(topic_name, OccupancyGrid, timeout_sec=10)
        tf_grabber = TF2TransformFetcher()

        map_msg = map_grabber.get_data()
        transform = tf_grabber.get_data()

        if map_msg is None or transform is None:
            return {"content": "Failed to get the map, wrong topic?"}

        base64_image = self._postprocess_msg(map_msg, transform)
        return {"content": "Map grabbed successfully", "images": [base64_image]}


class GetCurrentPositionToolInput(BaseModel):
    """Input for the get_current_position tool."""


class GetCurrentPositionTool(BaseTool):
    """Get the current position of the robot."""

    name = "GetCurrentPositionTool"
    description: str = "A tool for getting the current position of the robot."

    args_schema: Type[GetCurrentPositionToolInput] = GetCurrentPositionToolInput

    def _run(self):
        """Gets the current position from the specified topic."""
        tf_grabber = TF2TransformFetcher()
        transform_stamped = tf_grabber.get_data()
        position = transform_stamped.transform.translation  # type: ignore
        orientation = transform_stamped.transform.rotation  # type: ignore
        _, _, yaw = euler_from_quaternion(
            [
                orientation.x,  # type: ignore
                orientation.y,  # type: ignore
                orientation.z,  # type: ignore
                orientation.w,  # type: ignore
            ]
        )
        return {
            "content": str(
                {
                    "x": position.x,  # type: ignore
                    "y": position.y,  # type: ignore
                    "z": position.z,  # type: ignore
                    "yaw": yaw,
                }
            ),
        }
