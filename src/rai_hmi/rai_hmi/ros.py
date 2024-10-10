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

import threading
from queue import Queue
from typing import Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor

from rai.node import RaiBaseNode
from rai_hmi.base import BaseHMINode


def initialize_ros_nodes(
    _feedbacks_queue: Queue, robot_description_package: Optional[str]
) -> Tuple[BaseHMINode, RaiBaseNode]:
    rclpy.init()

    hmi_node = BaseHMINode(
        f"{robot_description_package}_hmi_node",
        queue=_feedbacks_queue,
        robot_description_package=robot_description_package,
    )

    # TODO(boczekbartek): this node shouldn't be required to initialize simple ros2 tools
    ros2_whitelist = [
        "/cmd_vel",
        "/rosout",
        "/map",
        "/odom",
        "/camera_image_color",
        "/backup",
        "/drive_on_heading",
        "/navigate_through_poses",
        "/navigate_to_pose",
        "/spin",
    ]

    rai_node = RaiBaseNode(node_name="__rai_node__", whitelist=ros2_whitelist)

    executor = MultiThreadedExecutor()
    executor.add_node(hmi_node)
    executor.add_node(rai_node)

    threading.Thread(target=executor.spin, daemon=True).start()

    return hmi_node, rai_node
