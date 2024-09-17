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

import logging
import threading
import time
from queue import Queue
from typing import Optional

import rclpy
from langchain_core.messages import HumanMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from rai.node import RaiBaseNode
from rai_hmi.agent import initialize_agent
from rai_hmi.base import BaseHMINode
from rai_hmi.text_hmi_utils import Memory

logger = logging.getLogger(__name__)


class VoiceApp:
    def __init__(self) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)

        self.memory = Memory()

        self.mission_queue = Queue()
        self.executor_thread = None
        self.voice_hmi_ros_node, self.rai_ros_node = self.initialize_ros_nodes()
        self.agent = initialize_agent(
            self.voice_hmi_ros_node, self.rai_ros_node, self.memory
        )
        self.voice_hmi_ros_node.set_agent(self.agent)

    def initialize_ros_nodes(self):
        rclpy.init()
        voice_hmi_node = VoiceHMINode(
            "voice_hmi_node",
            queue=self.mission_queue,
        )

        # TODO(boczekbartek): this node shouldn't be required to initialize simple ros2 tools
        rai_node = RaiBaseNode(node_name="__rai_node__")

        executor = MultiThreadedExecutor()
        executor.add_node(voice_hmi_node)
        executor.add_node(rai_node)

        self.executor_thread = threading.Thread(
            target=executor.spin, daemon=True
        ).start()
        return voice_hmi_node, rai_node

    def run(self):
        while True:
            if self.mission_queue.empty():
                time.sleep(0.5)
                continue
            logger.info("Got new mission update!")
            msg = self.mission_queue.get()
            self.memory.add_mission(msg)


class VoiceHMINode(BaseHMINode):
    def __init__(
        self,
        node_name: str,
        queue: Queue,
        robot_description_package: Optional[str] = None,
    ):
        super().__init__(node_name, queue, robot_description_package)

        self.callback_group = ReentrantCallbackGroup()
        self.hmi_subscription = self.create_subscription(
            String,
            "from_human",
            self.handle_human_message,
            10,
        )

        self.hmi_publisher = self.create_publisher(
            String, "to_human", 10, callback_group=self.callback_group
        )
        self.history = []

        self.get_logger().info("Voice HMI node initialized")

    def set_agent(self, agent):
        self.agent = agent

    def handle_human_message(self, msg: String):
        self.processing = True
        self.get_logger().info("Processing started")

        # handle human message
        self.history.append(HumanMessage(content=msg.data))

        for state in self.agent.stream(dict(messages=self.history)):
            node_name = list(state.keys())[0]
            if node_name == "thinker":
                last_message = state[node_name]["messages"][-1].content
                if last_message != "":
                    self.get_logger().info(
                        f'Sending message to human: "{last_message}"'
                    )
                    self.hmi_publisher.publish(String(data=last_message))

        self.get_logger().info("Processing finished")
        self.processing = False


def main():
    app = VoiceApp()
    app.run()
