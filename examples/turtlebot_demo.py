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


import rclpy
import rclpy.callback_groups
import rclpy.executors
import rclpy.qos
import rclpy.subscription
import rclpy.task
from langchain.tools.render import render_text_description_and_args
from langchain_openai import ChatOpenAI

from rai.agents.state_based import create_state_based_agent
from rai.node import RaiNode
from rai.tools.ros.native import (
    GetCameraImage,
    GetMsgFromTopic,
    Ros2PubMessageTool,
    Ros2ShowMsgInterfaceTool,
)
from rai.tools.ros.native_actions import Ros2RunActionSync
from rai.tools.ros.tools import GetCurrentPositionTool
from rai.tools.time import WaitForSecondsTool


def main():
    rclpy.init()
    llm = ChatOpenAI(model="gpt-4o")

    # TODO(boczekbartek): refactor system prompt

    SYSTEM_PROMPT = ""

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

    node = RaiNode(
        llm=ChatOpenAI(
            model="gpt-4o-mini"
        ),  # smaller model used to describe the environment
        whitelist=ros2_whitelist,
        system_prompt=SYSTEM_PROMPT,
    )

    tools = [
        WaitForSecondsTool(),
        GetMsgFromTopic(node=node),
        GetCameraImage(node=node),
        Ros2ShowMsgInterfaceTool(),
        Ros2PubMessageTool(node=node),
        Ros2RunActionSync(node=node),
        GetCurrentPositionTool(),
    ]

    state_retriever = node.get_robot_state

    SYSTEM_PROMPT = f"""You are an autonomous robot connected to ros2 environment. Your main goal is to fulfill the user's requests.
    Do not make assumptions about the environment you are currently in.

    Here are your tools:
    {render_text_description_and_args(tools)}

    You can use ros2 topics, services and actions to operate.

    Navigation tips:
    - for driving forward/backward, if specified, ros2 actions are better.
    - for driving for some specific time or in specific manner (like in circle) it good to use /cmd_vel topic
    - you are currently unable to read map or point-cloud, so please avoid subscribing to such topics.
    - if you are asked to drive towards some object, it's good to:
        1. check the camera image and verify if objects can be seen
        2. if only driving forward is required, do it
        3. if obstacle avoidance might be required, use ros2 actions navigate_*, but first check your currect position, then very accurately estimate the goal pose.
    """

    node.get_logger().info(f"{SYSTEM_PROMPT=}")

    node.system_prompt = node.initialize_system_prompt(SYSTEM_PROMPT)

    app = create_state_based_agent(
        llm=llm,
        tools=tools,
        state_retriever=state_retriever,
        logger=node.get_logger(),
    )

    node.set_app(app)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


main()
