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

import uuid
from typing import List

from langchain.tools import tool

from rai.agents.conversational_agent import create_conversational_agent
from rai.node import RaiBaseNode
from rai.tools.ros.native import GetCameraImage, Ros2GetRobotInterfaces
from rai.utils.model_initialization import get_llm_model
from rai_hmi.base import BaseHMINode
from rai_hmi.chat_msgs import MissionMessage
from rai_hmi.task import Task, TaskInput
from rai_hmi.text_hmi_utils import Memory


def initialize_agent(hmi_node: BaseHMINode, rai_node: RaiBaseNode, memory: Memory):
    llm = get_llm_model(model_type="complex_model")

    @tool
    def get_mission_memory(uid: str) -> List[MissionMessage]:
        """List mission memory. Mission uid is required."""
        return memory.get_mission_memory(uid)

    @tool
    def add_task_to_queue(task: TaskInput):
        """Use this tool to add a task to the queue. The task will be handled by the executor part of your system."""

        uid = uuid.uuid1()
        hmi_node.add_task_to_queue(
            Task(
                name=task.name,
                description=task.description,
                priority=task.priority,
                uid=uid,
            )
        )
        return f"UID={uid} | Task added to the queue: {task.json()}"

    node_tools = tools = [
        Ros2GetRobotInterfaces(node=rai_node),
        GetCameraImage(node=rai_node),
    ]
    task_tools = [add_task_to_queue, get_mission_memory]
    tools = hmi_node.tools + node_tools + task_tools

    agent = create_conversational_agent(
        llm, tools, hmi_node.system_prompt, logger=hmi_node.get_logger()
    )
    return agent
