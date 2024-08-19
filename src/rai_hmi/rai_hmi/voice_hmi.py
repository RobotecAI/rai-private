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

from typing import List

import rclpy
from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain.prompts import ChatPromptTemplate
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage
from langchain_openai import ChatOpenAI
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from rai_hmi.base import BaseHMINode
from rai_interfaces.action import TaskFeedback


class VoiceHMINode(BaseHMINode):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.callback_group = ReentrantCallbackGroup()
        self.hmi_subscription = self.create_subscription(
            String,
            "from_human",
            self.handle_human_message,
            10,
            callback_group=self.callback_group,
        )

        self.hmi_publisher = self.create_publisher(
            String, "to_human", 10, callback_group=self.callback_group
        )

        self.history: List[BaseMessage] = []
        self.agent = self.initialize_agent()

        self.get_logger().info("Voice HMI node initialized")

    def initialize_agent(self):
        prompt = ChatPromptTemplate.from_messages(
            [
                ("system", self.system_prompt),
                ("placeholder", "{chat_history}"),
                ("human", "{user_input}"),
                ("placeholder", "{agent_scratchpad}"),
            ]
        )
        llm = ChatOpenAI(model="gpt-4o")
        agent = create_tool_calling_agent(llm=llm, tools=self.tools, prompt=prompt)
        agent_executor = AgentExecutor(agent=agent, tools=self.tools)
        return agent_executor

    def handle_human_message(self, msg: String):
        self.processing = True

        # handle human message
        response = self.agent.invoke(
            {"user_input": msg.data, "chat_history": self.history}
        )
        output = response["output"]
        self.history.append(HumanMessage(msg.data))
        self.history.append(AIMessage(output))

        self.hmi_publisher.publish(String(data=output))
        self.processing = False

    # TODO: Implement
    def handle_task_feedback_request(self, goal_handle: ServerGoalHandle):

        # Extract goal data
        goal: TaskFeedback.Goal = goal_handle.request
        goal  # type: ignore

        # Handle the goal.current_task and goal.issue_description
        for _ in range(10):
            goal_handle.publish_feedback(
                TaskFeedback.Feedback(feedback="Processing...")
            )

        goal_handle.succeed()
        # Create and return a result
        result = TaskFeedback.Result()
        result.informations = "Do this and that..."
        result.success = True

        return result


def main(args=None):
    rclpy.init(args=args)
    voice_hmi_node = VoiceHMINode("voice_hmi_node")
    rclpy.spin(voice_hmi_node)
    voice_hmi_node.destroy_node()
    rclpy.shutdown()
