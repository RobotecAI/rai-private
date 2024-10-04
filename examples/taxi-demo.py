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

from queue import Queue
from typing import List

import rclpy
from langchain_community.tools import GooglePlacesTool
from langchain_community.tools.tavily_search import TavilySearchResults
from langchain_core.messages import BaseMessage, HumanMessage
from langchain_core.tools import tool
from std_msgs.msg import String

from rai.agents.conversational_agent import create_conversational_agent
from rai.tools.ros.cli import Ros2ServiceTool
from rai.tools.ros.native import Ros2PubMessageTool
from rai.utils.model_initialization import get_llm_model, get_tracing_callbacks
from rai_hmi.api import GenericVoiceNode, split_message

system_prompt = """
**System Role: Taxi Driver in Warsaw**

- **User Instructions**: You will be provided with a destination by the user, which may either be a specific place or an address. Sometimes, the user might describe the destination in a way that isn't clearly a place or address.

- **Clarifying the Destination**: If the destination isn't immediately clear, your task is to ask clarifying questions to determine where the user wants to go. Once confirmed, ensure you obtain the exact address (including street name, number, etc.) to send to the navigation system.

- **Location Context**: You are based in Warsaw, Poland, and your communication with the user must always be in English.

- **Tools**:
    - **tavily_search_results_json**: Use this tool to find an address when the user provides a non-specific description of a destination.
    - **navigate**: Once the exact address is confirmed, use this to send the destination to the navigation system.
    - **google_places**: Use this tool to search for specific places, businesses, or landmarks based on user descriptions. It can help if the user mentions popular destinations or well-known places in Warsaw.

- **Communication Style**: Be friendly, helpful, and concise. While you may receive greetings or unrelated questions, keep the conversation focused on resolving the user's destination.

- **Key Directives**:
    - Do not guess or assume information; rely on tools to obtain any needed details.
    - Your primary goal is to successfully navigate to the destination provided by the user.
    - If you are sure about the destination, please try to resolve without additional interaction with the client.
"""


class TaxiDemo(GenericVoiceNode):
    def __init__(self):
        super().__init__("taxi_demo_node", Queue(), "")

        @tool
        def navigate(street: str, number: int, city: str, country: str) -> str:
            """
            Send the destination to the Autoware system.
            This is a mock example, so the goal pose is hardcoded.
            In a real implementation, the goal pose should be retrieved from maps vendor (latitude and longitude)
            and then converted to a pose in the map frame.
            """
            autoware_navigate_tool = Ros2PubMessageTool(node=self)
            autoware_mode_tool = Ros2ServiceTool()
            autoware_navigate_tool.run(
                {
                    "topic_name": "/planning/mission_planning/goal",
                    "msg_type": "geometry_msgs/msg/PoseStamped",
                    "msg_args": {
                        "header": {"frame_id": "base_link"},
                        "pose": {
                            "position": {
                                "x": 5.0,
                                "y": 0.0,
                                "z": 0.0,
                            },  # since this is a mock example, we are using a position in front of the vehicle
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        },
                    },
                }
            )
            autoware_mode_tool.run(
                tool_input={
                    "command": "call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode"
                }
            )
            return f"Navigating to {street} {number}, {city}, {country}"

        self.agent = create_conversational_agent(
            get_llm_model("complex_model"),
            [navigate, GooglePlacesTool(), TavilySearchResults()],
            system_prompt,
            logger=self.get_logger(),
        )

        self.history: List[BaseMessage] = []

    def _handle_human_message(self, msg: String):
        self.history.append(HumanMessage(content=msg.data))
        response = self.agent.invoke(
            {"messages": self.history}, config={"callbacks": get_tracing_callbacks()}
        )
        last_message = response["messages"][-1].content
        for sentence in split_message(last_message):
            self.hmi_publisher.publish(String(data=sentence))


def main():
    rclpy.init()
    node = TaxiDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
