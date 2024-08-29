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

import asyncio
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain.callbacks.base import BaseCallbackHandler
from langchain.tools import tool
from langchain_community.vectorstores import FAISS
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI, OpenAIEmbeddings
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rai.messages import HumanMultimodalMessage
from rai.tools.ros.native import (
    GetCameraImage,
    GetMsgFromTopic,
    Ros2GetTopicsNamesAndTypesTool,
    Ros2PubMessageTool,
)
from rai_hmi.task import Task
from rai_hmi.tools import QueryDatabaseTool, QueueTaskTool
from rai_interfaces.srv import VectorStoreRetrieval


@tool
def get_current_image(topic: str) -> str:
    """Use this tool to get an image from a camera topic"""
    try:
        llm = ChatOpenAI(model="gpt-4o-mini", streaming=True)
        output = GetCameraImage()._run(topic_name=topic)
        images = output["images"]
        msg = HumanMultimodalMessage(
            content="Please describe the image as thoroughly as possible. Reply with I see...",
            images=images,
            name="tool",
        )
        return llm.invoke([msg]).content
    except Exception as e:
        return f"Error: {e}. Make sure the camera topic is correct."


class HMINode(Node):
    def __init__(self):
        super().__init__("rai_hmi_node")
        self.declare_parameter("robot_description_package", Parameter.Type.STRING)

        self.history: List[BaseMessage] = []

        self.callback_group = ReentrantCallbackGroup()
        self.hmi_subscription = self.create_subscription(
            String,
            "from_human",
            self.handle_human_message,
            10,
            callback_group=self.callback_group,
        )
        self.processing = False
        self.hmi_publisher = self.create_publisher(
            String, "to_human", 10, callback_group=self.callback_group
        )

        self.create_timer(
            0.01, self.status_callback, callback_group=self.callback_group
        )

        self.status_publisher = self.create_publisher(String, "hmi_status", 10)  # type: ignore
        self.task_addition_request_publisher = self.create_publisher(
            String, "task_addition_requests", 10
        )

        self.documentation_service = self.create_client(
            VectorStoreRetrieval,
            "rai_whoami_documentation_service",
            callback_group=self.callback_group,
        )
        self.constitution_service = self.create_client(
            Trigger,
            "rai_whoami_constitution_service",
        )
        self.identity_service = self.create_client(
            Trigger, "rai_whoami_identity_service"
        )

        self.robot_description_package = (
            self.get_parameter("robot_description_package")
            .get_parameter_value()
            .string_value
        )  # type: ignore

        self.get_logger().info("HMI Node has been started")
        system_prompt = self.initialize_system_prompt()

        self.llm = ChatOpenAI(model="gpt-4o", streaming=True)

        tools = [
            QueryDatabaseTool(get_response=self.get_database_response),
            QueueTaskTool(add_task=self.add_task_to_queue),
            Ros2GetTopicsNamesAndTypesTool(node=self),
            GetMsgFromTopic(node=self),
            Ros2PubMessageTool(node=self),
            get_current_image,
        ]
        self.name_to_tool_map = {tool.name: tool for tool in tools}

        prompt = ChatPromptTemplate.from_messages(
            [
                ("system", system_prompt),
                ("placeholder", "{chat_history}"),
                ("human", "{input}"),
                ("placeholder", "{agent_scratchpad}"),
            ]
        )
        self.agent = create_tool_calling_agent(llm=self.llm, tools=tools, prompt=prompt)
        self.agent_executor = AgentExecutor(agent=self.agent, tools=tools, verbose=True)
        self.faiss_index = self._load_documentation()

    def status_callback(self):
        if self.processing:
            self.status_publisher.publish(String(data="processing"))
        else:
            self.status_publisher.publish(String(data="waiting"))

    def get_database_response(self, query: str) -> VectorStoreRetrieval.Response:
        response = VectorStoreRetrieval.Response()
        output = self.faiss_index.similarity_search_with_score(query)
        response.message = "Query successful"
        response.success = True
        response.documents = [doc.page_content for doc, _ in output]
        response.scores = [float(score) for _, score in output]
        return response

        # Running the following code is problematic because
        # it works once, but then it blocks the handle_human_message
        while not self.documentation_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Identity service not available, waiting again...")

        request = VectorStoreRetrieval.Request()
        request.query = query

        request_future = self.documentation_service.call_async(request)
        rclpy.spin_until_future_complete(self, request_future)
        request_result = request_future.result()
        return request_result

    def initialize_system_prompt(self):
        while not self.constitution_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Constitution service not available, waiting again..."
            )

        while not self.identity_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Identity service not available, waiting again...")

        constitution_request = Trigger.Request()

        constitution_future = self.constitution_service.call_async(constitution_request)
        rclpy.spin_until_future_complete(self, constitution_future)
        constitution_response = constitution_future.result()

        identity_request = Trigger.Request()

        identity_future = self.identity_service.call_async(identity_request)
        rclpy.spin_until_future_complete(self, identity_future)
        identity_response = identity_future.result()

        system_prompt = f"""
        Constitution:
        {constitution_response.message}

        Identity:
        {identity_response.message}

        You are a helpful assistant. You converse with users.
        Assume the conversation is carried over a voice interface, so try not to overwhelm the user.
        If you have multiple questions, please ask them one by one allowing user to respond before
        moving forward to the next question. Keep the conversation short and to the point.
        Always reply in first person. When you use the tool and get the output, always present it in first person.
        """

        self.get_logger().info(f"System prompt initialized: {system_prompt}")
        return system_prompt

    def handle_human_message(self, human_ros_msg: String):
        asyncio.run(self.handle_human_message_async(human_ros_msg))

    def _load_documentation(self) -> FAISS:
        faiss_index = FAISS.load_local(
            get_package_share_directory(self.robot_description_package)
            + "/description",
            OpenAIEmbeddings(),
            allow_dangerous_deserialization=True,
        )
        return faiss_index

    def send_message_to_human(self, content: str):
        msg = String()
        msg.data = content
        self.hmi_publisher.publish(msg)

    async def send_message_to_human_async(self, content: str):
        msg = String()
        msg.data = content
        self.hmi_publisher.publish(msg)

    def add_task_to_queue(self, task: Task):
        msg = String()
        msg.data = task.json()
        self.task_addition_request_publisher.publish(msg)

    def clear_history(self):
        self.history = self.history[:1]

    class StreamingCallback(BaseCallbackHandler):
        def __init__(self, node: "HMINode"):
            self.node = node
            self.buffer = ""

        def on_llm_new_token(self, token: str, **kwargs) -> None:
            self.buffer += token
            if len(self.buffer) >= 30 and token.endswith((".", "!", "?")):
                self.node.send_message_to_human(self.buffer)
                self.buffer = ""

        def on_llm_end(self, response, **kwargs) -> None:
            if self.buffer:
                self.node.send_message_to_human(self.buffer)
                self.buffer = ""

    async def handle_human_message_async(self, human_ros_msg: String):
        self.processing = True
        self.get_logger().info("Handling human message")
        if not human_ros_msg.data:
            self.processing = False
            self.get_logger().warn("Received an empty message, discarding")
            return

        callback = self.StreamingCallback(self)
        response = await self.agent_executor.ainvoke(
            {"input": human_ros_msg.data, "chat_history": self.history},
            config={"callbacks": [callback]},
        )

        self.history.append(HumanMessage(human_ros_msg.data))
        self.history.append(SystemMessage(content=response["output"]))
        self.get_logger().info("Finished handling human message")
        self.processing = False


def main():
    rclpy.init()
    hmi_node = HMINode()

    async def run_node():
        executor = MultiThreadedExecutor()
        executor.add_node(hmi_node)

        try:
            while rclpy.ok():
                executor.spin_once()
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            hmi_node.get_logger().info("Keyboard interrupt, shutting down...")
        finally:
            hmi_node.destroy_node()
            rclpy.shutdown()

    asyncio.run(run_node())


if __name__ == "__main__":
    main()
