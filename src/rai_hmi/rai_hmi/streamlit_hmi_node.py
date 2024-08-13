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
import sys
from threading import Thread

import rclpy
import streamlit as st
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from langchain.memory import ConversationBufferMemory
from langchain.tools import tool
from langchain_community.vectorstores import FAISS
from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
from langchain_openai import OpenAIEmbeddings
from langchain_openai.chat_models import ChatOpenAI
from std_msgs.msg import String

from rai.node import RaiBaseNode
from rai.scenario_engine.messages import HumanMultimodalMessage, ToolMultimodalMessage
from rai.tools.ros.native import GetCameraImage, Ros2GetTopicsNamesAndTypesTool
from rai_hmi.agent import State as ConversationState
from rai_hmi.agent import create_conversational_agent
from rai_hmi.custom_mavigator import RaiNavigator
from rai_hmi.task import Task
from rai_interfaces.srv import VectorStoreRetrieval

package_name = sys.argv[1] if len(sys.argv) > 1 else None

st.set_page_config(page_title="LangChain Chat App", page_icon="🦜")
if package_name:
    st.title(f"{package_name.replace('_whoami', '')} chat app")
else:
    st.title("ROS 2 Chat App")

if "memory" not in st.session_state:
    st.session_state.memory = ConversationBufferMemory(
        memory_key="chat_history", return_messages=True
    )


class HMINode(RaiBaseNode):
    def __init__(self, robot_description_package: str):
        super().__init__("rai_hmi_node")
        self.robot_description_package = robot_description_package
        self.status_publisher = self.create_publisher(String, "hmi_status", 10)  # type: ignore self.task_addition_request_publisher = self.create_publisher(
            String, "task_addition_requests", 10
        )

        self.documentation_service = self.create_client(
            VectorStoreRetrieval,
            "rai_whoami_documentation_service",
            callback_group=self.callback_group,
        )
        self.faiss_index = self.load_documentation()

    def load_documentation(self) -> FAISS:
        faiss_index = FAISS.load_local(
            get_package_share_directory(self.robot_description_package)
            + "/description",
            OpenAIEmbeddings(),
            allow_dangerous_deserialization=True,
        )
        return faiss_index


@st.cache_resource
def initialize_ros(robot_description_package: str):

    rclpy.init()

    if package_name is not None:
        hmi_node = HMINode(robot_description_package=robot_description_package)
        thread = Thread(target=rclpy.spin, args=(hmi_node,), daemon=True)
        thread.start()
        system_prompt = hmi_node.initialize_system_prompt()
        faiss_index = hmi_node.load_documentation()
        return hmi_node, system_prompt, faiss_index
    else:
        return rclpy.node.Node("rai_chat_node"), "", None


llm = ChatOpenAI(
    temperature=0.5,
    model="gpt-4o",
    streaming=True,
)


@tool
def add_task_to_queue(task: Task):
    """Use this tool to add a task to the queue. The task will be handled by the executor part of your system."""
    hmi_node.task_addition_request_publisher.publish(String(data=task.json()))
    return f"Task added to the queue: {task.json()}"


@tool
def spin_robot(degrees_rad: float) -> str:
    """Use this tool to spin the robot."""
    navigator = RaiNavigator()
    navigator.spin(spin_dist=degrees_rad)
    return "Robot spinning."


@tool
def drive_forward(distance_m: float) -> str:
    """Use this tool to drive the robot forward."""
    navigator = RaiNavigator()
    p = Point()
    p.x = distance_m

    navigator.drive_on_heading(p, 0.5, 10)
    return "Robot driving forward."


@tool
def search_database(query: str) -> str:
    """Use this tool to search the documentation database."""
    results = faiss_index.similarity_search_with_score(query)
    formatted_results = []
    for idx, (document, score) in enumerate(results):
        formatted_results.append(
            f"Result {idx + 1}:\nDocument: {document}\nScore: {score:.4f}\n"
        )
    return "\n".join(formatted_results)


@st.cache_resource
def initialize_genAI(_node: HMINode):
    tools = [
        Ros2GetTopicsNamesAndTypesTool(node=_node),
        GetCameraImage(node=_node),
    ]
    if package_name:
        tools.append(add_task_to_queue)
        tools.append(search_database)
        tools.append(spin_robot)
        tools.append(drive_forward)

    agent = create_conversational_agent(
        llm, tools, debug=True, system_prompt=_node.system_prompt
    )

    state = ConversationState(messages=[])
    return agent, state


hmi_node = initialize_ros(sys.argv[1])
faiss_index = hmi_node.faiss_index
agent_executor, state = initialize_genAI(_node=hmi_node)

st.subheader("Chat")

if "messages" not in st.session_state:
    st.session_state["messages"] = []

for message in st.session_state["messages"]:
    message_type = message.type
    if isinstance(message, (HumanMultimodalMessage, ToolMessage)):
        message_type = "ai"
    with st.chat_message(message_type):
        if isinstance(message, HumanMultimodalMessage):
            base64_images = [image for image in message.images]
            images = [base64.b64decode(image) for image in base64_images]
            for image in images:
                st.image(image)
            if isinstance(message.content, list):
                content = message.content[0]["text"]
                st.markdown(content)
        elif isinstance(message, (ToolMessage, ToolMultimodalMessage)):
            st.expander(f"Tool: {message.name}").markdown(message.content)
        elif isinstance(message, AIMessage):
            if message.content == "":  # tool calling
                for tool_call in message.tool_calls:
                    st.markdown(f"Tool: {tool_call['name']}")
            else:
                st.markdown(message.content)
        else:
            st.markdown(message.content)

if prompt := st.chat_input("What is your question?"):
    st.chat_message("user").markdown(prompt)
    st.session_state["messages"].append(HumanMessage(content=prompt))
    state["messages"].append(HumanMessage(content=prompt))

    with st.chat_message("assistant"):
        message_placeholder = st.container()
        n_messages = len(state["messages"])
        with message_placeholder.status("Thinking..."):
            response = agent_executor.invoke(state)
        new_messages = state["messages"][n_messages:]
        for message in new_messages:
            if isinstance(message, HumanMultimodalMessage):
                base64_images = [image for image in message.images]
                # convert the str to bytes
                images = [base64.b64decode(image) for image in base64_images]
                for image in images:
                    message_placeholder.image(image)

        output = response["messages"][-1]

        message_placeholder.markdown(output.content)

    st.session_state["messages"].extend(new_messages)
