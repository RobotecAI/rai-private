import threading
from typing import List, Tuple, Union

import pytest
import rclpy
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolCall
from langchain_core.tools import render_text_description
from langchain_ollama import ChatOllama
from langchain_openai import ChatOpenAI
from rclpy.node import Node
from std_msgs.msg import String

from rai.tools.ros.native import Ros2GetTopicsNamesAndTypesTool


class Publisher(Node):
    def __init__(self):
        super().__init__("test_publisher")
        self.topic = "/rai_test_topic"
        self.publisher_ = self.create_publisher(String, self.topic, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World"
        self.publisher_.publish(msg)


@pytest.fixture(scope="session")
def ros2_node():
    rclpy.init()
    node = Publisher()
    t = threading.Thread(target=rclpy.spin, args=(node,))
    t.start()
    yield node
    rclpy.shutdown()
    t.join()


OLLAMA_URL = "http://localhost:11434"


@pytest.mark.parametrize(
    "provider,model_name",
    [
        ("ollama", "llama3.1:8b"),
        ("ollama", "llama3.2:3b"),
        ("ollama", "firefunction-v2"),
        ("ollama", "command-r-plus"),
        ("openai", "gpt-3.5-turbo"),
        ("openai", "gpt-4o"),
        ("openai", "gpt-4o-mini"),
    ],
)
@pytest.mark.parametrize("render_tools_in_system_prompt", [True, False])
@pytest.mark.parametrize(
    "system_prompt",
    [
        "",
        "You will be given a task and you will have tools. Please call tools only if you think the task requires additional knowledge that tools can provide.",
    ],
)
@pytest.mark.parametrize(
    "prompt,should_call",
    [("What are available ros2 topics?", True), ("Who are you?", False)],
)
def test_ros2_topic_listing_tool(
    ros2_node: Publisher,
    provider: str,
    model_name: str,
    render_tools_in_system_prompt: bool,
    system_prompt: str,
    prompt: str,
    should_call: bool,
):
    llm: Union[ChatOllama, ChatOpenAI] = {
        "openai": ChatOpenAI(model=model_name),
        "ollama": ChatOllama(model=model_name, base_url=OLLAMA_URL),
    }[provider]

    ros2_tool = Ros2GetTopicsNamesAndTypesTool(node=ros2_node)
    tools = [ros2_tool]
    llm = llm.bind_tools(tools)
    if render_tools_in_system_prompt:
        system_prompt += f"\nHere are the available tools: {render_text_description(tools)}"  # type: ignore

    messages = [SystemMessage(content=system_prompt), HumanMessage(content=prompt)]

    response = llm.invoke(messages)
    assert type(response) is AIMessage

    if not should_call:
        assert response.tool_calls == []
        return

    assert len(response.tool_calls) > 0

    tool_call = ToolCall(**response.tool_calls[0])
    assert tool_call["name"] == ros2_tool.name

    topics_and_types: List[Tuple[str, List[str]]] = eval(
        ros2_tool.invoke(tool_call).content
    )
    match = [(t, ty) for t, ty in topics_and_types if t == ros2_node.topic]
    assert len(match) == 1
    topic, msg_type = match[0]
    assert topic == ros2_node.topic
    assert msg_type[0] == "std_msgs/msg/String"
