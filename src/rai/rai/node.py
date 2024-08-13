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
import functools
import json
import logging
import time
from abc import ABCMeta, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Literal, Optional, Tuple, TypedDict, cast

import cv2
import rcl_interfaces.msg
import rclpy
import rclpy.callback_groups
import rclpy.executors
import rclpy.qos
import rclpy.subscription
import rclpy.task
import sensor_msgs.msg
import std_msgs.msg
from cv_bridge import CvBridge
from langchain.tools import tool
from langchain_core.messages import (
    BaseMessage,
    HumanMessage,
    SystemMessage,
    ToolMessage,
)
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.pydantic_v1 import BaseModel, Field
from langchain_ollama import ChatOllama
from langchain_openai import ChatOpenAI
from langgraph.graph import END, START, StateGraph
from langgraph.graph.graph import CompiledGraph
from rclpy.action import get_action_names_and_types
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from rai.scenario_engine.messages import HumanMultimodalMessage, ToolMultimodalMessage
from rai.tools.ros.native import (
    Ros2BaseInput,
    Ros2BaseTool,
    Ros2GetTopicsNamesAndTypesTool,
)
from rai.tools.ros.utils import import_message_from_str


class RaiActionStoreInterface(metaclass=ABCMeta):
    @abstractmethod
    def register_action(
        self,
        uid: str,
        action_name: str,
        action_type: str,
        action_goal_args: Dict[str, Any],
        result_future: rclpy.task.Future,
    ):
        pass

    @abstractmethod
    def get_uids(self) -> List[str]:
        pass

    @abstractmethod
    def get_results(self, uid: Optional[str] = None) -> Dict[str, Any]:
        pass

    @abstractmethod
    def cancel_action(self, uid: str) -> bool:
        pass


@dataclass
class RaiActionStoreRecord:
    uid: str
    action_name: str
    action_type: str
    action_goal_args: Dict[str, Any]
    result_future: rclpy.task.Future


class RaiActionStore(RaiActionStoreInterface):
    def __init__(self) -> None:
        self._actions: Dict[str, RaiActionStoreRecord] = dict()
        self._results: Dict[str, Any] = dict()
        self._feedbacks: Dict[str, List[Any]] = dict()

    def register_action(
        self,
        uid: str,
        action_name: str,
        action_type: str,
        action_goal_args: Dict[str, Any],
        result_future: rclpy.task.Future,
    ):
        self._actions[uid] = RaiActionStoreRecord(
            uid=uid,
            action_name=action_name,
            action_type=action_type,
            action_goal_args=action_goal_args,
            result_future=result_future,
        )
        self._feedbacks[uid] = list()

    def add_feedback(self, uid: str, feedback: Any):
        if uid not in self._feedbacks:
            return  # TODO(boczekbartek): fix
        self._feedbacks[uid].append(feedback)

    def clear(self):
        self._actions.clear()
        self._results.clear()
        self._feedbacks.clear()

    def get_uids(self) -> List[str]:
        return list(self._actions.keys())

    def drop_action(self, uid: str) -> bool:
        if uid not in self._actions:
            raise KeyError(f"Unknown action: {uid=}")
        self._actions.pop(uid, None)
        self._feedbacks.pop(uid, None)
        logging.getLogger().info(f"Action(uid={uid}) dropped")
        return True

    def cancel_action(self, uid: str) -> bool:
        if uid not in self._actions:
            raise KeyError(f"Unknown action: {uid=}")
        self._actions[uid].result_future.cancel()
        self.drop_action(uid)
        logging.getLogger().info(f"Action(uid={uid}) canceled")
        return True

    def get_results(self, uid: Optional[str] = None) -> Dict[str, Any]:
        results = dict()
        done_actions = list()
        if uid is not None:
            uids = [uid]
        else:
            uids = self.get_uids()
        for uid in uids:
            if uid not in self._actions:
                raise KeyError(f"Unknown action: {uid=}")
            action = self._actions[uid]
            done = action.result_future.done()
            logging.getLogger().debug(f"Action(uid={uid}) done: {done}")
            if done:
                results[uid] = action.result_future.result()
                done_actions.append(uid)
            else:
                results[uid] = "Not done yet"

        # Remove done actions
        logging.getLogger().info(f"Removing done actions: {results.keys()=}")
        for uid in done_actions:
            self._actions.pop(uid, None)

        return results

    def get_feedbacks(self, uid: Optional[str] = None) -> List[Any]:

        if uid is None:
            return self._feedbacks
        if uid not in self._feedbacks:
            raise KeyError(f"Unknown action: {uid=}")
        return self._feedbacks[uid]


class RosoutBuffer:
    def __init__(self, bufsize: int = 100) -> None:
        self.bufsize = bufsize
        self._buffer: Deque[str] = deque()
        self.template = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    "Shorten the following log keeping its format - for example merge simillar or repeating lines",
                ),
                ("human", "{rosout}"),
            ]
        )
        llm = ChatOllama(model="llama3.1")
        self.llm = self.template | llm

    def clear(self):
        self._buffer.clear()

    def append(self, line: str):
        self._buffer.append(line)
        if len(self._buffer) > self.bufsize:
            self._buffer.popleft()

    def get_raw_logs(self, last_n: int = 30) -> str:
        return "\n".join(list(self._buffer)[-last_n:])

    def summarize(self):
        if len(self._buffer) == 0:
            return "No logs"
        buffer = self.get_raw_logs()
        response = self.llm.invoke({"rosout": buffer})
        return str(response.content)


class State(TypedDict):
    messages: List[BaseMessage]
    robot_state: BaseMessage
    current_task: str
    next_edge: Optional[str]


class TaskResolved(BaseModel):
    resolved: bool = Field(..., description="Whether the task was resolved or not")


@tool
def wait_for_2s():
    """Wait for 2 seconds"""
    time.sleep(2)


class TopicInput(Ros2BaseInput):
    topic_name: str = Field(..., description="Ros2 topic name")


class GetMsgFromTopic(Ros2BaseTool):
    name = "get_msg_from_topic"
    description: str = "Get message from topic"
    args_schema = TopicInput

    def _run(self, topic_name: str):
        msg = node.get_raw_message_from_topic(topic_name)
        if type(msg) is sensor_msgs.msg.Image:
            img = self.node.convert_ros_img_to_base64(msg)
            return {"content": "Got image", "images": [img]}
        else:
            return str(msg)


@dataclass
class NodeDiscovery:
    topics_and_types: Dict[str, str] = field(default_factory=dict)
    services_and_types: Dict[str, str] = field(default_factory=dict)
    actions_and_types: Dict[str, str] = field(default_factory=dict)


class RaiNode(Node):
    def __init__(self):
        super().__init__("rai_node")
        self._actions_cache = RaiActionStore()
        self.robot_state = dict()
        self.tools = [
            wait_for_2s,
            GetMsgFromTopic(node=self),
            Ros2GetTopicsNamesAndTypesTool(node=self),
        ]
        self.tools_by_name = {tool.name: tool for tool in self.tools}
        self.state_topics = [
            "/camera/camera/color/image_raw",
        ]
        self.state_postprocessors = {
            "/camera/camera/color/image_raw": self.describe_image
        }

        # ---------- ROS Parameters ----------
        self.task_topic = "/task_addition_requests"

        # ---------- ROS configuration ----------
        self.ros_discovery_info = NodeDiscovery()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.qos_profile = rclpy.qos.qos_profile_sensor_data
        self.DISCOVERY_FREQ = 0.5
        self.DISCOVERY_DEPTH = 5
        self.timer = self.create_timer(
            self.DISCOVERY_FREQ,
            self.discovery,
            callback_group=self.callback_group,
        )
        self.discovery()

        self.state_subscribers = dict()
        self.initialize_robot_state_interfaces(self.state_topics)
        self.initialize_task_subscriber()
        self.llm_workflow = self.initialize_llm_workflow()

        # ---------- LLM Agents ----------
        self.big_llm = ChatOpenAI(model="gpt-4o").bind_tools(self.tools)
        self.small_llm = ChatOpenAI(model="gpt-4o-mini")

    def discovery(self):
        def to_dict(info: List[Tuple[str, List[str]]]) -> Dict[str, str]:
            return {k: v[0] for k, v in info}

        self.ros_discovery_info.topics_and_types = to_dict(
            self.get_topic_names_and_types()
        )
        self.ros_discovery_info.actions_and_types = to_dict(
            get_action_names_and_types(self)
        )
        self.ros_discovery_info.services_and_types = to_dict(
            self.get_service_names_and_types()
        )

    def get_msg_type(self, topic: str, n_tries: int = 10) -> Any:
        """Sometimes node fails to do full discovery, therefore we need to retry"""
        for _ in range(n_tries):
            if topic in self.ros_discovery_info.topics_and_types:
                msg_type = self.ros_discovery_info.topics_and_types[topic]
                return import_message_from_str(msg_type)
            else:
                self.get_logger().info(f"Waiting for topic: {topic}")
                self.discovery()
                time.sleep(self.DISCOVERY_FREQ)
        raise KeyError(f"Topic {topic} not found")

    def get_srv_type(self, service: str) -> Any:
        pass

    def get_action_type(self, action: str) -> Any:
        pass

    def get_raw_message_from_topic(self, topic: str, timeout_sec: int = 1) -> Any:
        self.get_logger().info(f"Getting msg from topic: {topic}")
        if topic in self.state_subscribers and topic in self.robot_state:
            self.get_logger().info("Returning cached message")
            return self.robot_state[topic]
        else:
            msg_type = self.get_msg_type(topic)
            success, msg = wait_for_message(
                msg_type,
                self,
                topic,
                qos_profile=self.qos_profile,
                time_to_wait=timeout_sec,
            )

            if success:
                self.get_logger().info(
                    f"Received message of type {self.message_type.__class__.__name__} from topic {self.topic}"  # type: ignore
                )
                return msg
            else:
                self.get_logger().error(
                    f"Failed to receive message of type {self.message_type.__class__.__name__} from topic {self.topic}"  # type: ignore
                )
                return f"Failed to get msg in {timeout_sec} seconds"

    def initialize_robot_state_interfaces(self, topics):
        self.rosout_buffer = RosoutBuffer()
        self.rosout_sub = self.create_subscription(
            rcl_interfaces.msg.Log,
            "/rosout",
            callback=self.rosout_callback,
            callback_group=self.callback_group,
            qos_profile=self.qos_profile,
        )

        for topic in topics:
            msg_type = self.get_msg_type(topic)
            topic_callback = functools.partial(
                self.generic_state_subscriber_callback, topic
            )
            subscriber = self.create_subscription(
                msg_type,
                topic,
                callback=topic_callback,
                callback_group=self.callback_group,
                qos_profile=self.qos_profile,
            )

            self.state_subscribers[topic] = subscriber

    def initialize_task_subscriber(self):
        self.task_sub = self.create_subscription(
            std_msgs.msg.String,
            self.task_topic,
            callback=self.task_callback,
            callback_group=self.callback_group,
            qos_profile=self.qos_profile,
        )

    def initialize_llm_workflow(self) -> CompiledGraph:
        workflow = StateGraph(State)

        # ---------- Nodes ----------
        workflow.add_node("get_robot_state", self.get_robot_state)
        workflow.add_node("reason", self.reason)
        workflow.add_node("should_continue", self.should_continue)
        workflow.add_node("tools", self.run_tools)

        # ----------  Edges ----------
        workflow.add_edge(START, "get_robot_state")
        workflow.add_edge("get_robot_state", "reason")
        workflow.add_edge("reason", "should_continue")
        workflow.add_edge("tools", "reason")
        workflow.add_conditional_edges("should_continue", self.route)

        return workflow.compile()

    def route(self, state: State) -> Literal["tools", "get_robot_state", END]:
        if state["next_edge"] is None:
            return END
        elif state["next_edge"] == "get_robot_state":
            return "get_robot_state"
        elif state["next_edge"] == "tools":
            return "tools"
        else:
            self.get_logger().info(f"Unknown edge: {state['next_edge']}")
            return END

    # ---------- LLM Workflow Nodes ----------
    def summarize_messages(self, state: State) -> str:
        summary = ""
        for m in state["messages"]:
            summary += f"{m=}\n"
        return summary

    def reason(self, state: State) -> State:
        if len(state["messages"]) > 0:  # TODO(boczekbartek): add robot state
            self.get_logger().info(self.summarize_messages(state))
            input = (
                [SystemMessage(content=state["current_task"])]
                + state["messages"]
                # + state["messages"][:-2]
                # # + [state["robot_state"]]
                # + state["messages"][-1:]
            )
        else:
            input = [SystemMessage(content=state["current_task"]), state["robot_state"]]

        msg = self.big_llm.invoke(input)
        state["messages"].append(msg)
        return state

    def run_tools(self, state: State) -> State:
        result = []
        for tool_call in state["messages"][-1].tool_calls:
            tool = self.tools_by_name[tool_call["name"]]
            try:
                self.get_logger().info(
                    f"Trying tool {tool_call['name']} call with args: {tool_call['args']}"
                )
                observation = tool.invoke(tool_call["args"])
                self.get_logger().info(f"Succeeded, observation: {observation}")
            except Exception as e:
                self.get_logger().error(f"Error running tool: {e}")
                observation = f"Error running tool: {e}"
            if isinstance(observation, dict):
                self.get_logger().info(
                    f"Tool observation: {observation.get('content')}"
                )
                tool_message = ToolMultimodalMessage(
                    content=observation.get("content", "No response from the tool."),
                    images=observation.get("images"),
                    tool_call_id=tool_call["id"],
                )
                tool_messages = tool_message.postprocess(format="openai")
                result.extend(tool_messages)
            else:
                self.get_logger().info(f"Tool observation: {observation}")
                result.append(
                    ToolMessage(content=str(observation), tool_call_id=tool_call["id"])
                )
        state["messages"].extend(result)

    def get_robot_state(self, state: State) -> State:
        state_dict = dict()
        for t in self.state_subscribers:
            if t not in self.robot_state:
                msg = "No message yet"
                state_dict[t] = msg
                continue
            msg = self.robot_state[t]
            if t in self.state_postprocessors:
                msg = self.state_postprocessors[t](msg)
            state_dict[t] = msg

        state_dict["logs_summary"] = self.rosout_buffer.summarize()
        state["robot_state"] = HumanMessage(json.dumps(state_dict))
        return state

    def should_continue(self, state: State) -> State:
        chat_history = state["messages"]

        # Check if tool call requested
        last_message = chat_history[-1]
        if last_message.tool_calls:
            self.get_logger().info(f"Tool call requested: {last_message.tool_calls}")
            state["next_edge"] = "tools"
            return state

        system_prompt = SystemMessage(
            content="You are a critic. Your job is to determine if the task is completed."
        )

        prompt = ChatPromptTemplate.from_messages(
            [
                ("system", "{system_prompt}"),
                ("human", "The task to judge. {task}"),
                ("human", "The chat history that you should judge upon"),
                ("placeholder", "{chat_history}"),
            ]
        )
        task = state["current_task"]
        chain = prompt | self.small_llm.with_structured_output(TaskResolved)

        task_resolved: TaskResolved = chain.invoke(
            {"system_prompt": system_prompt, "task": task, "chat_history": chat_history}
        )  # type: ignore

        state["next_edge"] = None if task_resolved.resolved else "get_robot_state"
        return state

    # ---------- ROS Callbacks ----------
    def task_callback(self, msg: std_msgs.msg.String):
        self.get_logger().info(f"Received task: {msg.data}")
        payload = {
            "current_task": msg.data,
            "messages": [],
            "robot_state": None,
        }

        self.llm_workflow.invoke(
            payload
        )  # TODO(boczekbartek): increase recursion limit

        self.clear_state()
        self.get_logger().info("Finished task")

    def generic_state_subscriber_callback(self, topic_name, msg: Any):
        self.robot_state[topic_name] = msg

    def rosout_callback(self, msg: rcl_interfaces.msg.Log):
        if "rai_node" in msg.name:
            return
        self.rosout_buffer.append(f"[{msg.stamp.sec}][{msg.name}]:{msg.msg}")

    # ---------- LLM Tools ----------

    # ---------- LLM Utilities ----------
    def describe_image(
        self, msg: sensor_msgs.msg.Image
    ) -> Dict[Literal["camera_image_summary"], str]:
        PROMPT = """Please describe the image in 2 sentences max 150 chars."""
        base64_image = self.convert_ros_img_to_base64(msg)
        llm_msg = HumanMultimodalMessage(content=PROMPT, images=[base64_image])
        output = self.small_llm.invoke([llm_msg])
        return {"camera_image_summary": str(output.content)}

    # ---------- ROS Utilities ----------

    # ---------- Other Utilities ----------
    def convert_ros_img_to_base64(self, msg: sensor_msgs.msg.Image) -> str:
        bridge = CvBridge()
        cv_image = cast(cv2.Mat, bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))  # type: ignore
        if cv_image.shape[-1] == 4:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGB)
            return base64.b64encode(bytes(cv2.imencode(".png", cv_image)[1])).decode(
                "utf-8"
            )
        else:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            image_data = cv2.imencode(".png", cv_image)[1].tostring()  # type: ignore
            return base64.b64encode(image_data).decode("utf-8")  # type: ignore

    def clear_state(self):
        self._actions_cache.clear()
        self.rosout_buffer.clear()

    #
    # def get_actions_cache(self) -> RaiActionStoreInterface:
    #     return self._actions_cache
    #
    # def get_results(self, uid: Optional[str]):
    #     self.get_logger().info("Getting results")
    #     return self._actions_cache.get_results(uid)
    #
    # def get_feedbacks(self, uid: Optional[str] = None):
    #     self.get_logger().info("Getting feedbacks")
    #     return self._actions_cache.get_feedbacks(uid)
    #
    # def cancel_action(self, uid: str):
    #     self.get_logger().info(f"Canceling action: {uid=}")
    #     return self._actions_cache.cancel_action(uid)
    #
    # def get_running_actions(self):
    #     return self._actions_cache.get_uids()
    #
    # def feedback_callback(self, uid: str, feedback_msg: Any):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().debug(f"Received feedback: {feedback}")
    #     self._actions_cache.add_feedback(uid, feedback)
    #
    # def get_state(self):
    #     self.get_logger().info("Getting state")
    #     future = self.state_client.call_async(std_srvs.srv.Trigger.Request())
    #     rclpy.spin_until_future_complete(self, future)
    #     result: std_srvs.srv.Trigger.Response = future.result()
    #     return result.message
    #


if __name__ == "__main__":
    rclpy.init()

    node = RaiNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
