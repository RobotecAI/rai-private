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
import json
import logging
from abc import ABCMeta, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, TypedDict, cast

import cv2
import rcl_interfaces.msg
import rclpy
import rclpy.callback_groups
import rclpy.executors
import rclpy.qos
import rclpy.task
import sensor_msgs.msg
import std_msgs.msg
import std_srvs.srv
from cv_bridge import CvBridge
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_ollama import ChatOllama
from langchain_openai import ChatOpenAI
from langgraph.graph import StateGraph
from rclpy.node import Node

from rai.run_task import run_task
from rai.scenario_engine.messages import HumanMultimodalMessage


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
    cont: bool


class RaiNode(Node):
    def __init__(self):
        super().__init__("rai_node")
        self._actions_cache = RaiActionStore()
        # ---------- ROS Parameters ----------
        self.camera_topic = (
            "/camera/camera/color/image_raw"  # TODO(boczekbartek): parametrize
        )
        self.task_topic = "/task_addition_requests"

        # ---------- ROS configuration ----------
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.qos_profile = rclpy.qos.qos_profile_sensor_data

        self.initialize_robot_state_interfaces()
        self.initialize_task_subscriber()
        self.llm_workflow = self.initialize_llm_workflow()

        # ---------- LLM Agents ----------
        self.big_llm = ChatOpenAI(model="gpt-4o")
        self.small_llm = ChatOpenAI(model="gpt-4o-mini")

    def initialize_robot_state_interfaces(self):
        self.rosout_buffer = RosoutBuffer()
        self.rosout_sub = self.create_subscription(
            rcl_interfaces.msg.Log,
            "/rosout",
            callback=self.rosout_callback,
            callback_group=self.callback_group,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

        self.camera_sub = self.create_subscription(
            sensor_msgs.msg.Image,
            "/camera/image_raw",
            callback=self.camera_image_callback,
            callback_group=self.callback_group,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

    def initialize_task_subscriber(self):
        self.task_sub = self.create_subscription(
            std_msgs.msg.String,
            self.task_topic,
            callback=self.task_callback,
            callback_group=self.callback_group,
            qos_profile=self.qos_profile,
        )

    def initialize_llm_workflow(self):
        workflow = StateGraph(State)
        workflow.add_node("get_robot_state", self.get_robot_state)
        workflow.add_node("reason", self.reason)
        return workflow

    # ---------- LLM Workflow Nodes ----------
    def reason(self, state: State) -> State:
        input = (
            [SystemMessage(content=state["current_task"])]
            + state["messages"]
            + [state["robot_state"]]
        )
        llm = self.big_llm
        msg = llm.invoke(input)
        state["messages"].append(msg)
        return state

    def get_robot_state(self, state: State) -> State:
        camera_img_summary = self.describe_image(self.robot_state.last_image)
        logs_summary = self.rosout_buffer.summarize()

        state["robot_state"] = HumanMessage(
            json.dumps(
                {"camera_img_summary": camera_img_summary, "logs_summary": logs_summary}
            )
        )
        return state

    # ---------- ROS Callbacks ----------
    def camera_image_callback(self, msg: sensor_msgs.msg.Image):
        pass

    # ---------- LLM Utilities ----------
    def describe_image(self, msg: sensor_msgs.msg.Image) -> str:
        PROMPT = """Please describe the image in 2 sentences max 150 chars."""
        base64_image = self.convert_ros_img_to_base64(msg)
        llm_msg = HumanMultimodalMessage(content=PROMPT, images=[base64_image])
        output = self.small_llm.invoke(llm_msg)
        return output.content

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
            image_data = cv2.imencode(".png", cv_image)[1].tostring()  # type: ignore
            return base64.b64encode(image_data).decode("utf-8")  # type: ignore

    def clear_state(self):
        self._actions_cache.clear()
        self.rosout_buffer.clear()

    def task_callback(self, msg: std_msgs.msg.String):
        self.get_logger().info(f"Received task: {msg.data}")

        self.workflow

        run_task(self, msg.data, self.history)
        self._actions_cache.clear()
        self.rosout_buffer.clear()
        self.get_logger().info("Finished task")
        rclpy.spin(self)

    def rosout_callback(self, msg: rcl_interfaces.msg.Log):
        if "rai_node" in msg.name:
            return
        self.rosout_buffer.append(f"[{msg.stamp.sec}][{msg.name}]:{msg.msg}")

    def summarize_logs(self):
        return self.rosout_buffer.summarize()

    def get_actions_cache(self) -> RaiActionStoreInterface:
        return self._actions_cache

    def get_results(self, uid: Optional[str]):
        self.get_logger().info("Getting results")
        return self._actions_cache.get_results(uid)

    def get_feedbacks(self, uid: Optional[str] = None):
        self.get_logger().info("Getting feedbacks")
        return self._actions_cache.get_feedbacks(uid)

    def cancel_action(self, uid: str):
        self.get_logger().info(f"Canceling action: {uid=}")
        return self._actions_cache.cancel_action(uid)

    def get_running_actions(self):
        return self._actions_cache.get_uids()

    def feedback_callback(self, uid: str, feedback_msg: Any):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f"Received feedback: {feedback}")
        self._actions_cache.add_feedback(uid, feedback)

    def get_state(self):
        self.get_logger().info("Getting state")
        future = self.state_client.call_async(std_srvs.srv.Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        result: std_srvs.srv.Trigger.Response = future.result()
        return result.message


if __name__ == "__main__":
    rclpy.init()

    node = RaiNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
