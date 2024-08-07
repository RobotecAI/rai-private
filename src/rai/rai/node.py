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
from abc import ABCMeta, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional

import rcl_interfaces.msg
import rclpy
import rclpy.callback_groups
import rclpy.qos
import std_msgs.msg
import std_srvs.srv
from langchain_core.prompts import ChatPromptTemplate
from langchain_ollama import ChatOllama
from rclpy.node import Node

from rai.run_task import run_task


class RaiActionStoreInterface(metaclass=ABCMeta):
    @abstractmethod
    def register_action(
        self,
        uid: str,
        action_name: str,
        action_type: str,
        action_goal_args: Dict[str, Any],
        result_future: rclpy.Future,
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
    result_future: rclpy.Future


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
        result_future: rclpy.Future,
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
            raise KeyError(f"Unknown action: {uid=}")
        self._feedbacks[uid].append(feedback)

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

    def get_feedbacks(self, uid: str) -> List[Any]:
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

    def append(self, line: str):
        self._buffer.append(line)
        if len(self._buffer) > self.bufsize:
            self._buffer.popleft()

    def get_raw_logs(self) -> str:
        return "\n".join(self._buffer)

    def summarize(self):
        if len(self._buffer) == 0:
            return "No logs"
        buffer = self.get_raw_logs()
        response = self.llm.invoke({"rosout": buffer})
        return str(response.content)


class RaiNode(Node):
    def __init__(self):
        super().__init__("rai_node")

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self._actions_cache = RaiActionStore()

        self.rosout_buffer = RosoutBuffer()
        self.state_client = self.create_client(
            std_srvs.srv.Trigger, "/rai/state", callback_group=self.callback_group
        )

        self.rosout_sub = self.create_subscription(
            rcl_interfaces.msg.Log,
            "/rosout",
            callback=self.rosout_callback,
            callback_group=self.callback_group,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )
        self.rosout_summary_service = self.create_service(
            std_srvs.srv.Trigger,
            "rai_rosout_summary_service",
            self.log_summary_callback,
            callback_group=self.callback_group,
        )

        self.task_sub = self.create_subscription(
            std_msgs.msg.String,
            "/task_addition_requests",
            callback=self.task_callback,
            callback_group=self.callback_group,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

    def task_callback(self, msg: std_msgs.msg.String):
        # task_dict = json.loads(msg.data)
        # task = Task(**task_dict)
        self.get_logger().info(f"Received task: {msg.data}")
        run_task(self, msg.data)
        self.get_logger().info("Finished task")
        rclpy.spin(self)

    def rosout_callback(self, msg: rcl_interfaces.msg.Log):
        self.rosout_buffer.append(f"[{msg.stamp.sec}][{msg.name}]:{msg.msg}")

    def log_summary_callback(self, request, response):
        response.success = True
        self.get_logger().info(f"Raw log\n{self.rosout_buffer.get_raw_logs()}")
        response.message = self.rosout_buffer.summarize()
        self.get_logger().info(f"Summary:\n{response.message}")
        return response

    def get_actions_cache(self) -> RaiActionStoreInterface:
        return self._actions_cache

    def get_results(self, uid: Optional[str]):
        self.get_logger().info("Getting results")
        return self._actions_cache.get_results(uid)

    def get_feedbacks(self, uid: str):
        self.get_logger().info("Getting feedbacks")
        return self._actions_cache.get_feedbacks(uid)

    def cancel_action(self, uid: str):
        self.get_logger().info(f"Canceling action: {uid=}")
        return self._actions_cache.cancel_action(uid)

    def get_running_actions(self, uid: str):
        self.get_logger().info(f"Getting running actions: {uid=}")
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
    rclpy.spin(node)
    rclpy.shutdown()
