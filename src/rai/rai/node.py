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

import functools
import time
from typing import Any, Callable, Dict, List, Literal, Optional

import rcl_interfaces.msg
import rclpy
import rclpy.callback_groups
import rclpy.executors
import rclpy.qos
import rclpy.subscription
import rclpy.task
import sensor_msgs.msg
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI
from langgraph.graph.graph import CompiledGraph
from rclpy.action.graph import get_action_names_and_types
from rclpy.action.server import ActionServer, GoalResponse, ServerGoalHandle
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger

from rai.agents.state_based import Report, State
from rai.messages.multimodal import HumanMultimodalMessage, MultimodalMessage
from rai.tools.ros.utils import convert_ros_img_to_base64, import_message_from_str
from rai.tools.utils import wait_for_message
from rai.utils.ros import NodeDiscovery, RosoutBuffer
from rai_interfaces.action import Task as TaskAction


class RaiBaseNode(Node):
    def __init__(
        self,
        whitelist: Optional[List[str]] = None,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)

        self.robot_state = dict()

        self.DISCOVERY_FREQ = 2.0
        self.DISCOVERY_DEPTH = 5
        self.timer = self.create_timer(
            self.DISCOVERY_FREQ,
            self.discovery,
        )
        self.ros_discovery_info = NodeDiscovery(whitelist=whitelist)
        self.discovery()
        self.qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
        )

        self.state_subscribers = dict()

    def discovery(self):
        self.ros_discovery_info.set(
            self.get_topic_names_and_types(),
            self.get_service_names_and_types(),
            get_action_names_and_types(self),
        )

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
                    f"Received message of type {msg_type.__class__.__name__} from topic {topic}"
                )
                return msg
            else:
                error = (
                    f"No message received in {timeout_sec} seconds from topic {topic}"
                )
                self.get_logger().error(error)
                return error

    def get_msg_type(self, topic: str, n_tries: int = 5) -> Any:
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


class RaiGenericBaseNode(RaiBaseNode):
    def __init__(
        self,
        node_name: str,
        system_prompt: str,
        llm,
        observe_topics: Optional[List[str]] = None,
        observe_postprocessors: Optional[Dict[str, Callable]] = None,
        whitelist: Optional[List[str]] = None,
        *args,
        **kwargs,
    ):
        super().__init__(node_name=node_name, whitelist=whitelist, *args, **kwargs)
        self.llm = llm

        self.robot_state = dict()
        self.state_topics = observe_topics if observe_topics is not None else []
        self.state_postprocessors = (
            observe_postprocessors if observe_postprocessors is not None else dict()
        )

        self.constitution_service = self.create_client(
            Trigger,
            "rai_whoami_constitution_service",
        )
        self.identity_service = self.create_client(
            Trigger, "rai_whoami_identity_service"
        )

        self.callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.initialize_robot_state_interfaces(self.state_topics)

        self.system_prompt = self.initialize_system_prompt(system_prompt)

    def initialize_system_prompt(self, prompt: str):
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

        {prompt}
        """

        self.get_logger().info(f"System prompt initialized: {system_prompt}")
        return system_prompt

    def generic_state_subscriber_callback(self, topic_name: str, msg: Any):
        self.get_logger().debug(
            f"Received message of type {type(msg)} from topic {topic_name}"
        )
        self.robot_state[topic_name] = msg

    def initialize_robot_state_interfaces(self, topics):
        self.rosout_buffer = RosoutBuffer(self.llm)

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


def parse_task_goal(ros_action_goal: TaskAction.Goal) -> Dict[str, Any]:
    return dict(
        task=ros_action_goal.task,
        description=ros_action_goal.description,
        priority=ros_action_goal.priority,
    )


class RaiNode(RaiGenericBaseNode):
    def __init__(
        self,
        system_prompt: str,
        llm,
        observe_topics: Optional[List[str]] = None,
        observe_postprocessors: Optional[Dict[str, Callable]] = None,
        whitelist: Optional[List[str]] = None,
        *args,
        **kwargs,
    ):
        super().__init__(
            "rai_node",
            llm=llm,
            system_prompt=system_prompt,
            observe_topics=observe_topics,
            observe_postprocessors=observe_postprocessors,
            whitelist=whitelist,
            *args,
            **kwargs,
        )

        # ---------- ROS configuration ----------
        self.rosout_sub = self.create_subscription(
            rcl_interfaces.msg.Log,
            "/rosout",
            callback=self.rosout_callback,
            callback_group=self.callback_group,
            qos_profile=self.qos_profile,
        )

        # ---------- Task Queue ----------
        self.task_action_server = ActionServer(
            self,
            TaskAction,
            "perform_task",
            execute_callback=self.agent_loop,
            goal_callback=self.goal_callback,
        )
        # Node is busy when task is executed. Only 1 task is allowed
        self.busy = False

        # ---------- LLM Agents ----------
        self.AGENT_RECURSION_LIMIT = 100
        self.llm_app: CompiledGraph = None

        # self.task_queue = Queue()
        # self.agent_loop_thread = Thread(target=self.agent_loop)
        # self.agent_loop_thread.start()

    def goal_callback(self, _) -> GoalResponse:
        """Accept or reject a client request to begin an action."""
        response = GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT
        self.get_logger().info(f"Received goal request. Response: {response}")
        return response

    async def agent_loop(self, goal_handle: ServerGoalHandle):
        self.busy = True
        try:
            action_request: TaskAction.Goal = goal_handle.request
            task: Dict[str, Any] = parse_task_goal(
                action_request
            )  # TODO(boczekbartek): base model and json

            self.get_logger().info(f"Received task: {task}")

            # ---- LLM Task Handling ----
            messages = [
                SystemMessage(content=self.system_prompt),
                HumanMessage(content=f"Task: {task}"),
            ]

            payload = State(messages=messages)

            state = None
            for state in self.llm_app.stream(
                payload, {"recursion_limit": self.AGENT_RECURSION_LIMIT}
            ):

                print(state.keys())
                graph_node_name = list(state.keys())[0]
                if graph_node_name == "reporter":
                    continue

                msg = state[graph_node_name]["messages"][-1]

                if isinstance(msg, MultimodalMessage):
                    last_msg = msg.text
                else:
                    last_msg = msg.content

                feedback_msg = TaskAction.Feedback()
                feedback_msg.current_status = f"{graph_node_name}: {last_msg}"

                goal_handle.publish_feedback(feedback_msg)

            # ---- Share Action Result ----
            if state is None:
                raise ValueError("No output from LLM")
            print(state)

            graph_node_name = list(state.keys())[0]
            if graph_node_name != "reporter":
                raise ValueError(f"Unexpected output llm node: {graph_node_name}")

            report = state["reporter"]["messages"][
                -1
            ]  # TODO define graph more strictly not as dict key

            if not isinstance(report, Report):
                raise ValueError(f"Unexpected type of agent output: {type(report)}")

            if report.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            result = TaskAction.Result()
            result.success = report.success
            result.report = report.response_to_user

            self.get_logger().info(f"Finished task:\n{result}")
            self.clear_state()

            return result
        finally:
            self.busy = False

    def set_app(self, app: CompiledGraph):
        self.llm_app = app

    def get_robot_state(self) -> Dict[str, str]:
        state_dict = dict()

        if self.robot_state is None:
            return state_dict

        for t in self.state_subscribers:
            if t not in self.robot_state:
                msg = "No message yet"
                state_dict[t] = msg
                continue
            msg = self.robot_state[t]
            if t in self.state_postprocessors:
                msg = self.state_postprocessors[t](msg)
            state_dict[t] = msg
        state_dict.update(
            {
                "robot_interfaces": self.ros_discovery_info.dict(),
            }
        )

        state_dict["logs_summary"] = self.rosout_buffer.summarize()
        self.get_logger().info(f"{state_dict=}")
        return state_dict

    def clear_state(self):
        self.rosout_buffer.clear()

    def rosout_callback(self, msg: rcl_interfaces.msg.Log):
        self.get_logger().debug(f"Received rosout: {msg}")
        if "rai_node" in msg.name:
            return
        self.rosout_buffer.append(f"[{msg.stamp.sec}][{msg.name}]:{msg.msg}")


def describe_ros_image(
    msg: sensor_msgs.msg.Image,
) -> Dict[Literal["camera_image_summary"], str]:
    PROMPT = """Please describe the image in 2 sentences max 150 chars."""
    small_llm = ChatOpenAI(model="gpt-4o-mini")
    base64_image = convert_ros_img_to_base64(msg)
    llm_msg = HumanMultimodalMessage(content=PROMPT, images=[base64_image])
    output = small_llm.invoke([llm_msg])
    return {"camera_image_summary": str(output.content)}
