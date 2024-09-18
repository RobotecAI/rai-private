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

import json
from typing import Any, Dict, OrderedDict, Tuple, Type

import rclpy
import rclpy.callback_groups
import rclpy.executors
import rclpy.node
import rclpy.qos
import rclpy.subscription
import rclpy.task
import rosidl_runtime_py.set_message
import rosidl_runtime_py.utilities
import sensor_msgs.msg
from langchain.tools import BaseTool
from pydantic import BaseModel, Field
from rclpy.impl.rcutils_logger import RcutilsLogger

from .utils import convert_ros_img_to_base64, import_message_from_str


# --------------------- Inputs ---------------------
class Ros2BaseInput(BaseModel):
    """Empty input for ros2 tool"""


class Ros2MsgInterfaceInput(BaseModel):
    """Input for the show_ros2_msg_interface tool."""

    msg_name: str = Field(..., description="Ros2 message name in typical ros2 format.")


class Ros2GetOneMsgFromTopicInput(BaseModel):
    """Input for the get_current_position tool."""

    topic: str = Field(..., description="Ros2 topic")
    msg_type: str = Field(
        ..., description="Type of ros2 message in typical ros2 format."
    )
    timeout_sec: int = Field(
        10, description="The time in seconds to wait for a message to be received."
    )


class PubRos2MessageToolInput(BaseModel):
    topic_name: str = Field(..., description="Ros2 topic to publish the message")
    msg_type: str = Field(
        ..., description="Type of ros2 message in typical ros2 format."
    )
    msg_args: Dict[str, Any] = Field(
        ..., description="The arguments of the service call."
    )


# --------------------- Tools ---------------------
class Ros2BaseTool(BaseTool):
    # TODO: Make the decision between rclpy.node.Node and RaiNode
    node: rclpy.node.Node = Field(..., exclude=True, required=True)

    args_schema: Type[Ros2BaseInput] = Ros2BaseInput

    @property
    def logger(self) -> RcutilsLogger:
        return self.node.get_logger()


class Ros2GetTopicsNamesAndTypesTool(Ros2BaseTool):
    name: str = "Ros2GetTopicsNamesAndTypes"
    description: str = "A tool for getting all ros2 topics names and types"

    def _run(self):
        return self.node.get_topic_names_and_types()


class Ros2GetRobotInterfaces(Ros2BaseTool):
    name: str = "ros2_robot_interfaces"
    description: str = (
        "A tool for getting all ros2 robot interfaces: topics, services and actions"
    )

    def _run(self):
        return self.node.ros_discovery_info.dict()


class Ros2ShowMsgInterfaceTool(BaseTool):
    name: str = "Ros2ShowMsgInterface"
    description: str = """A tool for showing ros2 message interface in json format.
    usage:
    ```python
    ShowRos2MsgInterface.run({"msg_name": "geometry_msgs/msg/PoseStamped"})
    ```
    """

    args_schema: Type[Ros2MsgInterfaceInput] = Ros2MsgInterfaceInput

    def _run(self, msg_name: str):
        """Show ros2 message interface in json format."""
        msg_cls: Type = rosidl_runtime_py.utilities.get_interface(msg_name)
        try:
            msg_dict: OrderedDict = rosidl_runtime_py.convert.message_to_ordereddict(
                msg_cls()
            )
            return json.dumps(msg_dict)
        except NotImplementedError:
            # For action classes that can't be instantiated
            goal_dict: OrderedDict = rosidl_runtime_py.convert.message_to_ordereddict(
                msg_cls.Goal()
            )

            result_dict: OrderedDict = rosidl_runtime_py.convert.message_to_ordereddict(
                msg_cls.Result()
            )

            feedback_dict: OrderedDict = (
                rosidl_runtime_py.convert.message_to_ordereddict(msg_cls.Feedback())
            )
            return json.dumps(
                {"goal": goal_dict, "result": result_dict, "feedback": feedback_dict}
            )


class Ros2PubMessageTool(Ros2BaseTool):
    name: str = "PubRos2MessageTool"
    description: str = """A tool for publishing a message to a ros2 topic
    Example usage:

    ```python
    tool = Ros2PubMessageTool()
    tool.run(
        {
            "topic_name": "/some_topic",
            "msg_type": "geometry_msgs/Point",
            "msg_args": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
    )

    ```
    """

    args_schema: Type[PubRos2MessageToolInput] = PubRos2MessageToolInput

    def _build_msg(
        self, msg_type: str, msg_args: Dict[str, Any]
    ) -> Tuple[object, Type]:
        msg_cls: Type = import_message_from_str(msg_type)
        msg = msg_cls()
        rosidl_runtime_py.set_message.set_message_fields(msg, msg_args)
        return msg, msg_cls

    def _run(self, topic_name: str, msg_type: str, msg_args: Dict[str, Any]):
        """Publishes a message to the specified topic."""
        if "/msg/" not in msg_type:
            raise ValueError("msg_name must contain 'msg' in the name.")
        msg, msg_cls = self._build_msg(msg_type, msg_args)

        publisher = self.node.create_publisher(
            msg_cls, topic_name, 10
        )  # TODO(boczekbartek): infer qos profile from topic info

        msg.header.stamp = self.node.get_clock().now().to_msg()
        publisher.publish(msg)


class TopicInput(Ros2BaseInput):
    topic_name: str = Field(..., description="Ros2 topic name")


class GetMsgFromTopic(Ros2BaseTool):
    name: str = "get_msg_from_topic"
    description: str = "Get message from topic"
    args_schema: Type[TopicInput] = TopicInput
    response_format: str = "content_and_artifact"

    def _run(self, topic_name: str):
        msg = self.node.get_raw_message_from_topic(topic_name)
        if type(msg) is sensor_msgs.msg.Image:
            img = convert_ros_img_to_base64(msg)
            return "Got image", {"images": [img]}
        else:
            return str(msg), {}


class GetCameraImage(Ros2BaseTool):
    name: str = "get_camera_image"
    description: str = "get image from robots camera"
    response_format: str = "content_and_artifact"
    args_schema: Type[TopicInput] = TopicInput

    def _run(self, topic_name: str):
        msg = self.node.get_raw_message_from_topic(topic_name)
        img = convert_ros_img_to_base64(msg)
        return "Got image", {"images": [img]}
