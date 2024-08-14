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
import logging
import subprocess
from typing import Any, Callable, cast

import cv2
import rclpy
import rclpy.qos
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

from ..tools.ros.utils import wait_for_message


class SingleMessageGrabber:
    def __init__(
        self,
        topic: str,
        message_type: type,
        timeout_sec: int,
        logging_level: int = logging.INFO,
        postprocess: Callable[[Any], Any] = lambda x: x,
    ):
        self.topic = topic
        self.message_type = message_type
        self.timeout_sec = timeout_sec
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging_level)
        self.postprocess = getattr(self, "postprocess", postprocess)

    def grab_message(self) -> Any:
        node = rclpy.create_node(self.__class__.__name__ + "_node")  # type: ignore
        qos_profile = rclpy.qos.qos_profile_sensor_data
        if (
            self.topic == "/map"
        ):  # overfitting to husarion TODO(maciejmajek): find a better way
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                lifespan=Duration(seconds=0),
                deadline=Duration(seconds=0),
                liveliness=QoSLivelinessPolicy.AUTOMATIC,
                liveliness_lease_duration=Duration(seconds=0),
            )
        success, msg = wait_for_message(
            self.message_type,
            node,
            self.topic,
            qos_profile=qos_profile,
            time_to_wait=self.timeout_sec,
        )

        if success:
            self.logger.info(
                f"Received message of type {self.message_type.__class__.__name__} from topic {self.topic}"  # type: ignore
            )
        else:
            self.logger.error(
                f"Failed to receive message of type {self.message_type.__class__.__name__} from topic {self.topic}"  # type: ignore
            )

        node.destroy_node()
        return msg

    def get_data(self) -> Any:
        msg = self.grab_message()
        return self.postprocess(msg)


class SingleImageGrabber(SingleMessageGrabber):
    def __init__(
        self, topic: str, timeout_sec: int = 10, logging_level: int = logging.INFO
    ):
        self.topic = topic
        super().__init__(
            topic=topic,
            message_type=Image,
            timeout_sec=timeout_sec,
            logging_level=logging_level,
        )

    def postprocess(self, msg: Image) -> str:
        bridge = CvBridge()
        cv_image = cast(cv2.Mat, bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))  # type: ignore
        if cv_image.shape[-1] == 4:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGB)
            base64_image = base64.b64encode(
                bytes(cv2.imencode(".png", cv_image)[1])
            ).decode("utf-8")
            return base64_image
        else:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        image_data = cv2.imencode(".png", cv_image)[1].tostring()  # type: ignore
        base64_image = base64.b64encode(image_data).decode("utf-8")  # type: ignore
        return base64_image


class ReadAvailableTopics:
    def __init__(self, logging_level: int = logging.INFO):
        self.logging_level = logging_level
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(self.logging_level)

    def get_data(self):
        command = "ros2 topic list"
        output = subprocess.check_output(command, shell=True).decode("utf-8")
        return output


class ReadAvailableNodes:
    def __init__(self, logging_level: int = logging.INFO):
        self.logging_level = logging_level
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(self.logging_level)

    def get_data(self):
        command = "ros2 node list"
        output = subprocess.check_output(command, shell=True).decode("utf-8")
        return output


class ReadAvailableServices:
    def __init__(self, logging_level: int = logging.INFO):
        self.logging_level = logging_level
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(self.logging_level)

    def get_data(self):
        command = (
            "for service in $(ros2 service list); do\n"
            'echo -n "service $service "\n'
            'echo "type: $(ros2 service type $service)"\n'
            "done"
        )
        output = subprocess.check_output(command, shell=True).decode("utf-8")
        return output


class ReadAvailableActions:
    def __init__(self, logging_level: int = logging.INFO):
        self.logging_level = logging_level
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(self.logging_level)

    def get_data(self):
        command = "ros2 action list"
        output = subprocess.check_output(command, shell=True).decode("utf-8")
        return output


class TF2Listener(Node):
    def __init__(self):
        super().__init__("tf2_listener")

        # Create a buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # This will store the transform when received
        self.transform = None

    def get_transform(self):
        try:
            # Lookup transform between base_link and map
            now = rclpy.time.Time()
            self.transform = self.tf_buffer.lookup_transform("map", "base_link", now)
        except Exception as ex:
            self.get_logger().debug(f"Could not transform: {ex}")


class TF2TransformFetcher:
    def get_data(self):
        node = TF2Listener()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while rclpy.ok() and node.transform is None:
                node.get_transform()
                rclpy.spin_once(node, timeout_sec=1.0)
        except KeyboardInterrupt:
            pass

        transform = node.transform
        node.destroy_node()
        return transform
