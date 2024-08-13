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
from typing import Type, cast

import cv2
import sensor_msgs.msg
from cv_bridge import CvBridge
from rosidl_parser.definition import NamespacedType
from rosidl_runtime_py.import_message import import_message_from_namespaced_type
from rosidl_runtime_py.utilities import get_namespaced_type


def import_message_from_str(msg_type: str) -> Type[object]:
    msg_namespaced_type: NamespacedType = get_namespaced_type(msg_type)
    return import_message_from_namespaced_type(msg_namespaced_type)


def convert_ros_img_to_base64(msg: sensor_msgs.msg.Image) -> str:
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
