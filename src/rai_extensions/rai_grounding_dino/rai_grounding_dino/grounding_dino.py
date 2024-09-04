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

import os
import subprocess
from pathlib import Path
from typing import TypedDict

import rclpy
from rai_grounding_dino.boxer import GDBoxer
from rclpy.node import Node
from sensor_msgs.msg import Image

from rai_interfaces.msg import RAIDetectionArray
from rai_interfaces.srv import RAIGroundingDino


class GDRequest(TypedDict):
    classes: str
    box_threshold: float
    text_threshold: float
    source_img: Image


class GDinoService(Node):
    def __init__(self):
        super().__init__(node_name="grounding_dino", parameter_overrides=[])
        self.srv = self.create_service(
            RAIGroundingDino, "grounding_dino_classify", self.classify_callback
        )
        self.declare_parameter("weights_path", "")
        try:
            weight_path = self.get_parameter("weights_path").value
            assert isinstance(weight_path, str)
            if self.get_parameter("weights_path").value == "":
                weight_path = self._init_weight_path()
            self.boxer = GDBoxer(weight_path)
        except Exception:
            self.get_logger().error("Could not load model")
            raise Exception("Could not load model")

    def _init_weight_path(self) -> Path:
        try:
            found_path = subprocess.check_output(
                ["ros2", "pkg", "prefix", "rai_grounding_dino"]
            ).decode("utf-8")
            install_path = (
                Path(found_path.strip())
                / "share"
                / "weights"
                / "groundingdino_swint_ogc.pth"
            )
            # make sure the file exists
            if install_path.exists():
                return install_path
            else:
                self._download_weights(install_path)
                return install_path

        except Exception:
            self.get_logger().error("Could not find package path")
            raise Exception("Could not find package path")

    def _download_weights(self, path: Path):
        try:
            os.makedirs(path.parent, exist_ok=True)
            subprocess.run(
                [
                    "wget",
                    "https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth",
                    "-O",
                    path,
                ]
            )
        except Exception:
            self.get_logger().error("Could not download weights")
            raise Exception("Could not download weights")

    def classify_callback(self, request, response: RAIDetectionArray):
        self.get_logger().info(
            f"Request received: {request.classes}, {request.box_threshold}, {request.text_threshold}"
        )

        class_array = request.classes.split(",")
        class_array = [class_name.strip() for class_name in class_array]
        class_dict = {class_name: i for i, class_name in enumerate(class_array)}

        boxes = self.boxer.get_boxes(
            request.source_img,
            class_array,
            request.box_threshold,
            request.text_threshold,
        )

        response.detections.detections = [
            box.to_detection_msg(class_dict, self.get_clock().now().to_msg())
            for box in boxes
        ]
        response.detections.header.stamp = self.get_clock().now().to_msg()
        response.detections.detection_classes = class_array

        return response


def main(args=None):
    rclpy.init(args=args)

    gdino_service = GDinoService()

    try:
        rclpy.spin(gdino_service)
    except KeyboardInterrupt:
        gdino_service.get_logger().info("Shutting down")
    except Exception as e:
        gdino_service.get_logger().error(f"Error: {e}")
    finally:
        gdino_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
