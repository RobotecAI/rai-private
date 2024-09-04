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

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from rai_interfaces.srv import RAIGroundingDino  # CHANGE


class GDClientExample(Node):
    def __init__(self):
        super().__init__(node_name="GDClientExample", parameter_overrides=[])
        self.declare_parameter("image_path", "")
        self.cli = self.create_client(RAIGroundingDino, "grounding_dino_classify")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = RAIGroundingDino.Request()
        self.bridge = CvBridge()

    def send_request(self):
        image_path = self.get_parameter("image_path").value
        assert isinstance(image_path, str)
        img = cv2.imread(image_path)
        # convert img to numpy array
        img = np.array(img)
        self.req.source_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.req.classes = "dragon , lizard , dinosaur"
        self.req.box_threshold = 0.4
        self.req.text_threshold = 0.4

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = GDClientExample()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info("Service call failed %r" % (e,))
            else:
                minimal_client.get_logger().info(f"{response.detections}")  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
