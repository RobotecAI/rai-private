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

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav2_msgs.action import DriveOnHeading
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.action.client import ActionClient


class RaiNavigator(BasicNavigator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.drive_on_heading_client = ActionClient(
            self, DriveOnHeading, "drive_on_heading"
        )

    def drive_on_heading(self, point: Point, speed: float, time_allowance: int):
        self.debug("Waiting for 'DriveOnHeading' action server")

        while not self.drive_on_heading_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = DriveOnHeading.Goal()
        goal_msg.target = point
        goal_msg.speed = speed
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(
            "Drive on heading to ({}, {}, {}) at {} m/s....".format(
                point.x, point.y, point.z, speed
            )
        )

        send_goal_future = self.drive_on_heading_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(
                "Goal to " + str(point.x) + " " + str(point.y) + " was rejected!"
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def destroy_node(self):
        super().destroy_node()
        self.drive_on_heading_client.destroy()
