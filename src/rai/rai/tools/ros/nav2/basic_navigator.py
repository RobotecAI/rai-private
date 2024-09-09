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

from geometry_msgs.msg import Point
from langchain.tools import tool

from rai.tools.ros.nav2.navigator import RaiNavigator


@tool
def spin_robot(degrees_rad: float) -> str:
    """Use this tool to spin the robot."""
    navigator = RaiNavigator()
    navigator.spin(spin_dist=degrees_rad)
    return "Robot spinning."


@tool
def drive_forward(distance_m: float) -> str:
    """Use this tool to drive the robot forward."""
    navigator = RaiNavigator()
    p = Point()
    p.x = distance_m

    navigator.drive_on_heading(p, 0.5, 10)
    return "Robot driving forward."
