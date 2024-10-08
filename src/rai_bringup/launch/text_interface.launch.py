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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_description_package_arg = DeclareLaunchArgument(
        "robot_description_package",
        default_value="default_whoami",
        description="Robot's whoami package name",
    )

    whoami_node = Node(
        package="rai_whoami",
        executable="rai_whoami_node",
        name="rai_whoami_node",
        parameters=[
            {
                "robot_description_package": LaunchConfiguration(
                    "robot_description_package"
                )
            }
        ],
        output="screen",
    )

    streamlit_cmd = ExecuteProcess(
        cmd=[
            "streamlit",
            "run",
            "src/rai_hmi/rai_hmi/text_hmi.py",
            LaunchConfiguration("robot_description_package"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [robot_description_package_arg, whoami_node, streamlit_cmd]
    )
