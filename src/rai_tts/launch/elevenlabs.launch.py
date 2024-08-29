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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rai_tts"), "config", "elevenlabs.yaml"
    )
    launch_configuration = [
        DeclareLaunchArgument(
            "config_file",
            default_value=config,
            description="Path to the config file",
        ),
        Node(
            package="rai_tts",
            executable="tts_node",
            name="tts_node",
            parameters=[LaunchConfiguration("config_file")],
        ),
        ExecuteProcess(
            cmd=[
                "ffplay",
                "-f",
                "lavfi",
                "-i",
                "sine=frequency=432",
                "-af",
                "volume=0.01",
                "-nodisp",
                "-v",
                "0",
            ],
            name="ffplay_sine_wave",
            output="screen",
            condition=IfCondition(LaunchConfiguration("keep_speaker_busy")),
        ),
    ]

    return LaunchDescription(launch_configuration)
