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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "recording_device",
                default_value="0",
                description="Microphone device number. See available by running python -c 'import sounddevice as sd; print(sd.query_devices())'",
            ),
            DeclareLaunchArgument(
                "language",
                default_value="en",
                description="Language code for the ASR model",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="whisper-1",
                description="Model type for the ASR model",
            ),
            DeclareLaunchArgument(
                "silence_grace_period",
                default_value="2.0",
                description="Grace period in seconds after silence to stop recording",
            ),
            DeclareLaunchArgument(
                "sample_rate",
                default_value="0",
                description="Sample rate for audio capture (0 for auto-detect)",
            ),
            Node(
                package="rai_asr",
                executable="asr_node",
                name="rai_asr",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "language": LaunchConfiguration("language"),
                        "model": LaunchConfiguration("model"),
                        "silence_grace_period": LaunchConfiguration(
                            "silence_grace_period"
                        ),
                        "sample_rate": LaunchConfiguration("sample_rate"),
                    }
                ],
            ),
        ]
    )
