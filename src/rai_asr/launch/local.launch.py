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
                "model_name",
                default_value="base",
                description="Model name for the ASR model",
            ),
            DeclareLaunchArgument(
                "model_vendor",
                default_value="whisper",
                description="Model vendor of the ASR",
            ),
            DeclareLaunchArgument(
                "silence_grace_period",
                default_value="1.0",
                description="Grace period in seconds after silence to stop recording",
            ),
            DeclareLaunchArgument(
                "use_wake_word",
                default_value="False",
                description="Whether to use wake word detection",
            ),
            DeclareLaunchArgument(
                "wake_word_model",
                default_value="",
                description="Wake word model to use",
            ),
            DeclareLaunchArgument(
                "wake_word_threshold",
                default_value="0.5",
                description="Threshold for wake word detection",
            ),
            DeclareLaunchArgument(
                "vad_threshold",
                default_value="0.5",
                description="Threshold for voice activity detection",
            ),
            Node(
                package="rai_asr",
                executable="asr_node",
                name="rai_asr",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "recording_device": LaunchConfiguration("recording_device"),
                        "language": LaunchConfiguration("language"),
                        "model_name": LaunchConfiguration("model_name"),
                        "model_vendor": LaunchConfiguration("model_vendor"),
                        "silence_grace_period": LaunchConfiguration(
                            "silence_grace_period"
                        ),
                        "use_wake_word": LaunchConfiguration("use_wake_word"),
                        "wake_word_model": LaunchConfiguration("wake_word_model"),
                        "wake_word_threshold": LaunchConfiguration(
                            "wake_word_threshold"
                        ),
                        "vad_threshold": LaunchConfiguration("vad_threshold"),
                    }
                ],
            ),
        ]
    )
