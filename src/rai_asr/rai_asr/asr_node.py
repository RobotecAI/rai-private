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

import io
import threading
import time
from datetime import datetime, timedelta
from functools import partial
from typing import Literal

import numpy as np
import rclpy
import sounddevice as sd
import torch
from openai import OpenAI
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.io import wavfile
from std_msgs.msg import String

SAMPLING_RATE = 16000


class ASRNode(Node):
    def __init__(self):
        super().__init__("rai_asr")
        self._declare_parameters()
        self._initialize_vad_model()
        self._setup_node_components()
        self._initialize_variables()
        self._setup_publishers_and_subscribers()
        self._load_whisper_model()

    def _declare_parameters(self):
        self.declare_parameter(
            "recording_device",
            0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description=(
                    "Recording device number. See available by running"
                    "python -c 'import sounddevice as sd; print(sd.query_devices())'"
                ),
            ),
        )
        self.declare_parameter(
            "language",
            "en",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Language code for the ASR model",
            ),
        )
        self.declare_parameter(
            "model",
            "whisper-1",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Model type for the ASR model",
            ),
        )
        self.declare_parameter(
            "silence_grace_period",
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Grace period in seconds after silence to stop recording",
            ),
        )

    def _initialize_vad_model(self):
        model, (_, _, _, VADIterator, _) = torch.hub.load(
            repo_or_dir="snakers4/silero-vad",
            model="silero_vad",
        )
        self.vad_iterator = VADIterator(model, sampling_rate=SAMPLING_RATE)

    def _setup_node_components(self):
        self.callback_group = ReentrantCallbackGroup()
        self.sample_rate = SAMPLING_RATE

    def _initialize_variables(self):
        self.is_recording = False
        self.audio_buffer = []
        self.silence_start_time = None
        self.last_transcription_time = 0
        self.hmi_lock = False
        self.tts_lock = False

        silence_grace_period = (
            self.get_parameter("silence_grace_period")
            .get_parameter_value()
            .double_value
        )  # type: ignore
        self.whisper_model = (
            self.get_parameter("model").get_parameter_value().string_value
        )  # type: ignore
        self.language = (
            self.get_parameter("language").get_parameter_value().string_value
        )  # type: ignore

        self.grace_period = timedelta(seconds=silence_grace_period)
        self.transcription_recording_timeout = 5
        self.recording_device_number = self.get_parameter("recording_device").get_parameter_value().integer_value  # type: ignore

    def _setup_publishers_and_subscribers(self):
        self.transcription_publisher = self.create_publisher(String, "/from_human", 10)
        self.status_publisher = self.create_publisher(String, "/asr_status", 10)
        self.tts_status_subscriber = self.create_subscription(
            String,
            "/tts_status",
            self.tts_status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.hmi_status_subscriber = self.create_subscription(
            String,
            "/hmi_status",
            self.hmi_status_callback,
            10,
            callback_group=self.callback_group,
        )

    def _load_whisper_model(self):
        self.openai_client = OpenAI()
        self.model = partial(
            self.openai_client.audio.transcriptions.create, model=self.whisper_model
        )

    def tts_status_callback(self, msg: String):
        if msg.data == "processing":
            self.tts_lock = True
        elif msg.data == "waiting":
            self.tts_lock = False

    def hmi_status_callback(self, msg: String):
        if msg.data == "processing":
            self.hmi_lock = True
        elif msg.data == "waiting":
            self.hmi_lock = False

    def capture_sound(self):
        window_size_samples = 512 if self.sample_rate == 16000 else 256
        stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            device=self.recording_device_number,
            dtype="int16",
            blocksize=window_size_samples,
        )
        stream.start()
        self.get_logger().info(
            "Voice Activity Detection enabled. Waiting for speech..."
        )
        while True:
            audio_data, _ = stream.read(window_size_samples)
            audio_data = audio_data.flatten()

            asr_lock = (
                time.time()
                < self.last_transcription_time + self.transcription_recording_timeout
            )
            if asr_lock or self.hmi_lock or self.tts_lock:
                continue

            speech_dict = self.vad_iterator(audio_data, return_seconds=True)
            if speech_dict:
                if not self.is_recording:
                    self.start_recording()
                self.audio_buffer.append(audio_data)
                self.silence_start_time = None
                if "end" in speech_dict.keys():
                    self.silence_start_time = datetime.now()
            elif self.is_recording:
                self.audio_buffer.append(audio_data)
                if self.silence_start_time is not None:
                    if datetime.now() - self.silence_start_time > self.grace_period:
                        self.stop_recording_and_transcribe()

    def start_recording(self):
        self.get_logger().info("Recording...")
        self.publish_status("recording")
        self.is_recording = True
        self.audio_buffer = []
        self.silence_start_time = None

    def stop_recording_and_transcribe(self):
        self.get_logger().info("Stopped recording. Transcribing...")
        self.is_recording = False
        self.publish_status("transcribing")
        self.transcribe_audio()
        self.publish_status("waiting")

    def transcribe_audio(self):
        self.get_logger().info("Calling ASR model")
        combined_audio = np.concatenate(self.audio_buffer)

        with io.BytesIO() as temp_wav_buffer:
            wavfile.write(temp_wav_buffer, self.sample_rate, combined_audio)
            temp_wav_buffer.seek(0)
            temp_wav_buffer.name = "temp.wav"

            response = self.model(file=temp_wav_buffer, language=self.language)
            transcription = response.text
            if transcription.lower() in ["you", ""]:
                self.get_logger().info(f"Dropping transcription: '{transcription}'")
                self.publish_status("dropping")
            else:
                self.get_logger().info(f"Transcription: {transcription}")
                self.publish_transcription(transcription)

        self.last_transcription_time = time.time()

        self.audio_buffer = []

    def publish_transcription(self, transcription: str):
        msg = String()
        msg.data = transcription
        self.transcription_publisher.publish(msg)

    def publish_status(
        self, status: Literal["recording", "transcribing", "dropping", "waiting"]
    ):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=node.capture_sound)
    thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
