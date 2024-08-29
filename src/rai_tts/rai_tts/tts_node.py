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

import re
import subprocess
import threading
import time
from queue import PriorityQueue
from typing import NamedTuple, cast

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .tts_clients import ElevenLabsClient, OpenTTSClient, TTSClient


class TTSJob(NamedTuple):
    id: int
    file_path: str


class TTSNode(Node):
    def __init__(self):
        super().__init__("rai_tts_node")

        self.declare_parameter("tts_client", "opentts")
        self.declare_parameter("voice", "larynx:blizzard_lessac-glow_tts")
        self.declare_parameter("base_url", "http://localhost:5500/api/tts")
        self.declare_parameter("topic", "to_human")

        topic_param = self.get_parameter("topic").get_parameter_value().string_value  # type: ignore

        self.subscription = self.create_subscription(  # type: ignore
            String, topic_param, self.listener_callback, 10  # type: ignore
        )
        self.playing = False
        self.status_publisher = self.create_publisher(String, "tts_status", 10)  # type: ignore
        self.queue: PriorityQueue[TTSJob] = PriorityQueue()
        self.it: int = 0
        self.job_id: int = 0
        self.queued_job_id = 0
        self.tts_client = self._initialize_client()
        self.create_timer(0.01, self.status_callback)
        threading.Thread(target=self._process_queue).start()
        self.get_logger().info("TTS Node has been started")  # type: ignore
        self.threads_number = 0
        self.threads_max = 5
        self.thread_lock = threading.Lock()

    def status_callback(self):
        if self.threads_number == 0 and self.playing is False and self.queue.empty():
            self.status_publisher.publish(String(data="waiting"))
        else:
            self.status_publisher.publish(String(data="processing"))

    def listener_callback(self, msg: String):
        self.playing = True
        self.get_logger().info(  # type: ignore
            f"Registering new TTS job: {self.job_id} length: {len(msg.data)} chars."  # type: ignore
        )

        threading.Thread(
            target=self.start_synthesize_thread, args=(msg, self.job_id)  # type: ignore
        ).start()
        self.job_id += 1

    def start_synthesize_thread(self, msg: String, job_id: int):
        while True:
            with self.thread_lock:
                if (
                    self.threads_number < self.threads_max
                    and self.queued_job_id == job_id
                ):
                    threading.Thread(
                        target=self.synthesize_speech, args=(job_id, msg.data)  # type: ignore
                    ).start()
                    self.threads_number += 1
                    self.queued_job_id += 1
                    return

    def synthesize_speech(
        self,
        id: int,
        text: str,
    ) -> str:
        text = self._preprocess_text(text)
        if id > 0:
            time.sleep(0.5)
        temp_file_path = self.tts_client.synthesize_speech_to_file(text)
        self.get_logger().info(f"Job {id} completed.")  # type: ignore
        tts_job = TTSJob(id, temp_file_path)
        self.queue.put(tts_job)
        with self.thread_lock:
            self.threads_number -= 1

        return temp_file_path

    def _process_queue(self):
        while rclpy.ok():
            time.sleep(0.01)
            if not self.queue.empty():
                if self.queue.queue[0][0] == self.it:
                    self.it += 1
                    tts_job = self.queue.get()
                    self.get_logger().info(  # type: ignore
                        f"Playing audio for job {tts_job.id}. {tts_job.file_path}"
                    )
                    self._play_audio(tts_job.file_path)

    def _play_audio(self, filepath: str):
        self.playing = True
        self.status_publisher.publish(String(data="playing"))
        subprocess.run(
            ["ffplay", "-v", "0", "-nodisp", "-autoexit", filepath],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.get_logger().debug(f"Playing audio: {filepath}")  # type: ignore
        self.playing = False

    def _initialize_client(self) -> TTSClient:
        tts_client_param = cast(str, self.get_parameter("tts_client").get_parameter_value().string_value)  # type: ignore
        voice_param = cast(str, self.get_parameter("voice").get_parameter_value().string_value)  # type: ignore
        base_url_param = cast(str, self.get_parameter("base_url").get_parameter_value().string_value)  # type: ignore

        if tts_client_param == "opentts":
            return OpenTTSClient(
                base_url=base_url_param,
                voice=voice_param,
            )
        elif tts_client_param == "elevenlabs":
            return ElevenLabsClient(
                voice=voice_param,
                base_url=base_url_param,
            )
        else:
            raise ValueError(f"Unknown TTS client: {tts_client_param}")

    def _preprocess_text(self, text: str) -> str:
        """Remove emojis from text."""
        emoji_pattern = re.compile(
            "["
            "\U0001F600-\U0001F64F"  # emoticons
            "\U0001F300-\U0001F5FF"  # symbols & pictographs
            "\U0001F680-\U0001F6FF"  # transport & map symbols
            "\U0001F1E0-\U0001F1FF"  # flags (iOS)
            "]+",
            flags=re.UNICODE,
        )
        text = emoji_pattern.sub(r"", text)
        return text


def main():
    rclpy.init()

    tts_node = TTSNode()

    rclpy.spin(tts_node)

    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
