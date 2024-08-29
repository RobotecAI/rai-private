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
import tempfile
from abc import abstractmethod
from typing import Optional

import requests
from elevenlabs.client import ElevenLabs

TTS_TRIES = 2


class TTSClient:
    @abstractmethod
    def synthesize_speech_to_file(self, text: str) -> str:
        pass

    @staticmethod
    def save_audio_to_file(audio_data: bytes, suffix: str) -> str:
        with tempfile.NamedTemporaryFile(
            delete=False, suffix=suffix
        ) as temp_audio_file:
            temp_audio_file.write(audio_data)
            temp_file_path = temp_audio_file.name

        return temp_file_path


class ElevenLabsClient(TTSClient):
    def __init__(self, voice: str, base_url: Optional[str] = None):
        self.base_url = base_url
        self.voice = voice
        api_key = os.getenv(key="ELEVENLABS_API_KEY")
        self.client = ElevenLabs(base_url=None, api_key=api_key)

    def synthesize_speech_to_file(self, text: str) -> str:
        tries = 0
        while tries < TTS_TRIES:
            try:
                response = self.client.generate(
                    text=text,
                    voice=self.voice,
                    optimize_streaming_latency=4,
                )
                audio_data = b"".join(response)
                return self.save_audio_to_file(audio_data, suffix=".mp3")
            except Exception as e:
                self.get_logger().warn(f"Error ocurred during sythesizing speech: {e}.")  # type: ignore
                tries += 1
        audio_data = b"".join(response)
        return self.save_audio_to_file(audio_data, suffix=".mp3")


class OpenTTSClient(TTSClient):
    def __init__(self, base_url: Optional[str] = None, voice: Optional[str] = None):
        self.base_url = base_url
        self.voice = voice

    def synthesize_speech_to_file(self, text: str) -> str:
        params = {
            "voice": self.voice,
            "text": text,
        }
        response = requests.get("http://localhost:5500/api/tts", params=params)
        response.raise_for_status()

        return self.save_audio_to_file(response.content, suffix=".wav")
