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

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

from langchain_core.language_models import BaseChatModel
from langchain_core.prompts import ChatPromptTemplate


class RosoutBuffer:
    def __init__(self, llm: BaseChatModel, bufsize: int = 100) -> None:
        self.bufsize = bufsize
        self._buffer: Deque[str] = deque()
        self.template = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    "Shorten the following log keeping its format - for example merge simillar or repeating lines",
                ),
                ("human", "{rosout}"),
            ]
        )
        llm = llm
        self.llm = self.template | llm

    def clear(self):
        self._buffer.clear()

    def append(self, line: str):
        self._buffer.append(line)
        if len(self._buffer) > self.bufsize:
            self._buffer.popleft()

    def get_raw_logs(self, last_n: int = 30) -> str:
        return "\n".join(list(self._buffer)[-last_n:])

    def summarize(self):
        if len(self._buffer) == 0:
            return "No logs"
        buffer = self.get_raw_logs()
        response = self.llm.invoke({"rosout": buffer})
        return str(response.content)


@dataclass
class NodeDiscovery:
    topics_and_types: Dict[str, str] = field(default_factory=dict)
    services_and_types: Dict[str, str] = field(default_factory=dict)
    actions_and_types: Dict[str, str] = field(default_factory=dict)
    whitelist: Optional[List[str]] = field(default_factory=list)

    def set(self, topics, services, actions):
        def to_dict(info: List[Tuple[str, List[str]]]) -> Dict[str, str]:
            return {k: v[0] for k, v in info}

        self.topics_and_types = to_dict(topics)
        self.services_and_types = to_dict(services)
        self.actions_and_types = to_dict(actions)
        if self.whitelist is not None:
            self.__filter(self.whitelist)

    def __filter(self, whitelist: List[str]):
        for d in [
            self.topics_and_types,
            self.services_and_types,
            self.actions_and_types,
        ]:
            to_remove = [k for k in d if k not in whitelist]
            for k in to_remove:
                d.pop(k)

    def dict(self):
        return {
            "topics_and_types": self.topics_and_types,
            "services_and_types": self.services_and_types,
            "actions_and_types": self.actions_and_types,
        }
