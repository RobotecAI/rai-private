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

import uuid
from pprint import pformat
from typing import List, Optional, Set, cast

from langchain_core.messages import ToolCall
from pydantic import UUID1

from rai_hmi.chat_msgs import MissionMessage


class Memory:
    def __init__(self) -> None:
        # TODO(boczekbartek): add typehints
        self.mission_memory: List[MissionMessage] = []
        self.chat_memory = []
        self.tool_calls = {}
        self.missions_uids: Set[UUID1] = set()

    def register_tool_calls(self, tool_calls: List[ToolCall]):
        for tool_call in tool_calls:
            tool_call = cast(ToolCall, tool_call)
            tool_id = tool_call["id"]
            self.tool_calls[tool_id] = tool_call

    def add_mission(self, msg: MissionMessage):
        self.mission_memory.append(msg)
        self.missions_uids.add(msg.uid)

    def get_mission_memory(self, uid: Optional[str] = None) -> List[MissionMessage]:
        if not uid:
            return self.mission_memory

        _uid = uuid.UUID(uid)
        print(f"{self.missions_uids=}")
        if _uid not in self.missions_uids:
            raise AssertionError(f"Mission with {_uid=} not found")

        return [m for m in self.mission_memory if m.uid == _uid]

    def __repr__(self) -> str:
        return f"===> Chat <===\n{pformat(self.chat_memory)}\n\n===> Mission <===\n{pformat(self.mission_memory)}\n\n===> Tool calls <===\n{pformat(self.tool_calls)}"
