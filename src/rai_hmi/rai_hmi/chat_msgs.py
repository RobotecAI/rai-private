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

from dataclasses import dataclass
from typing import ClassVar, Optional, Tuple

from pydantic import UUID4

from rai_interfaces.action import Task


# ---------- Helpers ----------
class EMOJIS:
    human = "🧑‍💻"
    bot = "🤖"
    tool = "🛠️"
    unknown = "❓"
    success = "✅"
    failure = "❌"
    in_progress = "🕒"
    accepted = "👍"


@dataclass
class MissionMessage:
    AVATAR: ClassVar[str] = EMOJIS.unknown
    STATUS: ClassVar[str] = "Status unknown"

    uid: UUID4
    content: Optional[str] = ""

    def __repr__(self):
        return f"{self.STATUS}({self.uid})\n{self.content}"

    def render_steamlit(self) -> Tuple[str, str]:
        return self.AVATAR, str(self)


class MissionAcceptanceMessage(MissionMessage):
    AVATAR: ClassVar[str] = EMOJIS.accepted
    STATUS: ClassVar[str] = "Accepted"


class MissionFeedbackMessage(MissionMessage):
    AVATAR: ClassVar[str] = EMOJIS.in_progress
    STATUS: ClassVar[str] = "Feedback"


class MissionDoneMessage(MissionMessage):
    AVATAR: ClassVar[str] = EMOJIS.success
    STATUS: ClassVar[str] = "Done"

    def __init__(self, uid: UUID4, result: Task.Result):
        self.success = result.success
        self.report = result.report
        super().__init__(uid=uid, content=self.report)

    def __repr__(self):
        repr = super().__repr__()
        return f"{repr}. Success={self.success}\nReport={self.report}"
