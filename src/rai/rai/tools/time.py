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

import time
from typing import Type

from langchain_core.tools import BaseTool
from pydantic import BaseModel, Field


class WaitForSecondsToolInput(BaseModel):
    """Input for the WaitForSecondsTool tool."""

    seconds: int = Field(..., description="The number of seconds to wait")


class WaitForSecondsTool(BaseTool):
    """Wait for a specified number of seconds"""

    name: str = "WaitForSecondsTool"
    description: str = (
        "A tool for waiting. "
        "Useful for pausing execution for a specified number of seconds. "
        "Input should be the number of seconds to wait."
        "Maximum allowed time is 5 seconds"
    )

    args_schema: Type[WaitForSecondsToolInput] = WaitForSecondsToolInput

    def _run(self, seconds: int):
        """Waits for the specified number of seconds."""
        if seconds > 5:
            seconds = 5
        time.sleep(seconds)
        return f"Waited for {seconds} seconds."
