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

from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, Field

STATE_ANALYZER_PROMPT = """
I am a part of a robot's supervising system. I am responsible for analyzing current robot's state with respect to the planned mission.
"""


class State(BaseModel):
    anomaly: bool = Field(
        ..., description="True if the robot's state is not as expected."
    )
    should_continue: bool = Field(
        ...,
        description="True if the robot should continue the task. False if the task is unrecoverable.",
    )
    message: str = Field(..., description="Message describing the robot's state.")


def robot_state_analyzer(
    llm: BaseChatModel,
    task: str,
    state: str,
    state_analyzer_prompt: str = STATE_ANALYZER_PROMPT,
) -> State:

    template = ChatPromptTemplate.from_messages(
        [
            ("system", state_analyzer_prompt),
            ("human", "Current task: {task}\nThe robot's state: {state}"),
        ]
    )

    status_analyzer = template | llm.with_structured_output(schema=State)
    return status_analyzer.invoke({"task": task, "state": state})
