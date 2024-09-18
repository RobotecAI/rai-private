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

PLANNER_PROMPT = """
I am a part of a robot's planning system. I am responsible for planning the robot's actions by splitting tasks in to atomic steps.
"""


class StepsList(BaseModel):
    steps: list[str] = Field(
        ...,
        description="List of steps to be performed by the robot to complete the task.",
    )


def robot_agnostic_planner(
    llm: BaseChatModel,
    robot_constitution: str,
    task: str,
    planner_prompt: str = PLANNER_PROMPT,
) -> StepsList:
    planner_template = ChatPromptTemplate.from_messages(
        [
            ("system", robot_constitution),
            ("system", planner_prompt),
            ("human", "The task: {input}"),
        ]
    )

    planner = llm.with_structured_output(schema=StepsList)

    chain = planner_template | planner
    return chain.invoke({"input": task})


def robot_specific_planner(
    llm: BaseChatModel,
    robot_constitution: str,
    platform_specific_information: str,
    task: str,
    planner_prompt: str = PLANNER_PROMPT,
) -> StepsList:
    planner_template = ChatPromptTemplate.from_messages(
        [
            ("system", robot_constitution),
            ("system", platform_specific_information),
            ("system", planner_prompt),
            ("human", "The task: {input}"),
        ]
    )

    planner = llm.with_structured_output(schema=StepsList)

    chain = planner_template | planner
    return chain.invoke({"input": task})
