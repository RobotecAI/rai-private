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

import logging
import time
import uuid
from typing import Callable, List, Optional

from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import (
    BaseMessage,
    HumanMessage,
    SystemMessage,
    ToolMessage,
)
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnableConfig
from langchain_core.tools import BaseTool, tool
from pydantic import BaseModel, Field

logger = logging.getLogger(__name__)

TASK_EXECUTOR_PROMPT = """
I am a robot's execution task system. I am responsible for executing the current task using reasoning and available tooling. I am focused on the task's completion.
When the simplest solution is not enough, I will reason about the task and use available tools to complete it.
My job is complete whenever the task is finished or the task is deemed impossible to complete.
Sometimes my job can be interrupted, and I will need to resume it later. I will be given a previous reasoning and state history to help me continue where I left off.
"""

TASK_SUMMARIZER_PROMPT = """
I am a robot's task summarizer. I am responsible for summarizing the task's completion status.
"""


@tool
def finish_execution(bool: bool) -> str:
    """This tool is used to stop execution when the task is considered done or unrecoverable."""
    return "The task is finished."


class TaskState(BaseModel):
    done: bool = Field(
        ..., description="True if the robot has reached the final state of the task."
    )
    summary: str = Field(
        ..., description="Message describing the robot's mission fulfilness."
    )


class Task(BaseModel):
    task: str = Field(..., description="The task to be executed by the robot.")


def task_executor(
    llm: BaseChatModel,
    task: Task,
    tools: List[BaseTool],
    task_executor_prompt: str = TASK_EXECUTOR_PROMPT,
    task_summarizer_prompt: str = TASK_SUMMARIZER_PROMPT,
) -> TaskState:
    agent_template = ChatPromptTemplate.from_messages(
        [
            ("system", task_executor_prompt),
            ("human", "The task: {task}"),
            ("placeholder", "{agent_scratchpad}"),
        ]
    )
    agent = create_tool_calling_agent(llm=llm, tools=tools, prompt=agent_template)
    agent_executor = AgentExecutor(
        agent=agent, tools=tools, verbose=True, return_intermediate_steps=True
    )

    agent_output = agent_executor.invoke({"task": task.task})

    agent_reasoning = f"intermediate_steps: {str(agent_output['intermediate_steps'])} final_output: {agent_output['output']}"
    history = [
        SystemMessage(content=task_summarizer_prompt),
        HumanMessage(content=agent_reasoning, name="agent"),
    ]
    task_state = llm.with_structured_output(schema=TaskState).invoke(history)

    return task_state


def structured_task_executor(
    llm: BaseChatModel,
    task: Task,
    tools: List[BaseTool],
    task_executor_prompt: str = TASK_EXECUTOR_PROMPT,
    task_summarizer_prompt: str = TASK_SUMMARIZER_PROMPT,
    callbacks: Optional[List[Callable[[], str]]] = None,
    rate: float = 1,
    max_iters: int = 10,
) -> TaskState:
    def get_latest_data():
        if callbacks is None:
            return ""
        return "\n".join([callback() for callback in callbacks])

    invoke_config = RunnableConfig(metadata={"task": task.task, "run_id": uuid.uuid4()})
    task_history: List[BaseMessage] = [
        SystemMessage(content=task_executor_prompt),
        HumanMessage(content=f"The task: {task.task}"),
    ]
    tools = tools + [finish_execution]

    tool_name_to_tool_map = {tool.name: tool for tool in tools}

    llm_with_tools = llm.bind_tools(tools)

    it = 0
    logger.info("Entering the task execution loop.")
    while it < max_iters:
        logger.info(f"Task execution loop iteration: {it+1}/{max_iters}")
        it += 1
        loop_start = time.time()
        state = get_latest_data()
        task_history += [HumanMessage(content=state, name="state")]
        output = llm_with_tools.invoke(task_history, config=invoke_config)

        tools_outputs: List[ToolMessage] = []
        if output.tool_calls:
            for tool_call in output.tool_calls:
                tool_name = tool_call["name"]
                tool_instance = tool_name_to_tool_map[tool_name]
                tool_output = tool_instance.run(tool_call["args"])
                tools_outputs.append(
                    ToolMessage(tool_call_id=tool_call["id"], content=tool_output)
                )

        task_history += [output]
        task_history += tools_outputs
        loop_end = time.time()

        time.sleep(max(0, 1 / rate - (loop_end - loop_start)))
        if "finish_execution" in [tool["name"] for tool in output.tool_calls]:
            logging.info("Finishing the task execution. Reason: finish requested.")
            break
    logger.info("Exiting the task execution loop.")
    history = [
        SystemMessage(content=task_summarizer_prompt),
        *task_history[1:],  # skip the first system message
    ]
    task_state = llm.with_structured_output(schema=TaskState).invoke(history)

    return task_state
