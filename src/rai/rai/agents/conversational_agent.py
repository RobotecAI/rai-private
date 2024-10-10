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
from functools import partial
from typing import Any, List, Literal, Optional, TypedDict, Union

from langchain.chat_models.base import BaseChatModel
from langchain_core.messages import AnyMessage, BaseMessage, SystemMessage
from langchain_core.tools import BaseTool
from langgraph.graph import END, START, StateGraph
from pydantic import BaseModel, ValidationError
from rclpy.impl.rcutils_logger import RcutilsLogger

loggers_type = Union[RcutilsLogger, logging.Logger]


class State(TypedDict):
    messages: List[BaseMessage]


def agent(llm: BaseChatModel, logger: loggers_type, system_prompt: str, state: State):
    logger.info("Running thinker")

    # If there are no messages, do nothing
    if len(state["messages"]) == 0:
        return state

    # Insert system message if not already present
    if not isinstance(state["messages"][0], SystemMessage):
        state["messages"].insert(0, SystemMessage(content=system_prompt))
    ai_msg = llm.invoke(state["messages"])
    state["messages"].append(ai_msg)
    return state


def reporter(
    llm: BaseChatModel, logger: loggers_type, state: State, report_template: BaseModel
):
    logger.info("Summarizing the conversation")
    n_tries = 5
    ai_msg = None
    for i in range(n_tries):
        try:
            ai_msg = llm.with_structured_output(report_template).invoke(
                state["messages"]
            )
            break
        except ValidationError:
            logger.info(
                f"Failed to summarize using given template. Repeating: {i}/{n_tries}"
            )

    if ai_msg is None:
        logger.info("Failed to summarize. Trying without template")
        ai_msg = llm.invoke(state["messages"])

    state["messages"].append(ai_msg)
    return state


def tools_condition(
    state: Union[list[AnyMessage], dict[str, Any]],
) -> Literal["tools", "reporter"]:
    if isinstance(state, list):
        ai_message = state[-1]
    elif messages := state.get("messages", []):
        ai_message = messages[-1]
    else:
        raise ValueError(f"No messages found in input state to tool_edge: {state}")
    if hasattr(ai_message, "tool_calls") and len(ai_message.tool_calls) > 0:
        return "tools"
    return "reporter"


def create_conversational_agent(
    llm: BaseChatModel,
    tools: List[BaseTool],
    system_prompt: str,
    report_template=None,
    logger: Optional[RcutilsLogger | logging.Logger] = None,
    debug=False,
):
    _logger = None
    if isinstance(logger, RcutilsLogger):
        _logger = logger
    else:
        _logger = logging.getLogger(__name__)

    _logger.info("Creating state based agent")

    llm_with_tools = llm

    workflow = StateGraph(State)
    workflow.add_node("thinker", partial(agent, llm_with_tools, _logger, system_prompt))
    workflow.add_node(
        "reporter", partial(reporter, llm, _logger, report_template=report_template)
    )

    workflow.add_edge(START, "thinker")
    workflow.add_edge("thinker", "reporter")
    workflow.add_edge("reporter", END)

    app = workflow.compile(debug=debug)
    _logger.info("State based agent created")
    return app
