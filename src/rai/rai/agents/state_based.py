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
from functools import partial
from typing import Any, Callable, Dict, List, Literal, Optional, TypedDict, Union

from langchain.chat_models.base import BaseChatModel
from langchain_core.language_models.base import LanguageModelInput
from langchain_core.messages import AnyMessage, BaseMessage, HumanMessage, SystemMessage
from langchain_core.runnables import Runnable
from langchain_core.tools import BaseTool
from langgraph.graph import END, START, StateGraph
from langgraph.graph.graph import CompiledGraph
from langgraph.prebuilt.tool_node import msg_content_output
from pydantic import BaseModel, Field, ValidationError
from rclpy.impl.rcutils_logger import RcutilsLogger

from rai.agents.tool_runner import ToolRunner
from rai.messages import HumanMultimodalMessage

loggers_type = Union[RcutilsLogger, logging.Logger]


class State(TypedDict):
    messages: List[BaseMessage]


class Report(BaseModel):
    problem: str = Field(..., title="Problem", description="The problem that occurred")
    solution: str = Field(
        ..., title="Solution", description="The solution to the problem"
    )
    outcome: str = Field(
        ..., title="Outcome", description="The outcome of the solution"
    )
    steps: List[str] = Field(
        ..., title="Steps", description="The steps taken to solve the problem"
    )
    success: bool = Field(
        ..., title="Success", description="Whether the problem was solved"
    )
    response_to_user: str = Field(
        ..., title="Response", description="The response to the user"
    )


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


def thinker(llm: BaseChatModel, logger: loggers_type, state: State):
    logger.info("Running thinker")
    prompt = (
        "Based on the data provided, reason about the situation. "
        "Analyze the context, identify any problems or challenges, "
        "and consider potential implications."
    )
    ai_msg = llm.invoke([SystemMessage(content=prompt)] + state["messages"])
    state["messages"].append(ai_msg)
    return state


def decider(
    llm: Runnable[LanguageModelInput, BaseMessage], logger: loggers_type, state: State
):
    logger.info("Running decider")
    prompt = (
        "Based on the previous information, make a decision using tools if necessary. "
        "If you are sure the problem has been solved, do not request any tools. "
        "Request one tool at a time."
    )

    input = state["messages"] + [HumanMessage(prompt)]
    ai_msg = llm.invoke(input)
    state["messages"].append(ai_msg)
    if ai_msg.tool_calls:
        logger.info("Tools requested: {}".format(ai_msg.tool_calls))
    return state


def reporter(
    llm: BaseChatModel, logger: loggers_type, state: State, report_template: BaseModel
):
    logger.info("Summarizing the conversation")
    prompt = (
        "You are the reporter. Your task is to summarize what happened previously. "
        "Make sure to mention the problem, solution and the outcome. Prepare clear response to the user."
    )
    n_tries = 5
    ai_msg = None
    for i in range(n_tries):
        try:
            ai_msg = llm.with_structured_output(report_template).invoke(
                [SystemMessage(content=prompt)] + state["messages"]
            )
            break
        except ValidationError:
            logger.info(
                f"Failed to summarize using given template. Repeating: {i}/{n_tries}"
            )

    if ai_msg is None:
        logger.info("Failed to summarize. Trying without template")
        ai_msg = llm.invoke([SystemMessage(content=prompt)] + state["messages"])

    state["messages"].append(ai_msg)
    return state


def retriever_wrapper(
    state_retriever: Callable[[], Dict[str, Any]], logger: loggers_type, state: State
):
    """This wrapper is used to retrieve multimodal information from the output of state_retriever."""
    ts = time.perf_counter()
    retrieved_info = state_retriever()
    te = time.perf_counter() - ts
    logger.info(f"Retrieved state in {te} seconds")

    images = retrieved_info.pop("images", [])
    audios = retrieved_info.pop("audios", [])

    info = msg_content_output(retrieved_info)
    state["messages"].append(
        HumanMultimodalMessage(
            content=f"Retrieved state: {info}", images=images, audios=audios
        )
    )
    return state


def create_state_based_agent(
    llm: BaseChatModel,
    tools: List[BaseTool],
    state_retriever: Callable[[], Dict[str, Any]],
    report_template=Report,
    logger: Optional[RcutilsLogger | logging.Logger] = None,
) -> CompiledGraph:
    _logger = None
    if isinstance(logger, RcutilsLogger):
        _logger = logger
    else:
        _logger = logging.getLogger(__name__)

    _logger.info("Creating state based agent")

    llm_with_tools = llm.bind_tools(tools)
    tool_node = ToolRunner(tools=tools, logger=_logger)

    workflow = StateGraph(State)
    workflow.add_node(
        "state_retriever", partial(retriever_wrapper, state_retriever, _logger)
    )
    workflow.add_node("tools", tool_node)
    # workflow.add_node("thinker", partial(thinker, llm, _logger))
    workflow.add_node("decider", partial(decider, llm_with_tools, _logger))
    workflow.add_node(
        "reporter", partial(reporter, llm, _logger, report_template=report_template)
    )

    workflow.add_edge(START, "state_retriever")
    workflow.add_edge("state_retriever", "decider")
    # workflow.add_edge("thinker", "decider")
    workflow.add_edge("tools", "state_retriever")
    workflow.add_edge("reporter", END)
    workflow.add_conditional_edges(
        "decider",
        tools_condition,
    )

    app = workflow.compile()
    _logger.info("State based agent created")
    return app
