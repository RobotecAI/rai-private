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

import datetime
import logging
import os
from typing import Callable, List, Literal, Sequence, Union, cast

import coloredlogs
import rclpy
from langchain.globals import set_llm_cache
from langchain_community.cache import RedisCache
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import (
    AIMessage,
    BaseMessage,
    HumanMessage,
    SystemMessage,
    ToolMessage,
)
from langchain_core.runnables import RunnableConfig
from langfuse.callback import CallbackHandler
from redis import Redis

from rai.history_saver import HistorySaver
from rai.node import RaiNode
from rai.scenario_engine.messages import (
    AgentLoop,
    FutureAiMessage,
    HumanMultimodalMessage,
)
from rai.scenario_engine.tool_runner import run_requested_tools

__all__ = [
    "ScenarioRunner",
    "ScenarioPartType",
    "ScenarioType",
    "ConditionalScenario",
]

coloredlogs.install()  # type: ignore


class ConditionalScenario:
    def __init__(
        self,
        if_true: "ScenarioType",
        if_false: "ScenarioType",
        condition: Callable[[Sequence[BaseMessage]], bool],
    ):
        self.if_true = if_true
        self.if_false = if_false
        self.condition = condition

    def __call__(self, messages: Sequence[BaseMessage]):
        response = self.condition(messages)
        if response:
            return self.if_true
        return self.if_false


ScenarioPartType = Union[
    SystemMessage,
    HumanMessage,
    HumanMultimodalMessage,
    AIMessage,
    ToolMessage,
    BaseMessage,
    ConditionalScenario,
    FutureAiMessage,
    AgentLoop,
]
ScenarioType = Sequence[ScenarioPartType]


class ScenarioRunner:
    """
    The ScenarioRunner class is responsible for running a given scenario. It iterates over the scenario and executes the
    actions defined in the scenario.

    Args:
        scenario (ScenarioType): The scenario to run.
        llm (BaseChatModel): The language model to use for the scenario
        ros_node (RaiNode): The ROS2 node to use for the scenario
        llm_type (Literal["openai", "bedrock"]): The type of language model to use for the scenario
        scenario_name (str, optional): The name of the scenario. Defaults to "".
        logging_level (int, optional): The logging level to use for the scenario. Defaults to logging.WARNING.
        log_usage (bool, optional): Whether to log usage. Defaults to True.
        use_cache (bool, optional): Whether to use the cache. Defaults to True.
        ros_spin_time (int, optional): The ROS2 spin time for every LLM iteration. Defaults to 1s.
    """

    def __init__(
        self,
        scenario: ScenarioType,
        llm: BaseChatModel,
        ros_node: RaiNode,
        llm_type: Literal["openai", "bedrock"],
        scenario_name: str = "",
        logging_level: int = logging.WARNING,
        log_usage: bool = True,
        use_cache: bool = True,
        ros_spin_time: int = 1,
    ):
        self.ros_node = ros_node

        self.ros_single_spin_time = 0.1
        self.ros_spins_per_iter = int(ros_spin_time / self.ros_single_spin_time)

        self.scenario_name = scenario_name
        self.scenario = scenario
        self.log_usage = log_usage
        self.llm = llm
        self.llm_type = llm_type
        self.logging_level = logging_level
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(self.logging_level)
        self.history: List[BaseMessage] = []
        now = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S.%f")
        self.logs_dir = os.path.join("logs", self.llm.__class__.__name__ + now)
        self.use_cache = use_cache

        if self.use_cache:
            cache_host = os.getenv("REDIS_CACHE_HOST")
            if cache_host is None:
                self.logger.warning("REDIS_CACHE_HOST is not set. Disabling cache.")
            else:
                set_llm_cache(RedisCache(redis_=Redis.from_url(cache_host)))
                self.logger.warning("Cache is enabled!")

        self.invoke_config: RunnableConfig = {}
        self.langfuse_handler = None
        if self.log_usage:
            public_key = os.getenv("LANGFUSE_PK")
            secret_key = os.getenv("LANGFUSE_SK")
            host = os.getenv("LANGFUSE_HOST")
            if not all((public_key, secret_key, host)):
                raise ValueError(
                    "Please provide LANGFUSE_PK, LANGFUSE_SK, LANGFUSE_HOST in the environment."
                )
            self.langfuse_handler = CallbackHandler(
                public_key=public_key,
                secret_key=secret_key,
                host=host,
                trace_name=scenario_name or "unknown scenario",
                tags=["scenario_runner"],
            )
            self.invoke_config["callbacks"] = [self.langfuse_handler]

    def run(self):
        self.logger.info("Starting conversation.")

        try:
            self._run(self.scenario)

            self.logger.info("Conversation completed.")
        finally:
            self.save_to_html()
        return self.history

    def ros_spin(self):
        for _ in range(self.ros_spins_per_iter):
            if rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=self.ros_single_spin_time)

    def _run(self, scenario: ScenarioType):
        """Recursively run the scenario."""

        for msg in scenario:
            if isinstance(msg, (HumanMessage, AIMessage, ToolMessage, SystemMessage)):
                self.history.append(msg)
            elif isinstance(msg, FutureAiMessage):
                llm_with_tools = self.llm.bind_tools(msg.tools)
                ai_msg = cast(
                    AIMessage,
                    llm_with_tools.invoke(self.history, config=self.invoke_config),
                )
                self.history.append(ai_msg)
                self.history = run_requested_tools(
                    ai_msg, msg.tools, self.history, llm_type=self.llm_type
                )
            elif isinstance(msg, AgentLoop):
                self.logger.info(
                    f"looping agent actions until {msg.stop_tool}. max {msg.stop_iters} loops."
                )
                llm_with_tools = self.llm.bind_tools(msg.tools)
                for _ in range(msg.stop_iters):
                    # if the last message is from the AI, we need to add a human message to continue the agent loop
                    # otherwise the bedrock model will not be able to continue the conversation
                    self.ros_spin()
                    if self.history[-1].type == "ai":
                        self.history.append(
                            HumanMessage(
                                content="Thank you. Please continue your mision using tools."
                            )
                        )
                    ai_msg = cast(
                        AIMessage,
                        llm_with_tools.invoke(self.history, config=self.invoke_config),
                    )
                    self.history.append(ai_msg)
                    self.history = run_requested_tools(
                        ai_msg, msg.tools, self.history, llm_type=self.llm_type
                    )
                    break_loop = False
                    for tool_call in ai_msg.tool_calls:
                        if tool_call["name"] == msg.stop_tool:
                            break_loop = True
                            break
                    if break_loop:
                        break
            elif isinstance(msg, ConditionalScenario):
                new_scenario = msg(self.history)
                self._run(new_scenario)
            else:
                raise ValueError(f"Unknown message type: {type(msg)}")
            self.logger.debug(
                f"Last message: {self.history[-1].type}:{self.history[-1].content}"
            )

    def save_to_html(self, folder: str = "") -> str:
        saver = HistorySaver(self.history, self.logs_dir)
        out_file = saver.save_to_html(folder=folder)
        self.logger.info(f"Conversation saved to: {out_file}")
        return out_file
