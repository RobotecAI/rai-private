import logging
import os
from typing import List

from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.tools import BaseTool
from langchain_openai import ChatOpenAI

from rai.config.models import OPENAI_MULTIMODAL
from rai.scenario_engine.messages import AgentLoop
from rai.scenario_engine.scenario_engine import ScenarioPartType, ScenarioRunner
from rai.tools.ros.native import Ros2ShowMsgInterfaceTool
from rai.tools.ros.native_actions import (
    Ros2ActionRunner,
    Ros2CancelAction,
    Ros2CheckActionResults,
    Ros2GetActionNamesAndTypesTool,
    Ros2GetRegisteredActions,
)
from rai.tools.ros.tools import GetCurrentPositionTool, GetOccupancyGridTool
from rai.tools.time import sleep_max_5s


def run_task(rai_node, task):

    log_usage = all((os.getenv("LANGFUSE_PK"), os.getenv("LANGFUSE_SK")))
    llm = ChatOpenAI(**OPENAI_MULTIMODAL)
    tools: List[BaseTool] = [
        Ros2GetActionNamesAndTypesTool(node=rai_node),
        # Ros2GetTopicsNamesAndTypesTool(node=rai_node),
        # Ros2PubMessageTool(node=rai_node),
        Ros2ShowMsgInterfaceTool(),
        Ros2ActionRunner(node=rai_node),
        Ros2CheckActionResults(node=rai_node),
        Ros2CancelAction(node=rai_node),
        # Ros2ListActionFeedbacks(node=rai_node),
        Ros2GetRegisteredActions(node=rai_node),
        GetCurrentPositionTool(),
        GetOccupancyGridTool(),
        sleep_max_5s,
    ]

    scenario: List[ScenarioPartType] = [
        SystemMessage(
            content="You are an autonomous agent. Your main goal is to fulfill the user's requests. "
            "Do not make assumptions about the environment you are currently in. "
            "Use the tooling provided to gather information about the environment."
        ),
        HumanMessage(content=task),
        AgentLoop(tools=tools, stop_tool=None, stop_iters=50),
    ]

    runner = ScenarioRunner(
        scenario,
        llm,
        ros_node=rai_node,
        llm_type="openai",
        scenario_name="Nav2 example",
        log_usage=log_usage,
        logging_level=logging.INFO,
    )

    runner.run()
