import logging
import os
from typing import List

from langchain.tools import tool
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.tools import BaseTool
from langchain_openai import ChatOpenAI

from rai.communication.ros_communication import SingleImageGrabber
from rai.config.models import OPENAI_MULTIMODAL
from rai.scenario_engine.messages import AgentLoop
from rai.scenario_engine.scenario_engine import ScenarioPartType, ScenarioRunner
from rai.tools.ros.cat_demo_tools import FinishTool
from rai.tools.ros.native import (
    Ros2GetTopicsNamesAndTypesTool,
    Ros2PubMessageTool,
    Ros2ShowMsgInterfaceTool,
)
from rai.tools.ros.native_actions import (
    Ros2ActionRunner,
    Ros2CancelAction,
    Ros2CheckActionResults,
    Ros2GetActionNamesAndTypesTool,
    Ros2GetRegisteredActions,
)
from rai.tools.ros.tools import GetCurrentPositionTool, GetOccupancyGridTool
from rai.tools.time import sleep_max_5s


@tool
def get_camera_image() -> str:
    """Gets the current image from the robots camera."""
    grabber = SingleImageGrabber(topic="/camera/camera/color/image_raw", timeout_sec=5)
    msg = grabber.grab_message()
    base64_image = grabber.postprocess(msg)
    return {"content": "Image grabbed successfully", "images": [base64_image]}


def run_task(rai_node, task, history):
    log_usage = all((os.getenv("LANGFUSE_PK"), os.getenv("LANGFUSE_SK")))
    llm = ChatOpenAI(**OPENAI_MULTIMODAL)
    tools: List[BaseTool] = [
        Ros2GetActionNamesAndTypesTool(node=rai_node),
        Ros2GetTopicsNamesAndTypesTool(node=rai_node),
        Ros2PubMessageTool(node=rai_node),
        Ros2ShowMsgInterfaceTool(),
        Ros2ActionRunner(node=rai_node),
        Ros2CheckActionResults(node=rai_node),
        Ros2CancelAction(node=rai_node),
        # Ros2ListActionFeedbacks(node=rai_node),
        Ros2GetRegisteredActions(node=rai_node),
        GetCurrentPositionTool(),
        GetOccupancyGridTool(),
        get_camera_image,
        sleep_max_5s,
        FinishTool(),
    ]

    actions = Ros2GetActionNamesAndTypesTool(node=rai_node)._run()
    scenario: List[ScenarioPartType] = [
        SystemMessage(
            content="You are an autonomous agent. Your main goal is to fulfill the user's requests. "
            "Do not make assumptions about the environment you are currently in. "
            "Use the tooling provided to gather information about the environment."
            "To drive to goals described in tasks please grab camera image, get current position and occupancy grid"
            "Based on that information figure out the goal pose and use ros2 interface to drive"
            f"Here are available ros2 actions: {actions}"
            "When the task is finished, use FinishTool. You will be given a robot state frequently. Use the state to accomplish the task."
            "Use the GetOccupancyGridTool to get the current occupancy grid."
        ),
        HumanMessage(content=f"Task:  {task}"),
        AgentLoop(
            tools=tools, stop_tool=FinishTool().__class__.__name__, stop_iters=50
        ),
    ]

    runner = ScenarioRunner(
        scenario,
        llm,
        ros_node=rai_node,
        llm_type="openai",
        scenario_name="Nav2 example",
        log_usage=log_usage,
        logging_level=logging.INFO,
        history=history,
    )

    runner.run()
