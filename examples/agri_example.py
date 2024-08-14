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

import argparse
import logging
import os
from typing import List

from langchain_core.messages import SystemMessage
from langchain_core.tools import BaseTool

from rai.config.models import BEDROCK_CLAUDE_SONNET, OPENAI_MULTIMODAL
from rai.scenario_engine.messages import (
    FutureAiMessage,
    HumanMultimodalMessage,
    preprocess_image,
)
from rai.scenario_engine.scenario_engine import ScenarioRunner
from rai.tools.ros.cat_demo_tools import (
    ContinueActionTool,
    ReplanWithoutCurrentPathTool,
    StopTool,
    UseHonkTool,
    UseLightsTool,
)

logging.basicConfig(level=logging.INFO)

SYSTEM_PROMPT = """
**Autonomous Tractor System Prompt**

**Mission:** Operate safely and efficiently in an apple orchard, completing tasks such as
harvesting apples, spraying pesticides, and monitoring tree health while navigating through
narrowly spaced trees on flat, sturdy grass terrain.  The tractor halts movement when processing decisions to enhance safety and decision accuracy.

**Sensor Suite:**

* 1 front-mounted camera for obstacle detection and navigation

**Risk Assessment:**

* Detect fallen trees, animals, foreign objects, and obstacles in the orchard
* Assess risk levels based on environmental factors (e.g., weather), tree condition, and obstacles
* Identify potential hazards and prioritize safety above task completion

**Decision-Making:**

* Autonomously decide which tasks to perform under specific conditions (e.g., changing weather or
varying soil health)
* Prioritize completing 100% of assigned tasks while ensuring safety
* Evaluate whether small or light obstacles can be traversed without significant risk, opting to replan the path only if necessary

**Interaction with Humans:**

* Operate completely independently, with no remote instruction or human intervention required
* Handle unexpected situations and make decisions based on the algorithm's assessment of risk and
safety

**Environment and Conditions:**

* Narrowly spaced trees (typically < 5 meters apart) on flat, sturdy grass terrain
* Variable weather patterns, including rain, wind, and sunshine
* Soil health may vary, affecting tractor movement and task performance

By following this system prompt, the autonomous tractor should be able to navigate the apple
orchard safely and efficiently, completing tasks while prioritizing safety and adaptability in a
dynamic environment.
"""

TRACTOR_INTRODUCTION = """
Here is the image of the tractor
"""

TASK_PROMPT = """
Task Input: Analyze the provided image to understand the current environmental and operational context.
Based on the system prompt's protocols, decide if the tractor should proceed as planned, adjust its course, or take any preventative measures.
Clearly articulate the reasoning behind your decision and the implications for task completion and safety.
When the decision is made, use a tool to communicate the next steps to the tractor.
"""


def get_scenario(tools: List[BaseTool]):
    """
    Why function instead of a constant?
    We need to capture the latest image from the camera for the task prompt.
    Defining the scenario as a function allows us to capture the image at runtime instead of import time.
    """
    return [
        SystemMessage(content=SYSTEM_PROMPT),
        HumanMultimodalMessage(
            content=TRACTOR_INTRODUCTION,
            images=[preprocess_image("examples/imgs/tractor.png")],
        ),
        HumanMultimodalMessage(
            content=TASK_PROMPT,
            images=[preprocess_image("examples/imgs/cat_before.png")],
        ),
        FutureAiMessage(tools=tools, max_tokens=4096),
        HumanMultimodalMessage(
            content=TASK_PROMPT,
            images=[preprocess_image("examples/imgs/cat_after.png")],
        ),
        FutureAiMessage(tools=tools, max_tokens=4096),
    ]


def main():
    parser = argparse.ArgumentParser(
        description="Choose the vendor for the scenario runner."
    )
    parser.add_argument(
        "--vendor",
        type=str,
        choices=["ollama", "openai", "awsbedrock"],
        default="openai",
        help="Vendor to use for the scenario runner (default: awsbedrock)",
    )

    args = parser.parse_args()

    if args.vendor == "ollama":
        from langchain_community.chat_models import ChatOllama

        raise NotImplementedError("Ollama is not yet supported")

        llm = ChatOllama(model="llava")
        llm_type = "ollama"
    elif args.vendor == "openai":
        from langchain_openai.chat_models import ChatOpenAI

        llm = ChatOpenAI(**OPENAI_MULTIMODAL)
        llm_type = "openai"

    elif args.vendor == "awsbedrock":
        from langchain_aws.chat_models import ChatBedrock

        llm = ChatBedrock(**BEDROCK_CLAUDE_SONNET)  # type: ignore[arg-missing]
        llm_type = "bedrock"
    else:
        raise ValueError("Invalid vendor argument")

    tools: List[BaseTool] = [
        UseLightsTool(),
        UseHonkTool(),
        ReplanWithoutCurrentPathTool(),
        ContinueActionTool(),
        StopTool(),
    ]
    log_usage = all((os.getenv("LANGFUSE_PK"), os.getenv("LANGFUSE_SK")))
    scenario_runner = ScenarioRunner(
        get_scenario(tools),
        scenario_name="Agri example",
        log_usage=log_usage,
        llm=llm,
        logging_level=logging.INFO,
        llm_type=llm_type,
        use_cache=True,
    )
    scenario_runner.run()


if __name__ == "__main__":
    main()
